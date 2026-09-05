#include "runtime/controller/joint_trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {
namespace {

const std::vector<double>& values_for(
    const JointTrajectoryPoint& point,
    const ActuatorCommandType type)
{
    switch (type) {
    case ActuatorCommandType::Position:
        return point.positions;
    case ActuatorCommandType::Velocity:
        return point.velocities;
    case ActuatorCommandType::Effort:
        return point.effort;
    }
    return point.positions;
}

} // namespace

JointTrajectoryController::JointTrajectoryController(
    std::string controller_id,
    std::string source_id,
    std::vector<TrajectoryActuatorBinding> bindings)
    : controller_id_(std::move(controller_id))
    , source_id_(std::move(source_id))
    , configured_bindings_(std::move(bindings))
{
    if (controller_id_.empty() || source_id_.empty()
        || configured_bindings_.empty()) {
        throw std::invalid_argument(
            "trajectory controller requires IDs and actuator bindings");
    }
    std::unordered_set<std::string> joints;
    std::unordered_set<std::string> actuators;
    for (const auto& binding : configured_bindings_) {
        if (binding.joint_name.empty() || binding.actuator_id.empty()
            || !joints.insert(binding.joint_name).second
            || !actuators.insert(binding.actuator_id).second) {
            throw std::invalid_argument(
                "trajectory bindings must be non-empty and unique");
        }
    }
}

std::string_view JointTrajectoryController::id() const noexcept
{
    return controller_id_;
}

std::string_view JointTrajectoryController::source_id() const noexcept
{
    return source_id_;
}

bool JointTrajectoryController::accepts(
    const IControllerInput& input) const noexcept
{
    return input.type_name() == "trajectory_msgs/JointTrajectory";
}

ControllerOutput JointTrajectoryController::update(
    std::shared_ptr<const IControllerInput> input,
    const RobotState& current_state,
    const RuntimeStepContext& context)
{
    (void)current_state;
    ComponentStatus status {controller_id_, ComponentState::WaitingForInput, {}};
    bool activated_this_step = false;

    if (input != nullptr) {
        const auto trajectory =
            std::dynamic_pointer_cast<const JointTrajectoryInput>(input);
        if (trajectory == nullptr
            || trajectory->metadata().source_id != source_id_) {
            return {
                controller_id_,
                {controller_id_, ComponentState::Error,
                    "invalid trajectory controller input"},
                {},
            };
        }
        const auto error = validate_and_activate(
            trajectory->trajectory(), context);
        if (!error.empty()) {
            status = {controller_id_, ComponentState::Degraded, error};
        } else {
            activated_this_step = true;
        }
    }

    // A time-based trajectory must not keep advancing invisibly while another
    // logical control is selected. Once a previously active trajectory is
    // preempted, discard it and require a new trajectory command to resume AUTO.
    if (active_ && !activated_this_step
        && (!context.previous_selected_control_id.has_value()
            || *context.previous_selected_control_id != controller_id_)) {
        cancel();
    }

    if (!active_) {
        return {controller_id_, std::move(status), {}};
    }
    if (status.state != ComponentState::Degraded) {
        status.state = ComponentState::Ready;
    }
    return {
        controller_id_,
        std::move(status),
        interpolate(context),
    };
}

void JointTrajectoryController::reset(const RobotState& current_state)
{
    (void)current_state;
    cancel();
}

void JointTrajectoryController::cancel() noexcept
{
    active_bindings_.clear();
    active_trajectory_ = {};
    start_time_usec_ = 0;
    active_ = false;
}

std::string JointTrajectoryController::validate_and_activate(
    const JointTrajectory& trajectory,
    const RuntimeStepContext& context)
{
    if (trajectory.joint_names.empty()) {
        return "JointTrajectory has no joint_names";
    }
    if (trajectory.points.empty()) {
        return "JointTrajectory has no points";
    }
    std::unordered_map<std::string, std::size_t> indices;
    for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index) {
        const auto& name = trajectory.joint_names[index];
        if (name.empty() || !indices.emplace(name, index).second) {
            return "JointTrajectory contains an empty or duplicate joint name";
        }
    }
    std::uint64_t previous_time = 0;
    bool first = true;
    for (const auto& point : trajectory.points) {
        if (!first && point.time_from_start_usec < previous_time) {
            return "JointTrajectory point times must be non-decreasing";
        }
        previous_time = point.time_from_start_usec;
        first = false;
    }

    std::vector<ActiveBinding> bindings;
    bindings.reserve(configured_bindings_.size());
    for (const auto& configured : configured_bindings_) {
        const auto iterator = indices.find(configured.joint_name);
        if (iterator == indices.end()) {
            return "JointTrajectory is missing configured joint: "
                + configured.joint_name;
        }
        for (const auto& point : trajectory.points) {
            const auto& values = values_for(point, configured.command_type);
            if (iterator->second >= values.size()
                || !std::isfinite(values[iterator->second])) {
                return "JointTrajectory point has missing or non-finite data for joint: "
                    + configured.joint_name;
            }
        }
        bindings.push_back({configured, iterator->second});
    }
    active_trajectory_ = trajectory;
    active_bindings_ = std::move(bindings);
    start_time_usec_ = context.simulation_time_usec;
    active_ = true;
    return {};
}

std::vector<ActuatorCommand> JointTrajectoryController::interpolate(
    const RuntimeStepContext& context) const
{
    const std::uint64_t elapsed = context.simulation_time_usec >= start_time_usec_
        ? context.simulation_time_usec - start_time_usec_
        : 0;
    const auto& points = active_trajectory_.points;
    const JointTrajectoryPoint* lower = &points.front();
    const JointTrajectoryPoint* upper = &points.front();
    double alpha = 0.0;
    if (points.size() > 1 && elapsed > points.front().time_from_start_usec) {
        if (elapsed >= points.back().time_from_start_usec) {
            lower = &points.back();
            upper = lower;
        } else {
            const auto iterator = std::upper_bound(
                points.begin(), points.end(), elapsed,
                [](const std::uint64_t time, const JointTrajectoryPoint& point) {
                    return time < point.time_from_start_usec;
                });
            upper = &*iterator;
            lower = &*(iterator - 1);
            const auto span = upper->time_from_start_usec - lower->time_from_start_usec;
            alpha = span > 0
                ? static_cast<double>(elapsed - lower->time_from_start_usec)
                    / static_cast<double>(span)
                : 1.0;
        }
    }

    std::vector<ActuatorCommand> commands;
    commands.reserve(active_bindings_.size());
    for (const auto& active : active_bindings_) {
        const auto& from_values = values_for(*lower, active.binding.command_type);
        const auto& to_values = values_for(*upper, active.binding.command_type);
        const double from = from_values[active.trajectory_index];
        const double value = from
            + (to_values[active.trajectory_index] - from) * alpha;
        commands.push_back({
            active.binding.actuator_id,
            active.binding.command_type,
            value,
            source_id_,
            context.simulation_time_usec,
            context.simulation_time_usec + context.delta_time_usec,
        });
    }
    return commands;
}

} // namespace hakoniwa::robot_runtime::runtime
