#include "runtime/publisher/joint_state_publisher.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

JointStatePublisher::JointStatePublisher(
    std::string component_id,
    std::vector<JointStateOutputBinding> bindings,
    const double update_rate_hz,
    std::shared_ptr<IJointStateWriter> writer)
    : component_id_(std::move(component_id))
    , bindings_(std::move(bindings))
    , writer_(std::move(writer))
{
    if (component_id_.empty() || bindings_.empty() || writer_ == nullptr) {
        throw std::invalid_argument(
            "joint state publisher requires an ID, bindings, and writer");
    }
    if (!std::isfinite(update_rate_hz) || update_rate_hz <= 0.0) {
        throw std::invalid_argument("joint state update rate must be finite and positive");
    }
    const double period_usec = 1'000'000.0 / update_rate_hz;
    if (!std::isfinite(period_usec) || period_usec < 1.0
        || period_usec > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
        throw std::invalid_argument("joint state update period is not representable");
    }
    update_period_usec_ = static_cast<std::uint64_t>(std::llround(period_usec));
    std::unordered_set<std::string> names;
    std::unordered_set<std::string> actuator_ids;
    for (const auto& binding : bindings_) {
        if (binding.joint_name.empty() || binding.actuator_id.empty()
            || !names.insert(binding.joint_name).second
            || !actuator_ids.insert(binding.actuator_id).second) {
            throw std::invalid_argument("joint state bindings must be non-empty and unique");
        }
    }
}

std::string_view JointStatePublisher::id() const noexcept
{
    return component_id_;
}

ComponentStatus JointStatePublisher::publish(
    const RobotState& state,
    const RuntimeStepContext& context) noexcept
{
    if (state.sample_time_usec != context.simulation_time_usec) {
        return {component_id_, ComponentState::Error,
            "RobotState sample time does not match runtime context"};
    }
    if (!publish_ready_ && context.simulation_time_usec < next_publish_time_usec_) {
        return {component_id_, ComponentState::Ready, {}};
    }
    std::unordered_map<std::string_view, const ActuatorState*> states;
    states.reserve(state.actuators.size());
    for (const auto& actuator : state.actuators) {
        if (!states.emplace(actuator.actuator_id, &actuator).second) {
            return {component_id_, ComponentState::Error,
                "RobotState contains duplicate actuator IDs"};
        }
    }

    JointStateOutput output;
    output.simulation_time_usec = state.sample_time_usec;
    output.names.reserve(bindings_.size());
    output.position.reserve(bindings_.size());
    output.velocity.reserve(bindings_.size());
    output.effort.reserve(bindings_.size());
    for (const auto& binding : bindings_) {
        const auto iterator = states.find(binding.actuator_id);
        if (iterator == states.end()) {
            return {component_id_, ComponentState::Error,
                "RobotState is missing actuator: " + binding.actuator_id};
        }
        const auto& actuator = *iterator->second;
        output.names.push_back(binding.joint_name);
        output.position.push_back(actuator.position_rad);
        output.velocity.push_back(actuator.velocity_rad_per_sec);
        output.effort.push_back(actuator.effort_newton_metre);
    }
    if (!writer_->send(output)) {
        return {component_id_, ComponentState::Degraded,
            "failed to send joint state output"};
    }
    publish_ready_ = false;
    if (context.simulation_time_usec
        > std::numeric_limits<std::uint64_t>::max() - update_period_usec_) {
        next_publish_time_usec_ = std::numeric_limits<std::uint64_t>::max();
    } else {
        next_publish_time_usec_ = context.simulation_time_usec + update_period_usec_;
    }
    return {component_id_, ComponentState::Ready, {}};
}

void JointStatePublisher::reset() noexcept
{
    next_publish_time_usec_ = 0;
    publish_ready_ = true;
    writer_->reset();
}

} // namespace hakoniwa::robot_runtime::runtime
