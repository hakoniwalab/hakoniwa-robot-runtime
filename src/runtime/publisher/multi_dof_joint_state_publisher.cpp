#include "runtime/publisher/multi_dof_joint_state_publisher.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

MultiDofJointStatePublisher::MultiDofJointStatePublisher(
    std::string component_id,
    std::string frame_id,
    std::vector<MultiDofOutputBinding> bindings,
    const double update_rate_hz,
    std::shared_ptr<IMultiDofJointStateWriter> writer)
    : component_id_(std::move(component_id))
    , frame_id_(std::move(frame_id))
    , bindings_(std::move(bindings))
    , writer_(std::move(writer))
{
    if (component_id_.empty() || frame_id_.empty()
        || bindings_.empty() || writer_ == nullptr) {
        throw std::invalid_argument(
            "MultiDOF publisher requires IDs, bindings, and writer");
    }
    const double period_usec = 1'000'000.0 / update_rate_hz;
    if (!std::isfinite(period_usec) || period_usec < 1.0
        || period_usec > static_cast<double>(
            std::numeric_limits<std::int64_t>::max())) {
        throw std::invalid_argument(
            "MultiDOF update period is not representable");
    }
    update_period_usec_ = static_cast<std::uint64_t>(std::llround(period_usec));
    std::unordered_set<std::string> names;
    std::unordered_set<std::string> body_ids;
    for (const auto& binding : bindings_) {
        if (binding.name.empty() || binding.body_id.empty()
            || !names.insert(binding.name).second
            || !body_ids.insert(binding.body_id).second) {
            throw std::invalid_argument(
                "MultiDOF bindings must be non-empty and unique");
        }
    }
}

std::string_view MultiDofJointStatePublisher::id() const noexcept
{
    return component_id_;
}

ComponentStatus MultiDofJointStatePublisher::publish(
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
    std::unordered_map<std::string_view, const BodyState*> states;
    for (const auto& body : state.bodies) {
        if (!states.emplace(body.body_id, &body).second) {
            return {component_id_, ComponentState::Error,
                "RobotState contains duplicate body IDs"};
        }
    }

    MultiDofJointStateOutput output;
    output.simulation_time_usec = state.sample_time_usec;
    output.frame_id = frame_id_;
    output.bodies.reserve(bindings_.size());
    for (const auto& binding : bindings_) {
        const auto iterator = states.find(binding.body_id);
        if (iterator == states.end()) {
            return {component_id_, ComponentState::Error,
                "RobotState is missing body: " + binding.body_id};
        }
        const auto& body = *iterator->second;
        output.bodies.push_back({
            binding.name,
            body.position,
            body.orientation,
            body.linear_velocity,
            body.angular_velocity,
        });
    }
    if (!writer_->send(output)) {
        return {component_id_, ComponentState::Degraded,
            "failed to send MultiDOF body state output"};
    }
    publish_ready_ = false;
    next_publish_time_usec_ = context.simulation_time_usec
        > std::numeric_limits<std::uint64_t>::max() - update_period_usec_
        ? std::numeric_limits<std::uint64_t>::max()
        : context.simulation_time_usec + update_period_usec_;
    return {component_id_, ComponentState::Ready, {}};
}

void MultiDofJointStatePublisher::reset() noexcept
{
    next_publish_time_usec_ = 0;
    publish_ready_ = true;
    writer_->reset();
}

} // namespace hakoniwa::robot_runtime::runtime
