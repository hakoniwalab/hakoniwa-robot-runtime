#include "runtime/publisher/sensor_joint_state_publisher.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

SensorJointStatePublisher::SensorJointStatePublisher(
    std::string component_id,
    std::vector<std::string> sensor_names,
    const double update_rate_hz,
    std::shared_ptr<IJointStateWriter> writer)
    : component_id_(std::move(component_id))
    , sensor_names_(std::move(sensor_names))
    , writer_(std::move(writer))
{
    if (component_id_.empty() || sensor_names_.empty() || writer_ == nullptr) {
        throw std::invalid_argument(
            "sensor joint state publisher requires an ID, sensors, and writer");
    }
    if (!std::isfinite(update_rate_hz) || update_rate_hz <= 0.0) {
        throw std::invalid_argument("sensor joint state update rate must be finite and positive");
    }
    const double period_usec = 1'000'000.0 / update_rate_hz;
    if (!std::isfinite(period_usec) || period_usec < 1.0
        || period_usec > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
        throw std::invalid_argument("sensor joint state update period is not representable");
    }
    update_period_usec_ = static_cast<std::uint64_t>(std::llround(period_usec));
    std::unordered_set<std::string> names;
    for (const auto& name : sensor_names_) {
        if (name.empty() || !names.insert(name).second) {
            throw std::invalid_argument("sensor names must be non-empty and unique");
        }
    }
}

std::string_view SensorJointStatePublisher::id() const noexcept
{
    return component_id_;
}

ComponentStatus SensorJointStatePublisher::publish(
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
    std::unordered_map<std::string_view, const SensorReading*> readings;
    readings.reserve(state.sensor_readings.size());
    for (const auto& reading : state.sensor_readings) {
        readings.emplace(reading.sensor_name, &reading);
    }

    JointStateOutput output;
    output.simulation_time_usec = state.sample_time_usec;
    output.names.reserve(sensor_names_.size());
    output.position.reserve(sensor_names_.size());
    output.velocity.reserve(sensor_names_.size());
    for (const auto& name : sensor_names_) {
        const auto found = readings.find(name);
        if (found == readings.end()) {
            return {component_id_, ComponentState::Error,
                "RobotState is missing sensor: " + name};
        }
        if (!found->second->has_sample) {
            // Not every sensor has measured yet; try again next step.
            return {component_id_, ComponentState::Ready, {}};
        }
        output.names.push_back(found->second->mount_name);
        output.position.push_back(found->second->position);
        output.velocity.push_back(found->second->velocity);
        if (found->second->has_effort) {
            output.effort.push_back(found->second->effort);
        }
    }
    if (!output.effort.empty() && output.effort.size() != output.names.size()) {
        return {component_id_, ComponentState::Error,
            "sensors of one JointState differ in measuring the joint torque"};
    }
    if (!writer_->send(output)) {
        return {component_id_, ComponentState::Degraded,
            "failed to send sensor joint state output"};
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

void SensorJointStatePublisher::reset() noexcept
{
    next_publish_time_usec_ = 0;
    publish_ready_ = true;
    writer_->reset();
}

} // namespace hakoniwa::robot_runtime::runtime
