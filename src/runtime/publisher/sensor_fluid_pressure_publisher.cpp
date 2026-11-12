#include "runtime/publisher/sensor_fluid_pressure_publisher.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

SensorFluidPressurePublisher::SensorFluidPressurePublisher(
    std::string component_id,
    std::string sensor_name,
    const double update_rate_hz,
    std::shared_ptr<IFluidPressureWriter> writer)
    : component_id_(std::move(component_id))
    , sensor_name_(std::move(sensor_name))
    , writer_(std::move(writer))
{
    if (component_id_.empty() || sensor_name_.empty() || writer_ == nullptr) {
        throw std::invalid_argument(
            "sensor fluid pressure publisher requires an ID, a sensor, and a writer");
    }
    if (!std::isfinite(update_rate_hz) || update_rate_hz <= 0.0) {
        throw std::invalid_argument("sensor fluid pressure update rate must be finite and positive");
    }
    const double period_usec = 1'000'000.0 / update_rate_hz;
    if (!std::isfinite(period_usec) || period_usec < 1.0
        || period_usec > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
        throw std::invalid_argument("sensor fluid pressure update period is not representable");
    }
    update_period_usec_ = static_cast<std::uint64_t>(std::llround(period_usec));
}

std::string_view SensorFluidPressurePublisher::id() const noexcept
{
    return component_id_;
}

ComponentStatus SensorFluidPressurePublisher::publish(
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
    const SensorReading* reading = nullptr;
    for (const auto& candidate : state.sensor_readings) {
        if (candidate.sensor_name == sensor_name_) {
            reading = &candidate;
            break;
        }
    }
    if (reading == nullptr) {
        return {component_id_, ComponentState::Error,
            "RobotState is missing sensor: " + sensor_name_};
    }
    if (!reading->has_sample) {
        return {component_id_, ComponentState::Ready, {}};
    }
    if (!reading->has_pressure) {
        return {component_id_, ComponentState::Error,
            "sensor " + sensor_name_ + " is not a barometer"};
    }
    FluidPressureOutput output;
    // ROS: the stamp is when the value was measured.
    output.simulation_time_usec = reading->sample_time_usec;
    output.frame_id = sensor_name_;
    output.pressure_pa = reading->pressure_pa;
    if (!writer_->send(output)) {
        return {component_id_, ComponentState::Degraded,
            "failed to send sensor fluid pressure output"};
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

void SensorFluidPressurePublisher::reset() noexcept
{
    next_publish_time_usec_ = 0;
    publish_ready_ = true;
    writer_->reset();
}

} // namespace hakoniwa::robot_runtime::runtime
