#pragma once

#include "runtime/publisher/state_publisher.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace hakoniwa::robot_runtime::runtime {

struct ImuOutput {
    std::uint64_t simulation_time_usec {0};
    std::string frame_id;
    std::array<double, 3> angular_velocity {0.0, 0.0, 0.0};
    std::array<double, 3> linear_acceleration {0.0, 0.0, 0.0};
};

class IImuWriter {
public:
    virtual ~IImuWriter() = default;
    [[nodiscard]] virtual bool send(const ImuOutput& output) noexcept = 0;
    virtual void reset() noexcept = 0;
};

/**
 * Publishes the measured angular velocity and specific force of one IMU as a
 * sensor_msgs/Imu, rate-limited on simulation time. frame_id is the sensor
 * name. An IMU does not measure orientation, so the message carries none
 * (orientation_covariance[0] = -1, the ROS convention). Nothing is sent
 * before both the gyroscope and the accelerometer have measured.
 */
class SensorImuPublisher final : public IStatePublisher {
public:
    SensorImuPublisher(
        std::string component_id,
        std::string sensor_name,
        double update_rate_hz,
        std::shared_ptr<IImuWriter> writer);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] ComponentStatus publish(
        const RobotState& state,
        const RuntimeStepContext& context) noexcept override;
    void reset() noexcept override;

private:
    std::string component_id_;
    std::string sensor_name_;
    std::uint64_t update_period_usec_ {0};
    std::shared_ptr<IImuWriter> writer_;
    std::uint64_t next_publish_time_usec_ {0};
    bool publish_ready_ {true};
};

} // namespace hakoniwa::robot_runtime::runtime
