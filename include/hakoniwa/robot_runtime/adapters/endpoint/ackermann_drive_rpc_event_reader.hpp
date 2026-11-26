#pragma once

#include "runtime/plant/actuator_plant.hpp"

#include <cstddef>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

struct AckermannDriveCommand {
    double steering_angle_rad {0.0};
    double steering_angle_velocity_rad_s {0.0};
    double speed_m_s {0.0};
    double acceleration_m_s2 {0.0};
    double jerk_m_s3 {0.0};
};

} // namespace hakoniwa::robot_runtime::runtime

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

/** State returned by one AckermannStep request after physics step k. */
struct AckermannStepState {
    std::uint64_t simulation_time_usec {0};
    runtime::Vector3State position;
    runtime::QuaternionState orientation;
    runtime::Vector3State linear_velocity;
    runtime::Vector3State angular_velocity;
};

[[nodiscard]] std::vector<std::byte> encode_ackermann_command(
    const runtime::AckermannDriveCommand& command);
[[nodiscard]] bool decode_ackermann_command(
    std::span<const std::byte> payload,
    runtime::AckermannDriveCommand& command) noexcept;
[[nodiscard]] std::vector<std::byte> encode_ackermann_state(
    const AckermannStepState& state);
[[nodiscard]] bool decode_ackermann_state(
    std::span<const std::byte> payload,
    AckermannStepState& state) noexcept;

/** Common `<domain>/<operation>/<robot>` service-name convention. */
[[nodiscard]] std::string make_robot_rpc_service_name(
    std::string_view domain,
    std::string_view operation,
    std::string_view robot_name);

/** `CarService/AckermannStep/<vehicle>`. */
[[nodiscard]] std::string ackermann_step_service_name(
    std::string_view vehicle_name);

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
