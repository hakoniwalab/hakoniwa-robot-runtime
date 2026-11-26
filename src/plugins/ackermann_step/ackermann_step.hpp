#pragma once

#include "hakoniwa/robot_runtime/plugin/service_plugin.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace hakoniwa::robot_runtime::plugins::ackermann_step {

struct AckermannDriveCommand {
    double steering_angle_rad {0};
    double steering_angle_velocity_rad_s {0};
    double speed_m_s {0};
    double acceleration_m_s2 {0};
    double jerk_m_s3 {0};
};

struct Request {
    std::uint64_t step;
    AckermannDriveCommand drive;
    double tire_friction;
};

[[nodiscard]] std::optional<Request> decode_request(std::span<const std::byte> request);
[[nodiscard]] bool valid_topic(std::span<const std::byte> pdu);
[[nodiscard]] std::optional<AckermannDriveCommand> decode_topic(std::span<const std::byte> pdu);

} // namespace hakoniwa::robot_runtime::plugins::ackermann_step
