#pragma once

#include "hakoniwa/robot_runtime/runner/async_rpc_input.hpp"
#include "hakoniwa/robot_runtime/runner/hakoniwa/async_rpc_service_channel.hpp"
#include "runtime/types.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

inline constexpr std::array<std::string_view, 6> person_limb_joint_names {
    "shoulder_left_joint",
    "shoulder_right_joint",
    "hip_left_joint",
    "hip_right_joint",
    "knee_left_joint",
    "knee_right_joint",
};

struct PersonStepCommand {
    runtime::Vector3State linear_velocity;
    runtime::Vector3State angular_velocity;
    std::string animation;
    std::string ride;
};

struct PersonStepState {
    std::uint32_t result_code {0};
    std::uint64_t simulation_time_usec {0};
    runtime::Vector3State position;
    runtime::QuaternionState orientation;
    runtime::Vector3State linear_velocity;
    runtime::Vector3State angular_velocity;
    std::array<double, person_limb_joint_names.size()> joint_positions {};
};

[[nodiscard]] std::vector<std::byte> encode_person_command(
    const PersonStepCommand& command);
[[nodiscard]] bool decode_person_command(
    std::span<const std::byte> payload,
    PersonStepCommand& command) noexcept;
[[nodiscard]] std::vector<std::byte> encode_person_state(
    const PersonStepState& state);
[[nodiscard]] bool decode_person_state(
    std::span<const std::byte> payload,
    PersonStepState& state) noexcept;

/** `PersonService/PersonStep/<person>`. */
[[nodiscard]] std::string person_step_service_name(
    std::string_view person_name);

/** One PersonStep Core service server per person. */
[[nodiscard]] std::shared_ptr<runner::hakoniwa::CoreAsyncRpcServiceChannel>
make_person_step_service_channel(
    std::string person_name,
    std::string asset_name);

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
