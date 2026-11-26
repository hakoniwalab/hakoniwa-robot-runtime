#pragma once

#include "runtime/types.hpp"
#include "plugins/person_step/limb_names.hpp"

#include <array>
#include <cstdint>
#include <cstddef>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

inline constexpr auto& person_limb_joint_names = plugins::person_step::limb_names;

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

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
