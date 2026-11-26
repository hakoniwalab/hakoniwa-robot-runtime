#pragma once

#include <array>
#include <string_view>

namespace hakoniwa::robot_runtime::plugins::person_step {

inline constexpr std::array<std::string_view, 6> limb_names {
    "shoulder_left_joint",
    "shoulder_right_joint",
    "hip_left_joint",
    "hip_right_joint",
    "knee_left_joint",
    "knee_right_joint",
};

} // namespace hakoniwa::robot_runtime::plugins::person_step
