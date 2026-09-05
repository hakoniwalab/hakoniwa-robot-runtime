#pragma once

#include "runtime/types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

struct RuntimeActuatorConfig {
    std::string component_id;
    std::string component_type;
    std::string config_path;
    std::string pdu_robot;
    std::string pdu_name;
    std::string joint_name;
    std::string mujoco_actuator_name;
    ActuatorCommandType command_type {ActuatorCommandType::Position};
    ActuatorLimits limits {};
    double update_rate_hz {0.0};
    std::uint64_t command_timeout_usec {0};
};

struct RuntimeJointStateBinding {
    std::string joint_name;
    std::string actuator_id;
};

struct RuntimeStateOutputConfig {
    std::string component_id;
    std::string config_path;
    std::string pdu_robot;
    std::string pdu_name;
    double update_rate_hz {0.0};
    std::vector<RuntimeJointStateBinding> joints;
};

struct RuntimeTrajectoryBinding {
    std::string joint_name;
    std::string actuator_id;
};

struct RuntimeTrajectoryControllerConfig {
    std::string component_id;
    std::string config_path;
    std::string pdu_robot;
    std::string pdu_name;
    double update_rate_hz {0.0};
    std::vector<RuntimeTrajectoryBinding> joints;
};

struct RuntimeManualAxisBinding {
    std::size_t axis_index {0};
    std::string joint_name;
    std::string actuator_id;
    double velocity_per_sec {0.0};
    bool invert {false};
};

struct RuntimeManualBankConfig {
    std::string id;
    std::optional<std::size_t> select_button_index;
    std::vector<RuntimeManualAxisBinding> bindings;
};

struct RuntimeManualControllerConfig {
    std::string component_id;
    std::string config_path;
    std::string pdu_robot;
    std::string pdu_name;
    std::uint64_t input_timeout_usec {0};
    double update_rate_hz {0.0};
    double deadzone {0.0};
    double expo {0.0};
    std::size_t joy_axis_count {0};
    std::size_t joy_button_count {0};
    std::size_t manual_enable_button_index {0};
    std::vector<RuntimeManualBankConfig> banks;
};

/**
 * Resolved Runtime-local definition produced from the declarative manifest.
 *
 * This is not the source manifest document. Paths, component references,
 * message bindings, command types, limits, and controller relationships have
 * already been validated and normalized for Runtime composition.
 */
struct RuntimeDefinition {
    std::string model_path;
    std::vector<RuntimeActuatorConfig> actuators;
    std::vector<RuntimeStateOutputConfig> state_outputs;
    std::vector<RuntimeTrajectoryControllerConfig> trajectory_controllers;
    std::vector<RuntimeManualControllerConfig> manual_controllers;
};

} // namespace hakoniwa::robot_runtime::runtime
