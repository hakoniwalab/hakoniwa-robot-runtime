#pragma once

#include "runtime/runtime_factory.hpp"
#include "runtime/service/service_plugin_adapter.hpp"

#include <algorithm>
#include <array>
#include <nlohmann/json.hpp>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

inline ServicePluginConfig ackermann_plugin_config(const RuntimeAckermannControllerConfig& drive,
    const std::vector<RuntimeActuatorConfig>& actuators, bool rpc, const std::vector<std::string>& tire_surfaces)
{
    ServicePluginConfig config;
    config.robot = drive.pdu_robot;
    config.control_id = ackermann_control_group;
    const auto& geometry = drive.geometry;
    const auto& bindings = drive.actuators;
    config.config_json = nlohmann::json {
        {"mode", rpc ? "rpc" : "topic"},
        {"input_timeout_usec", drive.input_timeout_usec},
        {"geometry", {
            {"wheelbase_m", geometry.wheelbase_m},
            {"track_width_m", geometry.track_width_m},
            {"wheel_radius_m", geometry.wheel_radius_m},
            {"max_steering_angle_rad", geometry.max_steering_angle_rad},
            {"max_wheel_angular_velocity_rad_s", geometry.max_wheel_angular_velocity_rad_s},
        }},
        {"actuators", {
            {"steering_left", bindings.steering_left},
            {"steering_right", bindings.steering_right},
            {"drive_left", bindings.drive_left},
            {"drive_right", bindings.drive_right},
        }},
        {"tire_surfaces", tire_surfaces},
    }.dump();
    const std::array ids {bindings.steering_left, bindings.steering_right, bindings.drive_left, bindings.drive_right};
    for (const auto& actuator : actuators) {
        if (std::find(ids.begin(), ids.end(), actuator.component_id) != ids.end()) {
            config.actuators.push_back({actuator.component_id, actuator.command_type, actuator.limits});
        }
    }
    return config;
}

inline ServicePluginConfig person_plugin_config(std::string name, RobotServiceSpec spec, std::uint64_t physics_step_ns)
{
    ServicePluginConfig config;
    config.robot = name;
    config.humanoids.push_back(std::move(name));
    config.services.push_back(std::move(spec));
    config.physics_step_ns = physics_step_ns;
    return config;
}

} // namespace hakoniwa::robot_runtime::runtime
