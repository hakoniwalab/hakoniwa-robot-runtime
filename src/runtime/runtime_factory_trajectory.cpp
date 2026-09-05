#include "runtime/runtime_factory_internal.hpp"

#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime::detail {

bool load_trajectory_definitions(
    RuntimeParseContext& context,
    RuntimeDefinition& definition,
    std::string* error)
{
    std::unordered_map<std::string, const RuntimeActuatorConfig*> actuator_by_id;
    for (const auto& actuator : definition.actuators) {
        actuator_by_id.emplace(actuator.component_id, &actuator);
    }

    for (const auto& component : context.components) {
        if (component.value("kind", "") != "controller"
            || component.value("type", "") != "joint_trajectory_controller") {
            continue;
        }

        RuntimeTrajectoryControllerConfig controller;
        std::string config_value;
        if (!required_string(component, "id", controller.component_id,
                error, "trajectory controller")
            || !required_string(component, "config", config_value,
                error, "trajectory controller")
            || !required_string(component, "pdu_robot", controller.pdu_robot,
                error, "trajectory controller")) {
            return false;
        }

        const fs::path config_path = resolve(context.base_path, config_value);
        if (!regular_file(config_path, error)) {
            return false;
        }
        controller.config_path = config_path.string();

        json controller_root;
        if (!read_json(config_path, controller_root, error)) {
            return false;
        }
        if (!controller_root.contains("input")
            || !controller_root.at("input").is_object()
            || !controller_root.contains("joints")
            || !controller_root.at("joints").is_array()
            || controller_root.at("joints").empty()) {
            return fail(error,
                "invalid trajectory controller config: "
                + config_path.string());
        }

        const auto& input = controller_root.at("input");
        std::string message_type;
        if (!required_string(input, "pdu_name", controller.pdu_name,
                error, "trajectory controller input")
            || !required_string(input, "message_type", message_type,
                error, "trajectory controller input")
            || message_type != "trajectory_msgs/JointTrajectory") {
            return fail(error,
                "trajectory input message_type must be trajectory_msgs/JointTrajectory: "
                + controller.component_id);
        }
        if (!input.contains("update_rate_hz")
            || !input.at("update_rate_hz").is_number()) {
            return fail(error,
                "trajectory input update_rate_hz is missing or invalid: "
                + controller.component_id);
        }
        controller.update_rate_hz = input.at("update_rate_hz").get<double>();
        if (!std::isfinite(controller.update_rate_hz)
            || controller.update_rate_hz <= 0.0) {
            return fail(error,
                "trajectory input update_rate_hz must be finite and positive: "
                + controller.component_id);
        }

        std::unordered_set<std::string> joint_names;
        std::unordered_set<std::string> actuator_ids;
        for (const auto& binding : controller_root.at("joints")) {
            RuntimeTrajectoryBinding parsed;
            if (!required_string(binding, "name", parsed.joint_name,
                    error, "trajectory joint binding")
                || !required_string(binding, "actuator", parsed.actuator_id,
                    error, "trajectory joint binding")) {
                return false;
            }
            if (!joint_names.insert(parsed.joint_name).second
                || !actuator_ids.insert(parsed.actuator_id).second) {
                return fail(error,
                    "trajectory joint and actuator bindings must be unique");
            }
            if (!actuator_by_id.contains(parsed.actuator_id)) {
                return fail(error,
                    "trajectory controller references unknown actuator: "
                    + parsed.actuator_id);
            }
            controller.joints.push_back(std::move(parsed));
        }

        if (!verify_pdu_binding(
                context,
                controller.pdu_robot,
                controller.pdu_name,
                "trajectory_msgs/JointTrajectory",
                1,
                error)) {
            return false;
        }

        definition.trajectory_controllers.push_back(std::move(controller));
    }

    return true;
}

} // namespace hakoniwa::robot_runtime::runtime::detail
