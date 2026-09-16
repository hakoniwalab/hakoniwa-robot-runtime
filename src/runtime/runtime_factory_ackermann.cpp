#include "runtime/runtime_factory_internal.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime::detail {
namespace {

bool positive_number(
    const json& object,
    const char* field,
    double& output,
    std::string* error)
{
    if (!object.contains(field) || !object.at(field).is_number()) {
        return fail(error,
            std::string("Ackermann geometry field is missing or invalid: ") + field);
    }
    output = object.at(field).get<double>();
    if (!std::isfinite(output) || output <= 0.0) {
        return fail(error,
            std::string("Ackermann geometry field must be finite and positive: ") + field);
    }
    return true;
}

} // namespace

bool load_ackermann_definitions(
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
            || component.value("type", "") != "ackermann_controller") {
            continue;
        }

        RuntimeAckermannControllerConfig controller;
        std::string config_value;
        if (!required_string(component, "id", controller.component_id,
                error, "Ackermann Controller")
            || !required_string(component, "config", config_value,
                error, "Ackermann Controller")
            || !required_string(component, "pdu_robot", controller.pdu_robot,
                error, "Ackermann Controller")) {
            return false;
        }
        const fs::path config_path = resolve(context.base_path, config_value);
        if (!regular_file(config_path, error)) {
            return false;
        }
        controller.config_path = config_path.string();

        json root;
        if (!read_json(config_path, root, error)
            || !root.contains("input") || !root.at("input").is_object()
            || !root.contains("geometry") || !root.at("geometry").is_object()
            || !root.contains("actuators") || !root.at("actuators").is_object()) {
            return fail(error,
                "invalid Ackermann Controller config: " + config_path.string());
        }

        const auto& input = root.at("input");
        std::string message_type;
        if (!required_string(input, "pdu_name", controller.pdu_name,
                error, "Ackermann Controller input")
            || !required_string(input, "message_type", message_type,
                error, "Ackermann Controller input")
            || message_type != "ackermann_msgs/AckermannDrive"
            || !input.contains("update_rate_hz")
            || !input.at("update_rate_hz").is_number()
            || !input.contains("timeout_sec")
            || !input.at("timeout_sec").is_number()) {
            return fail(error,
                "Ackermann input requires AckermannDrive, update_rate_hz, and timeout_sec");
        }
        controller.update_rate_hz = input.at("update_rate_hz").get<double>();
        const double timeout_usec = input.at("timeout_sec").get<double>() * 1'000'000.0;
        if (!std::isfinite(controller.update_rate_hz)
            || controller.update_rate_hz <= 0.0
            || !std::isfinite(timeout_usec)
            || timeout_usec < 1.0
            || timeout_usec > static_cast<double>(
                std::numeric_limits<std::int64_t>::max())) {
            return fail(error,
                "Ackermann input rate or timeout is outside its valid range");
        }
        controller.input_timeout_usec =
            static_cast<std::uint64_t>(std::llround(timeout_usec));

        const auto& geometry = root.at("geometry");
        if (!positive_number(geometry, "wheelbase_m",
                controller.geometry.wheelbase_m, error)
            || !positive_number(geometry, "track_width_m",
                controller.geometry.track_width_m, error)
            || !positive_number(geometry, "wheel_radius_m",
                controller.geometry.wheel_radius_m, error)
            || !positive_number(geometry, "max_steering_angle_rad",
                controller.geometry.max_steering_angle_rad, error)
            || !positive_number(geometry, "max_wheel_angular_velocity_rad_s",
                controller.geometry.max_wheel_angular_velocity_rad_s, error)) {
            return false;
        }

        const auto& bindings = root.at("actuators");
        if (!required_string(bindings, "steering_left",
                controller.actuators.steering_left, error, "Ackermann actuators")
            || !required_string(bindings, "steering_right",
                controller.actuators.steering_right, error, "Ackermann actuators")
            || !required_string(bindings, "drive_left",
                controller.actuators.drive_left, error, "Ackermann actuators")
            || !required_string(bindings, "drive_right",
                controller.actuators.drive_right, error, "Ackermann actuators")) {
            return false;
        }
        const std::array<std::pair<std::string, ActuatorCommandType>, 4> required {{
            {controller.actuators.steering_left, ActuatorCommandType::Position},
            {controller.actuators.steering_right, ActuatorCommandType::Position},
            {controller.actuators.drive_left, ActuatorCommandType::Velocity},
            {controller.actuators.drive_right, ActuatorCommandType::Velocity},
        }};
        std::unordered_set<std::string> unique;
        for (const auto& [id, type] : required) {
            const auto actuator = actuator_by_id.find(id);
            if (!unique.insert(id).second
                || actuator == actuator_by_id.end()
                || actuator->second->command_type != type) {
                return fail(error,
                    "Ackermann actuator binding is missing, duplicated, or has the wrong command type: "
                    + id);
            }
        }

        if (!verify_pdu_binding(
                context,
                controller.pdu_robot,
                controller.pdu_name,
                "ackermann_msgs/AckermannDrive",
                48,
                error)) {
            return false;
        }
        definition.ackermann_controllers.push_back(std::move(controller));
    }
    return true;
}

} // namespace hakoniwa::robot_runtime::runtime::detail
