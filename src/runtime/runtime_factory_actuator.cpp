#include "runtime/runtime_factory_internal.hpp"

#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>

namespace hakoniwa::robot_runtime::runtime::detail {
namespace {

bool parse_command_type(
    const std::string& component_type,
    const std::string& spec_type,
    ActuatorCommandType& output,
    std::string* error)
{
    struct Mapping {
        const char* component;
        const char* spec;
        ActuatorCommandType command;
    };
    constexpr Mapping mappings[] {
        {"joint_position_actuator", "position", ActuatorCommandType::Position},
        {"joint_velocity_actuator", "velocity", ActuatorCommandType::Velocity},
        {"joint_torque_actuator", "torque", ActuatorCommandType::Effort},
    };
    for (const auto& mapping : mappings) {
        if (component_type != mapping.component) {
            continue;
        }
        if (spec_type != mapping.spec) {
            return fail(error, "component type " + component_type
                + " does not match actuator spec.type " + spec_type);
        }
        output = mapping.command;
        return true;
    }
    return fail(error,
        "unsupported scalar actuator component type: " + component_type);
}

bool parse_limits(
    const json& spec,
    const ActuatorCommandType type,
    ActuatorLimits& output,
    std::string* error)
{
    if (!spec.contains("limit") || !spec.at("limit").is_object()) {
        return fail(error, "scalar actuator spec.limit is required");
    }
    const auto& limit = spec.at("limit");
    if (type == ActuatorCommandType::Position) {
        if (!limit.contains("lower") || !limit.at("lower").is_number()
            || !limit.contains("upper") || !limit.at("upper").is_number()) {
            return fail(error,
                "position actuator requires numeric lower and upper limits");
        }
        output = {
            limit.at("lower").get<double>(),
            limit.at("upper").get<double>(),
        };
    } else {
        const char* field = type == ActuatorCommandType::Velocity
            ? "velocity" : "effort";
        if (!limit.contains(field) || !limit.at(field).is_number()) {
            return fail(error,
                std::string("actuator requires numeric limit: ") + field);
        }
        const double magnitude = limit.at(field).get<double>();
        if (!std::isfinite(magnitude) || magnitude <= 0.0) {
            return fail(error,
                std::string("actuator limit must be finite and positive: ")
                + field);
        }
        output = {-magnitude, magnitude};
    }
    if (!std::isfinite(output.minimum) || !std::isfinite(output.maximum)
        || !output.is_valid()) {
        return fail(error, "actuator limits must be finite and ordered");
    }
    return true;
}

bool timeout_usec(
    const json& runtime_root,
    const std::string& component_id,
    std::uint64_t& output,
    std::string* error)
{
    if (!runtime_root.contains("actuators")
        || !runtime_root.at("actuators").is_object()
        || !runtime_root.at("actuators").contains(component_id)) {
        return fail(error,
            "runtime timeout is missing for actuator: " + component_id);
    }
    const auto& item = runtime_root.at("actuators").at(component_id);
    if (!item.is_object() || !item.contains("command_timeout_sec")
        || !item.at("command_timeout_sec").is_number()) {
        return fail(error,
            "command_timeout_sec is missing or invalid for actuator: "
            + component_id);
    }
    const double seconds = item.at("command_timeout_sec").get<double>();
    constexpr double usec_per_sec = 1'000'000.0;
    const double converted = seconds * usec_per_sec;
    if (!std::isfinite(converted) || converted < 1.0
        || converted > static_cast<double>(
            std::numeric_limits<std::int64_t>::max())) {
        return fail(error,
            "command_timeout_sec is outside the representable range for actuator: "
            + component_id);
    }
    output = static_cast<std::uint64_t>(std::llround(converted));
    return true;
}

} // namespace

bool load_actuator_definitions(
    RuntimeParseContext& context,
    RuntimeDefinition& definition,
    std::string* error)
{
    for (const auto& component : context.components) {
        const std::string kind = component.value("kind", "");
        const std::string type = component.value("type", "");
        if (kind != "actuator"
            || type == "joint_trajectory_actuator"
            || type == "differential_drive_twist") {
            continue;
        }

        RuntimeActuatorConfig actuator;
        actuator.component_type = type;
        std::string config_value;
        if (!required_string(component, "id", actuator.component_id,
                error, "scalar actuator component")
            || !required_string(component, "config", config_value,
                error, "scalar actuator component")
            || !required_string(component, "pdu_robot", actuator.pdu_robot,
                error, "scalar actuator component")) {
            return false;
        }

        const fs::path config_path = resolve(context.base_path, config_value);
        if (!regular_file(config_path, error)) {
            return false;
        }
        actuator.config_path = config_path.string();

        json config_root;
        if (!read_json(config_path, config_root, error)) {
            return false;
        }
        if (!config_root.contains("spec")
            || !config_root.at("spec").is_object()
            || !config_root.contains("mjcf_binding")
            || !config_root.at("mjcf_binding").is_object()
            || !config_root.contains("pdu_config")
            || !config_root.at("pdu_config").is_object()) {
            return fail(error,
                "invalid joint actuator config: " + config_path.string());
        }

        const auto& spec = config_root.at("spec");
        std::string spec_type;
        if (!required_string(spec, "joint_name", actuator.joint_name,
                error, "actuator spec")
            || !required_string(spec, "type", spec_type,
                error, "actuator spec")
            || !parse_command_type(
                type, spec_type, actuator.command_type, error)
            || !parse_limits(
                spec, actuator.command_type, actuator.limits, error)) {
            return false;
        }

        actuator.mujoco_actuator_name =
            config_root.at("mjcf_binding").value(
                "actuator_name", actuator.joint_name);

        const auto& pdu = config_root.at("pdu_config");
        std::string message_type;
        if (!required_string(pdu, "pdu_name", actuator.pdu_name,
                error, "actuator pdu_config")
            || !required_string(pdu, "message_type", message_type,
                error, "actuator pdu_config")
            || message_type != "std_msgs/Float64") {
            return fail(error,
                "actuator message_type must be std_msgs/Float64: "
                + actuator.component_id);
        }
        if (!pdu.contains("update_rate_hz")
            || !pdu.at("update_rate_hz").is_number()) {
            return fail(error,
                "actuator update_rate_hz is missing or invalid: "
                + actuator.component_id);
        }
        actuator.update_rate_hz = pdu.at("update_rate_hz").get<double>();
        if (!std::isfinite(actuator.update_rate_hz)
            || actuator.update_rate_hz <= 0.0) {
            return fail(error,
                "actuator update_rate_hz must be finite and positive: "
                + actuator.component_id);
        }

        if (!timeout_usec(
                context.runtime_root,
                actuator.component_id,
                actuator.command_timeout_usec,
                error)
            || !verify_pdu_binding(
                context,
                actuator.pdu_robot,
                actuator.pdu_name,
                "std_msgs/Float64",
                32,
                error)) {
            return false;
        }

        definition.actuators.push_back(std::move(actuator));
    }

    if (definition.actuators.empty()) {
        return fail(error,
            "manifest contains no scalar actuator components");
    }

    std::unordered_map<std::string, std::string> actuator_by_joint;
    for (const auto& actuator : definition.actuators) {
        if (!actuator_by_joint.emplace(
                actuator.joint_name,
                actuator.component_id).second) {
            return fail(error,
                "multiple scalar actuators target joint: "
                + actuator.joint_name);
        }
    }
    return true;
}

} // namespace hakoniwa::robot_runtime::runtime::detail
