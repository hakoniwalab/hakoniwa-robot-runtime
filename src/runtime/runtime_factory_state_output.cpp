#include "runtime/runtime_factory_internal.hpp"

#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime::detail {

bool load_state_output_definitions(
    RuntimeParseContext& context,
    RuntimeDefinition& definition,
    std::string* error)
{
    std::unordered_map<std::string, std::string> actuator_by_joint;
    for (const auto& actuator : definition.actuators) {
        actuator_by_joint.emplace(actuator.joint_name, actuator.component_id);
    }

    for (const auto& component : context.components) {
        if (component.value("kind", "") != "state_output"
            || component.value("type", "") != "joint_state") {
            continue;
        }

        RuntimeStateOutputConfig state_output;
        std::string config_value;
        if (!required_string(component, "id", state_output.component_id,
                error, "state output")
            || !required_string(component, "config", config_value,
                error, "state output")
            || !required_string(component, "pdu_robot", state_output.pdu_robot,
                error, "state output")) {
            return false;
        }

        const fs::path config_path = resolve(context.base_path, config_value);
        if (!regular_file(config_path, error)) {
            return false;
        }
        state_output.config_path = config_path.string();

        json state_root;
        if (!read_json(config_path, state_root, error)) {
            return false;
        }
        if (!state_root.contains("spec")
            || !state_root.at("spec").is_object()
            || !state_root.contains("pdu_config")
            || !state_root.at("pdu_config").is_object()) {
            return fail(error,
                "invalid joint state output config: "
                + config_path.string());
        }

        const auto& pdu = state_root.at("pdu_config");
        std::string message_type;
        if (!required_string(pdu, "pdu_name", state_output.pdu_name,
                error, "state output pdu_config")
            || !required_string(pdu, "message_type", message_type,
                error, "state output pdu_config")
            || message_type != "sensor_msgs/JointState") {
            return fail(error,
                "state output message_type must be sensor_msgs/JointState: "
                + state_output.component_id);
        }
        if (!pdu.contains("update_rate_hz")
            || !pdu.at("update_rate_hz").is_number()) {
            return fail(error,
                "state output update_rate_hz is missing or invalid: "
                + state_output.component_id);
        }
        state_output.update_rate_hz =
            pdu.at("update_rate_hz").get<double>();
        if (!std::isfinite(state_output.update_rate_hz)
            || state_output.update_rate_hz <= 0.0) {
            return fail(error,
                "state output update_rate_hz must be finite and positive: "
                + state_output.component_id);
        }

        const auto& spec = state_root.at("spec");
        if (!spec.contains("joints")
            || !spec.at("joints").is_array()
            || spec.at("joints").empty()) {
            return fail(error,
                "state output joints must be a non-empty array: "
                + state_output.component_id);
        }

        std::unordered_map<std::string, std::string> mjcf_joint_by_name;
        if (state_root.contains("mjcf_binding")
            && state_root.at("mjcf_binding").is_object()
            && state_root.at("mjcf_binding").contains("joints")
            && state_root.at("mjcf_binding").at("joints").is_array()) {
            for (const auto& binding :
                 state_root.at("mjcf_binding").at("joints")) {
                std::string name;
                std::string mjcf_joint;
                if (!required_string(binding, "name", name,
                        error, "state output MJCF binding")
                    || !required_string(binding, "mjcf_joint", mjcf_joint,
                        error, "state output MJCF binding")) {
                    return false;
                }
                if (!mjcf_joint_by_name.emplace(name, mjcf_joint).second) {
                    return fail(error,
                        "duplicate state output MJCF binding: " + name);
                }
            }
        }

        std::unordered_set<std::string> output_names;
        for (const auto& joint : spec.at("joints")) {
            std::string name;
            if (!required_string(joint, "name", name,
                    error, "state output joint")) {
                return false;
            }
            if (!output_names.insert(name).second) {
                return fail(error,
                    "duplicate state output joint: " + name);
            }
            const auto physical_joint = mjcf_joint_by_name.contains(name)
                ? mjcf_joint_by_name.at(name)
                : name;
            const auto actuator = actuator_by_joint.find(physical_joint);
            if (actuator == actuator_by_joint.end()) {
                return fail(error,
                    "state output joint has no scalar actuator component: "
                    + name);
            }
            state_output.joints.push_back({name, actuator->second});
        }

        if (!verify_pdu_binding(
                context,
                state_output.pdu_robot,
                state_output.pdu_name,
                "sensor_msgs/JointState",
                1,
                error)) {
            return false;
        }

        definition.state_outputs.push_back(std::move(state_output));
    }

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MOBILE_BASE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MOBILE_BASE
    for (const auto& component : context.components) {
        if (component.value("kind", "") != "state_output"
            || component.value("type", "") != "multi_dof_joint_state") {
            continue;
        }

        RuntimeMultiDofStateOutputConfig output;
        std::string config_value;
        if (!required_string(component, "id", output.component_id,
                error, "MultiDOF state output")
            || !required_string(component, "config", config_value,
                error, "MultiDOF state output")
            || !required_string(component, "pdu_robot", output.pdu_robot,
                error, "MultiDOF state output")) {
            return false;
        }
        const fs::path config_path = resolve(context.base_path, config_value);
        if (!regular_file(config_path, error)) {
            return false;
        }
        output.config_path = config_path.string();

        json root;
        if (!read_json(config_path, root, error)
            || !root.contains("spec") || !root.at("spec").is_object()
            || !root.contains("mjcf_binding")
            || !root.at("mjcf_binding").is_object()
            || !root.contains("pdu_config")
            || !root.at("pdu_config").is_object()) {
            return fail(error,
                "invalid MultiDOF state output config: "
                + config_path.string());
        }

        const auto& pdu = root.at("pdu_config");
        std::string message_type;
        if (!required_string(pdu, "pdu_name", output.pdu_name,
                error, "MultiDOF pdu_config")
            || !required_string(pdu, "message_type", message_type,
                error, "MultiDOF pdu_config")
            || message_type != "sensor_msgs/MultiDOFJointState"
            || !pdu.contains("update_rate_hz")
            || !pdu.at("update_rate_hz").is_number()) {
            return fail(error,
                "MultiDOF output requires sensor_msgs/MultiDOFJointState and update_rate_hz");
        }
        output.update_rate_hz = pdu.at("update_rate_hz").get<double>();
        if (!std::isfinite(output.update_rate_hz)
            || output.update_rate_hz <= 0.0) {
            return fail(error,
                "MultiDOF output rate must be finite and positive");
        }

        const auto& spec = root.at("spec");
        if (!required_string(spec, "frame_id", output.frame_id,
                error, "MultiDOF spec")
            || !spec.contains("bodies")
            || !spec.at("bodies").is_array()
            || spec.at("bodies").empty()) {
            return fail(error,
                "MultiDOF spec requires frame_id and a non-empty bodies array");
        }
        const auto& bindings = root.at("mjcf_binding");
        if (!bindings.contains("bodies")
            || !bindings.at("bodies").is_array()) {
            return fail(error,
                "MultiDOF mjcf_binding requires bodies array");
        }
        std::unordered_map<std::string, std::string> freejoint_by_name;
        for (const auto& binding : bindings.at("bodies")) {
            std::string name;
            std::string freejoint;
            if (!required_string(binding, "name", name,
                    error, "MultiDOF body binding")
                || !required_string(binding, "mjcf_freejoint", freejoint,
                    error, "MultiDOF body binding")
                || !freejoint_by_name.emplace(name, freejoint).second) {
                return fail(error,
                    "MultiDOF body bindings must be valid and unique");
            }
        }
        std::unordered_set<std::string> names;
        for (const auto& body : spec.at("bodies")) {
            std::string name;
            if (!required_string(body, "name", name,
                    error, "MultiDOF body")
                || !names.insert(name).second
                || !freejoint_by_name.contains(name)) {
                return fail(error,
                    "MultiDOF body names must be unique and bound to a freejoint");
            }
            output.bodies.push_back({
                name,
                output.component_id + "/" + name,
                freejoint_by_name.at(name),
            });
        }
        if (names.size() != freejoint_by_name.size()) {
            return fail(error,
                "MultiDOF body bindings contain names absent from spec");
        }
        if (!verify_pdu_binding(
                context,
                output.pdu_robot,
                output.pdu_name,
                "sensor_msgs/MultiDOFJointState",
                1,
                error)) {
            return false;
        }
        definition.multi_dof_state_outputs.push_back(std::move(output));
    }
#endif

    return true;
}

} // namespace hakoniwa::robot_runtime::runtime::detail
