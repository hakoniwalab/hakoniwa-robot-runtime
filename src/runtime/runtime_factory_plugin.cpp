#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN) && HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN

#include "runtime/runtime_factory.hpp"
#include "runtime/runtime_factory_internal.hpp"

#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime::detail {

/*
 * Component:  {"id": "...", "kind": "controller", "type": "plugin",
 *              "config": "config/controller/my_plugin.json"}
 * Its config: {"plugin": {"path": "libmy_plugin.so", "config": "my_plugin.yaml"},
 *              "joints": ["actuator_id", ...],
 *              "priority": 10,
 *              "joint_state_source": "truth",
 *              "pdus": ["lidar_scan", "wrist_wrench"],
 *              "stop_on_error": true}
 * "pdus" names sensor output PDUs (sensor builds only); their types come
 * from the sensor outputs (resolve_plugin_controller_pdus).
 * Relative plugin paths resolve against the directory of the config file.
 * The library itself is opened by build_runtime(), not here.
 */
bool load_plugin_controller_definitions(
    RuntimeParseContext& context,
    RuntimeDefinition& definition,
    std::string* error)
{
    std::unordered_set<std::string> actuator_ids;
    for (const auto& actuator : definition.actuators) {
        actuator_ids.insert(actuator.component_id);
    }

    for (const auto& component : context.components) {
        if (component.value("kind", "") != "controller"
            || component.value("type", "") != "plugin") {
            continue;
        }

        RuntimePluginControllerConfig controller;
        std::string config_value;
        if (!required_string(component, "id", controller.component_id,
                error, "plugin controller")
            || !required_string(component, "config", config_value,
                error, "plugin controller")) {
            return false;
        }

        const fs::path config_path = resolve(context.base_path, config_value);
        if (!regular_file(config_path, error)) {
            return false;
        }
        controller.config_path = config_path.string();
        const fs::path config_dir = config_path.parent_path();

        json root;
        if (!read_json(config_path, root, error)) {
            return false;
        }
        if (!root.is_object()
            || !root.contains("plugin")
            || !root.at("plugin").is_object()) {
            return fail(error,
                "plugin controller config needs a \"plugin\" object: "
                + config_path.string());
        }

        const auto& plugin = root.at("plugin");
        std::string plugin_path;
        if (!required_string(plugin, "path", plugin_path,
                error, "plugin controller plugin")) {
            return false;
        }
        const fs::path resolved_plugin = resolve(config_dir, plugin_path);
        if (!regular_file(resolved_plugin, error)) {
            return false;
        }
        controller.plugin_path = resolved_plugin.string();

        if (plugin.contains("config")) {
            std::string plugin_config;
            if (!required_string(plugin, "config", plugin_config,
                    error, "plugin controller plugin")) {
                return false;
            }
            // The plugin reads its own config; the Runtime only checks it exists.
            const fs::path resolved_config = resolve(config_dir, plugin_config);
            if (!regular_file(resolved_config, error)) {
                return false;
            }
            controller.plugin_config_path = resolved_config.string();
        }

        if (!root.contains("joints")
            || !root.at("joints").is_array()
            || root.at("joints").empty()) {
            return fail(error,
                "plugin controller config needs a non-empty \"joints\" array: "
                + config_path.string());
        }
        std::unordered_set<std::string> joint_ids;
        for (const auto& joint : root.at("joints")) {
            if (!joint.is_string() || joint.get_ref<const std::string&>().empty()) {
                return fail(error,
                    "plugin controller joints must be actuator IDs: "
                    + config_path.string());
            }
            const auto actuator_id = joint.get<std::string>();
            if (!actuator_ids.contains(actuator_id)) {
                return fail(error,
                    "plugin controller references unknown actuator: " + actuator_id);
            }
            if (!joint_ids.insert(actuator_id).second) {
                return fail(error,
                    "plugin controller joints must be unique: " + actuator_id);
            }
            controller.actuator_ids.push_back(actuator_id);
        }

        if (root.contains("priority")) {
            const auto& priority = root.at("priority");
            if (!priority.is_number_integer()
                || priority.get<std::int64_t>() < std::numeric_limits<std::int32_t>::min()
                || priority.get<std::int64_t>() > std::numeric_limits<std::int32_t>::max()) {
                return fail(error,
                    "plugin controller priority must be a 32-bit integer: "
                    + controller.component_id);
            }
            controller.priority = priority.get<std::int32_t>();
        }

        if (root.contains("joint_state_source")) {
            if (!required_string(root, "joint_state_source",
                    controller.joint_state_source, error, "plugin controller")) {
                return false;
            }
        }
        if (controller.joint_state_source == "observed") {
            // TODO: read the joint sensors once the sensor stage provides
            // them to controllers (design 5.1).
            return fail(error,
                "plugin controller joint_state_source \"observed\" is not supported yet: "
                + controller.component_id);
        }
        if (controller.joint_state_source != "truth") {
            return fail(error,
                "plugin controller joint_state_source must be \"truth\" or \"observed\": "
                + controller.component_id);
        }

        if (root.contains("stop_on_error")) {
            if (!root.at("stop_on_error").is_boolean()) {
                return fail(error,
                    "plugin controller stop_on_error must be a boolean: "
                    + controller.component_id);
            }
            controller.stop_on_error = root.at("stop_on_error").get<bool>();
        }

        if (root.contains("pdus")) {
            const auto& pdus = root.at("pdus");
            if (!pdus.is_array()) {
                return fail(error,
                    "plugin controller pdus must be an array of PDU names: "
                    + controller.component_id);
            }
#if !defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) || !HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR
            if (!pdus.empty()) {
                return fail(error,
                    "plugin controller " + controller.component_id
                    + " lists sensor PDUs, but this application is built without"
                      " sensor support");
            }
#endif
            std::unordered_set<std::string> pdu_names;
            for (const auto& pdu : pdus) {
                if (!pdu.is_string() || pdu.get_ref<const std::string&>().empty()) {
                    return fail(error,
                        "plugin controller pdus must be PDU names: "
                        + controller.component_id);
                }
                const auto pdu_name = pdu.get<std::string>();
                if (!pdu_names.insert(pdu_name).second) {
                    return fail(error,
                        "plugin controller pdus must be unique: " + pdu_name);
                }
                controller.pdu_names.push_back(pdu_name);
            }
        }

        definition.plugin_controllers.push_back(std::move(controller));
    }
    return true;
}

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR
bool resolve_plugin_controller_pdus(
    RuntimeDefinition& definition,
    std::string* error)
{
    for (auto& controller : definition.plugin_controllers) {
        controller.pdu_types.clear();
        for (const auto& pdu_name : controller.pdu_names) {
            const RuntimeSensorOutputConfig* match = nullptr;
            for (const auto& output : definition.sensor_outputs) {
                if (output.pdu_name != pdu_name) {
                    continue;
                }
                if (match != nullptr) {
                    return fail(error,
                        "plugin controller " + controller.component_id
                        + ": more than one sensor output writes PDU " + pdu_name);
                }
                match = &output;
            }
            if (match == nullptr) {
                return fail(error,
                    "plugin controller " + controller.component_id
                    + ": no sensor output writes PDU " + pdu_name);
            }
            // Only the writers that hand their encoded bytes to a plugin.
            if (match->message_type != "sensor_msgs/PointCloud2"
                && match->message_type != "geometry_msgs/WrenchStamped") {
                return fail(error,
                    "plugin controller " + controller.component_id
                    + ": PDU " + pdu_name + " is " + match->message_type
                    + "; plugins receive sensor_msgs/PointCloud2 and"
                      " geometry_msgs/WrenchStamped outputs only");
            }
            controller.pdu_types.push_back(match->message_type);
        }
    }
    return true;
}
#endif

} // namespace hakoniwa::robot_runtime::runtime::detail

namespace hakoniwa::robot_runtime::runtime {

PluginPduChannels make_plugin_pdu_channels(const RuntimeDefinition& definition)
{
    PluginPduChannels channels;
    for (const auto& controller : definition.plugin_controllers) {
        if (controller.pdu_types.size() != controller.pdu_names.size()) {
            throw std::invalid_argument(
                "plugin controller " + controller.component_id
                + ": PDU types are not resolved");
        }
        for (std::size_t index = 0; index < controller.pdu_names.size(); ++index) {
            (void)channels.add(controller.pdu_names[index], controller.pdu_types[index]);
        }
    }
    return channels;
}

} // namespace hakoniwa::robot_runtime::runtime

#endif
