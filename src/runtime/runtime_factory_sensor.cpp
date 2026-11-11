// Sensor components and outputs. Compiled only into applications built with
// HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR; other builds reject sensor components
// in runtime_factory_manifest.cpp and do not need this file.
#include "runtime/runtime_factory_internal.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR
#include "runtime/sensor/sensor_catalog.hpp"
#include "runtime/sensor/sensor_registry.hpp"

#include <cmath>
#include <regex>
#include <unordered_map>
#include <unordered_set>
#endif

namespace hakoniwa::robot_runtime::runtime::detail {

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#ifndef HAKONIWA_ROBOT_RUNTIME_SENSOR_CATALOG_PATH
#error "HAKONIWA_ROBOT_RUNTIME_SENSOR_CATALOG_PATH must name catalog/sensor-implementations.json"
#endif

namespace {

constexpr const char* kSensorKind = "sensor";
constexpr const char* kSensorInstancesType = "sensor_instances";
constexpr const char* kSensorOutputType = "sensor_output";

bool is_sensor_component(const json& component)
{
    return component.value("kind", "") == kSensorKind
        || (component.value("kind", "") == "state_output"
            && component.value("type", "") == kSensorOutputType);
}

bool only_keys(
    const json& object,
    const std::unordered_set<std::string>& allowed,
    const std::string& owner,
    std::string* error)
{
    for (const auto& [key, value] : object.items()) {
        (void)value;
        if (!allowed.contains(key)) {
            return fail(error, owner + " has an unknown key: " + key);
        }
    }
    return true;
}

bool schema_version_one(const json& root, const std::string& owner, std::string* error)
{
    if (!root.contains("schema_version") || root.at("schema_version") != 1) {
        return fail(error, owner + " requires \"schema_version\": 1");
    }
    return true;
}

bool load_number_array(
    const json& object,
    const char* field,
    std::array<double, 3>& output,
    const std::string& owner,
    std::string* error)
{
    if (!object.contains(field)) {
        return true;
    }
    const auto& value = object.at(field);
    if (!value.is_array() || value.size() != 3) {
        return fail(error, owner + "." + field + " must be an array of three numbers");
    }
    for (std::size_t index = 0; index < 3; ++index) {
        if (!value.at(index).is_number()
            || !std::isfinite(value.at(index).get<double>())) {
            return fail(error, owner + "." + field + " must hold finite numbers");
        }
        output[index] = value.at(index).get<double>();
    }
    return true;
}

bool load_orientation(
    const json& mount,
    std::array<double, 3>& output,
    const std::string& owner,
    std::string* error)
{
    if (!mount.contains("orientation_deg")) {
        return true;
    }
    const auto& value = mount.at("orientation_deg");
    if (!value.is_object()
        || !only_keys(value, {"roll", "pitch", "yaw"}, owner + ".orientation_deg", error)) {
        return fail(error, owner + ".orientation_deg must be an object of roll, pitch and yaw");
    }
    const char* axes[] = {"roll", "pitch", "yaw"};
    for (std::size_t index = 0; index < 3; ++index) {
        if (!value.contains(axes[index])) {
            continue;
        }
        const auto& angle = value.at(axes[index]);
        if (!angle.is_number() || !std::isfinite(angle.get<double>())) {
            return fail(error, owner + ".orientation_deg." + axes[index] + " must be a finite number");
        }
        output[index] = angle.get<double>();
    }
    return true;
}

bool load_mount(
    const json& sensor,
    RuntimeSensorConfig& output,
    const std::string& owner,
    std::string* error)
{
    if (!sensor.at("mount").is_object()) {
        return fail(error, owner + ".mount must be an object");
    }
    const auto& mount = sensor.at("mount");
    if (output.type == "joint") {
        if (!only_keys(mount, {"mjcf_joint"}, owner + ".mount (joint sensor)", error)) {
            return false;
        }
        return required_string(mount, "mjcf_joint", output.mjcf_joint, error, owner + ".mount");
    }
    if (output.type == "lidar") {
        // A range sensor may name the root body whose subtree its rays never
        // hit; by default only the mount body (the sensor housing) is skipped.
        if (!only_keys(mount, {"mjcf_body", "position_m", "orientation_deg", "exclude_mjcf_body"},
                owner + ".mount (range sensor)", error)) {
            return false;
        }
        if (mount.contains("exclude_mjcf_body")
            && !required_string(mount, "exclude_mjcf_body", output.exclude_mjcf_body, error,
                owner + ".mount")) {
            return false;
        }
    } else if (!only_keys(mount, {"mjcf_body", "position_m", "orientation_deg"},
            owner + ".mount (body mount)", error)) {
        return false;
    }
    if (!required_string(mount, "mjcf_body", output.mjcf_body, error, owner + ".mount")
        || !load_number_array(mount, "position_m", output.position_m, owner + ".mount", error)
        || !load_orientation(mount, output.orientation_deg, owner + ".mount", error)) {
        return false;
    }
    return true;
}

bool load_sensor(
    const json& sensor,
    const fs::path& config_directory,
    const sensor::SensorCatalog& catalog,
    const std::set<std::string>& linked,
    RuntimeSensorConfig& output,
    std::string* error)
{
    static const std::regex name_pattern("^[A-Za-z][A-Za-z0-9_]*$");
    static const std::unordered_set<std::string> known_types {
        "joint", "force_torque", "imu", "mag", "baro", "gps",
        "rangefinder", "lidar", "depth_camera", "rgb_camera"};

    const std::string owner = "sensor of component " + output.component_id;
    if (!sensor.is_object()) {
        return fail(error, owner + " must be an object");
    }
    std::string definition_value;
    if (!only_keys(sensor, {"name", "type", "implementation", "mount", "definition"}, owner, error)
        || !required_string(sensor, "name", output.name, error, owner)
        || !required_string(sensor, "type", output.type, error, owner)
        || !required_string(sensor, "implementation", output.implementation, error, owner)
        || !required_string(sensor, "definition", definition_value, error, owner)) {
        return false;
    }
    const std::string named = "sensor " + output.name;
    if (!std::regex_match(output.name, name_pattern)) {
        return fail(error, named + ": name must match ^[A-Za-z][A-Za-z0-9_]*$");
    }
    if (!known_types.contains(output.type)) {
        return fail(error, named + ": unknown sensor type \"" + output.type + "\"");
    }
    if (output.type != "joint" && output.type != "force_torque" && output.type != "imu"
        && output.type != "lidar") {
        return fail(error, named + ": sensor type \"" + output.type
            + "\" is not supported by this Runtime yet (supported: joint, force_torque, imu, lidar)");
    }
    if (!sensor.contains("mount") || !load_mount(sensor, output, named, error)) {
        if (!sensor.contains("mount")) {
            return fail(error, named + " requires a mount");
        }
        return false;
    }

    const fs::path definition_path = resolve(config_directory, definition_value);
    if (!regular_file(definition_path, error)) {
        return false;
    }
    output.definition_path = definition_path.string();
    json definition;
    if (!read_json(definition_path, definition, error)) {
        return false;
    }
    output.measures_torque = definition.is_object() && definition.contains("torque");
    if (!sensor::validate_sensor_definition(
            catalog, linked, output.implementation, output.type, definition,
            named + " (" + definition_path.string() + ")", error)) {
        return false;
    }
    // The catalog says the implementation provides this type; its linked
    // library must agree.
    const auto* implementation =
        sensor::sensor_implementation_registry().find(output.implementation);
    if (implementation == nullptr
        || !sensor::provides_sensor_type(*implementation, output.type)) {
        return fail(error, named + ": the linked " + output.implementation
            + " library does not provide sensor type \"" + output.type
            + "\" although the catalog lists it");
    }
    return true;
}

bool load_sensor_seed(const json& runtime_root, RuntimeDefinition& definition, std::string* error)
{
    if (!runtime_root.is_object() || !runtime_root.contains("sensors")) {
        return true;
    }
    const auto& sensors = runtime_root.at("sensors");
    if (!sensors.is_object() || !only_keys(sensors, {"seed"}, "runtime config sensors", error)) {
        return fail(error, "runtime config \"sensors\" must be an object with an optional seed");
    }
    if (sensors.contains("seed")) {
        if (!sensors.at("seed").is_number_unsigned()) {
            return fail(error, "runtime config sensors.seed must be a non-negative integer");
        }
        definition.sensor_seed = sensors.at("seed").get<std::uint64_t>();
    }
    return true;
}

bool load_sensor_output(
    RuntimeParseContext& context,
    const json& component,
    const std::unordered_map<std::string, const RuntimeSensorConfig*>& sensors,
    RuntimeSensorOutputConfig& output,
    std::string* error)
{
    std::string config_value;
    if (!required_string(component, "id", output.component_id, error, "sensor output")
        || !required_string(component, "config", config_value, error, "sensor output")
        || !required_string(component, "pdu_robot", output.pdu_robot, error, "sensor output")) {
        return false;
    }
    const std::string owner = "sensor output " + output.component_id;
    const fs::path config_path = resolve(context.base_path, config_value);
    json root;
    if (!regular_file(config_path, error) || !read_json(config_path, root, error)) {
        return false;
    }
    if (!root.is_object()
        || !only_keys(root, {"$schema", "schema_version", "sensors", "pdu_config"}, owner, error)
        || !schema_version_one(root, owner, error)) {
        return root.is_object() ? false : fail(error, owner + " must be an object");
    }
    if (!root.contains("sensors") || !root.at("sensors").is_array()
        || root.at("sensors").empty()) {
        return fail(error, owner + ": sensors must be a non-empty array");
    }
    if (!root.contains("pdu_config") || !root.at("pdu_config").is_object()) {
        return fail(error, owner + " requires pdu_config");
    }
    std::string message_type;
    if (!required_string(root.at("pdu_config"), "message_type", message_type,
            error, owner + ".pdu_config")) {
        return false;
    }
    const bool joint_state = message_type == "sensor_msgs/JointState";
    const bool wrench = message_type == "geometry_msgs/WrenchStamped";
    const bool imu = message_type == "sensor_msgs/Imu";
    const bool scan = message_type == "sensor_msgs/LaserScan";
    if (!joint_state && !wrench && !imu && !scan) {
        return fail(error, owner + ": message_type must be sensor_msgs/JointState,"
            " geometry_msgs/WrenchStamped, sensor_msgs/Imu or sensor_msgs/LaserScan");
    }
    output.message_type = message_type;
    if (!joint_state && root.at("sensors").size() != 1) {
        return fail(error, owner + ": a " + message_type + " output carries exactly one sensor");
    }
    std::unordered_set<std::string> seen;
    for (const auto& item : root.at("sensors")) {
        if (!item.is_string() || item.get<std::string>().empty()) {
            return fail(error, owner + ": sensors must hold sensor names");
        }
        const auto name = item.get<std::string>();
        if (!seen.insert(name).second) {
            return fail(error, owner + ": sensor listed twice: " + name);
        }
        const auto found = sensors.find(name);
        if (found == sensors.end()) {
            return fail(error, owner + ": no sensor named " + name);
        }
        const std::string expected_type = joint_state ? "joint"
            : (wrench ? "force_torque" : (imu ? "imu" : "lidar"));
        if (found->second->type != expected_type) {
            return fail(error, owner + ": sensor " + name + " is of type " + found->second->type
                + " and cannot go into " + message_type);
        }
        if (joint_state && !output.sensors.empty()
            && sensors.at(output.sensors.front())->measures_torque != found->second->measures_torque) {
            return fail(error, owner + ": sensors " + output.sensors.front() + " and " + name
                + " differ in measuring the joint torque; one JointState carries effort for all"
                  " joints or for none");
        }
        output.sensors.push_back(name);
    }
    const auto& pdu = root.at("pdu_config");
    if (!only_keys(pdu, {"pdu_name", "message_type", "update_rate_hz"}, owner + ".pdu_config", error)
        || !required_string(pdu, "pdu_name", output.pdu_name, error, owner + ".pdu_config")) {
        return false;
    }
    if (!pdu.contains("update_rate_hz") || !pdu.at("update_rate_hz").is_number()
        || !std::isfinite(pdu.at("update_rate_hz").get<double>())
        || pdu.at("update_rate_hz").get<double>() <= 0.0) {
        return fail(error, owner + ": update_rate_hz must be finite and positive");
    }
    output.update_rate_hz = pdu.at("update_rate_hz").get<double>();
    return verify_pdu_binding(
        context, output.pdu_robot, output.pdu_name, message_type, 1, error);
}

} // namespace

bool load_sensor_definitions(
    RuntimeParseContext& context,
    RuntimeDefinition& definition,
    std::string* error)
{
    if (!load_sensor_seed(context.runtime_root, definition, error)) {
        return false;
    }
    bool any_sensor = false;
    for (const auto& component : context.components) {
        any_sensor = any_sensor || is_sensor_component(component);
    }
    if (!any_sensor) {
        return true;
    }

    const auto linked = sensor::sensor_implementation_registry().names();
    if (linked.empty()) {
        return fail(error,
            "the manifest has sensors, but no sensor implementation is linked into this application");
    }
    sensor::SensorCatalog catalog;
    if (!sensor::load_sensor_catalog(HAKONIWA_ROBOT_RUNTIME_SENSOR_CATALOG_PATH, catalog, error)) {
        return false;
    }

    std::unordered_set<std::string> names;
    for (const auto& component : context.components) {
        if (component.value("kind", "") != kSensorKind) {
            continue;
        }
        std::string component_id;
        std::string type;
        std::string config_value;
        std::string pdu_robot;
        if (!required_string(component, "id", component_id, error, "sensor component")
            || !required_string(component, "type", type, error, "sensor component")
            || !required_string(component, "config", config_value, error, "sensor component")
            || !required_string(component, "pdu_robot", pdu_robot, error, "sensor component")) {
            return false;
        }
        if (type != kSensorInstancesType) {
            return fail(error, "sensor component " + component_id
                + " must have type \"" + kSensorInstancesType + "\"");
        }
        const fs::path config_path = resolve(context.base_path, config_value);
        json root;
        if (!regular_file(config_path, error) || !read_json(config_path, root, error)) {
            return false;
        }
        const std::string owner = "sensor component " + component_id;
        if (!root.is_object()) {
            return fail(error, owner + " config must be an object");
        }
        if (!only_keys(root, {"$schema", "schema_version", "sensors"}, owner, error)
            || !schema_version_one(root, owner, error)) {
            return false;
        }
        if (!root.contains("sensors") || !root.at("sensors").is_array()
            || root.at("sensors").empty()) {
            return fail(error, owner + ": sensors must be a non-empty array");
        }
        for (const auto& item : root.at("sensors")) {
            RuntimeSensorConfig sensor_config;
            sensor_config.component_id = component_id;
            sensor_config.pdu_robot = pdu_robot;
            if (!load_sensor(item, config_path.parent_path(), catalog, linked,
                    sensor_config, error)) {
                return false;
            }
            if (!names.insert(sensor_config.name).second) {
                return fail(error, "duplicate sensor name: " + sensor_config.name);
            }
            definition.sensors.push_back(std::move(sensor_config));
        }
    }

    std::unordered_map<std::string, const RuntimeSensorConfig*> by_name;
    for (const auto& sensor_config : definition.sensors) {
        by_name.emplace(sensor_config.name, &sensor_config);
    }
    for (const auto& component : context.components) {
        if (component.value("kind", "") != "state_output"
            || component.value("type", "") != kSensorOutputType) {
            continue;
        }
        RuntimeSensorOutputConfig output;
        if (!load_sensor_output(context, component, by_name, output, error)) {
            return false;
        }
        definition.sensor_outputs.push_back(std::move(output));
    }
    return true;
}

#endif

} // namespace hakoniwa::robot_runtime::runtime::detail
