// Sensor components and outputs. Compiled only into applications built with
// HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR; other builds reject sensor components
// in runtime_factory_manifest.cpp and do not need this file.
#include "runtime/runtime_factory_internal.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR
#include "runtime/sensor/sensor_catalog.hpp"
#include "runtime/sensor/sensor_registry.hpp"

#include <algorithm>
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
    if (output.type == "lidar" || output.type == "rangefinder" || output.type == "depth_camera") {
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
        && output.type != "lidar" && output.type != "rangefinder"
        && output.type != "mag" && output.type != "baro" && output.type != "gps"
        && output.type != "depth_camera" && output.type != "rgb_camera") {
        return fail(error, named + ": sensor type \"" + output.type
            + "\" is not supported by this Runtime yet (supported: joint, force_torque, imu, lidar, rangefinder, mag, baro, gps, depth_camera, rgb_camera)");
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
    if (output.type == "lidar" && definition.is_object()) {
        const auto scan = definition.find("scan");
        if (scan != definition.end() && scan->is_object()) {
            const auto vertical = scan->find("vertical");
            output.scans_3d = vertical != scan->end() && vertical->is_object()
                && vertical->value("samples", 1) > 1;
        }
    }
    if (output.type == "rgb_camera") {
        // The rig's camera names: "" for a single camera without "cameras".
        output.camera_names.clear();
        const auto cameras = definition.is_object() ? definition.find("cameras") : definition.end();
        if (definition.is_object() && cameras != definition.end() && cameras->is_array()) {
            for (const auto& camera : *cameras) {
                output.camera_names.push_back(camera.is_object() ? camera.value("name", "") : "");
            }
        } else {
            output.camera_names.push_back("");
        }
    }
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

bool number_field(const json& object, const char* key, double& value, const std::string& owner,
    std::string* error, bool required)
{
    if (!object.contains(key)) {
        return required ? fail(error, owner + "." + key + " is required") : true;
    }
    if (!object.at(key).is_number() || !std::isfinite(object.at(key).get<double>())) {
        return fail(error, owner + "." + key + " must be a finite number");
    }
    value = object.at(key).get<double>();
    return true;
}

bool load_sensor_world(const json& world_value, RuntimeSensorWorld& world, std::string* error)
{
    const std::string owner = "runtime config sensors.world";
    if (!world_value.is_object()
        || !only_keys(world_value, {"origin", "magnetic_field", "atmosphere"}, owner, error)) {
        return fail(error, owner + " must be an object of origin, magnetic_field and atmosphere");
    }
    if (world_value.contains("origin")) {
        const auto& origin = world_value.at("origin");
        const std::string name = owner + ".origin";
        if (!origin.is_object()
            || !only_keys(origin, {"latitude_deg", "longitude_deg", "altitude_m"}, name, error)
            || !number_field(origin, "latitude_deg", world.latitude_deg, name, error, true)
            || !number_field(origin, "longitude_deg", world.longitude_deg, name, error, true)
            || !number_field(origin, "altitude_m", world.altitude_m, name, error, true)) {
            return fail(error, name + " needs latitude_deg, longitude_deg and altitude_m");
        }
        if (std::fabs(world.latitude_deg) > 90.0 || std::fabs(world.longitude_deg) > 180.0) {
            return fail(error, name + ": latitude must be within +-90 and longitude within +-180 degrees");
        }
        world.has_origin = true;
    }
    if (world_value.contains("magnetic_field")) {
        const auto& field = world_value.at("magnetic_field");
        const std::string name = owner + ".magnetic_field";
        if (!field.is_object()
            || !only_keys(field, {"intensity_nT", "declination_deg", "inclination_deg"}, name, error)
            || !number_field(field, "intensity_nT", world.magnetic_intensity_nT, name, error, true)
            || !number_field(field, "declination_deg", world.magnetic_declination_deg, name, error, true)
            || !number_field(field, "inclination_deg", world.magnetic_inclination_deg, name, error, true)) {
            return fail(error, name + " needs intensity_nT, declination_deg and inclination_deg");
        }
        if (world.magnetic_intensity_nT <= 0.0) {
            return fail(error, name + ".intensity_nT must be positive");
        }
        world.has_magnetic_field = true;
    }
    if (world_value.contains("atmosphere")) {
        const auto& atmosphere = world_value.at("atmosphere");
        const std::string name = owner + ".atmosphere";
        if (!atmosphere.is_object()
            || !only_keys(atmosphere, {"sea_level_pressure_pa", "sea_level_temperature_c"}, name, error)
            || !number_field(atmosphere, "sea_level_pressure_pa", world.sea_level_pressure_pa, name, error, false)
            || !number_field(atmosphere, "sea_level_temperature_c", world.sea_level_temperature_c, name, error, false)) {
            return fail(error, name + " may hold sea_level_pressure_pa and sea_level_temperature_c");
        }
        if (world.sea_level_pressure_pa <= 0.0) {
            return fail(error, name + ".sea_level_pressure_pa must be positive");
        }
    }
    return true;
}

bool load_sensor_seed(const json& runtime_root, RuntimeDefinition& definition, std::string* error)
{
    if (!runtime_root.is_object() || !runtime_root.contains("sensors")) {
        return true;
    }
    const auto& sensors = runtime_root.at("sensors");
    if (!sensors.is_object() || !only_keys(sensors, {"seed", "world"}, "runtime config sensors", error)) {
        return fail(error, "runtime config \"sensors\" must be an object with an optional seed and world");
    }
    if (sensors.contains("seed")) {
        if (!sensors.at("seed").is_number_unsigned()) {
            return fail(error, "runtime config sensors.seed must be a non-negative integer");
        }
        definition.sensor_seed = sensors.at("seed").get<std::uint64_t>();
    }
    if (sensors.contains("world")
        && !load_sensor_world(sensors.at("world"), definition.sensor_world, error)) {
        return false;
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
    const bool cloud = message_type == "sensor_msgs/PointCloud2";
    const bool range = message_type == "sensor_msgs/Range";
    const bool magnetic = message_type == "sensor_msgs/MagneticField";
    const bool pressure = message_type == "sensor_msgs/FluidPressure";
    const bool fix = message_type == "sensor_msgs/NavSatFix";
    const bool image = message_type == "sensor_msgs/Image";
    const bool compressed = message_type == "sensor_msgs/CompressedImage";
    if (!joint_state && !wrench && !imu && !scan && !cloud && !range && !magnetic && !pressure && !fix
        && !image && !compressed) {
        return fail(error, owner + ": message_type must be sensor_msgs/JointState,"
            " geometry_msgs/WrenchStamped, sensor_msgs/Imu, sensor_msgs/LaserScan,"
            " sensor_msgs/PointCloud2, sensor_msgs/Range, sensor_msgs/MagneticField,"
            " sensor_msgs/FluidPressure, sensor_msgs/NavSatFix, sensor_msgs/Image"
            " or sensor_msgs/CompressedImage");
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
            : wrench ? "force_torque" : imu ? "imu" : range ? "rangefinder"
            : magnetic ? "mag" : pressure ? "baro" : fix ? "gps" : image ? "depth_camera"
            : compressed ? "rgb_camera" : "lidar";
        if (found->second->type != expected_type) {
            return fail(error, owner + ": sensor " + name + " is of type " + found->second->type
                + " and cannot go into " + message_type);
        }
        if (scan && found->second->scans_3d) {
            return fail(error, owner + ": sensor " + name + " is a 3D LiDAR and cannot go into "
                + message_type + "; use sensor_msgs/PointCloud2");
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
    if (compressed) {
        // One CompressedImage and one CameraInfo channel per camera of the rig.
        if (!only_keys(pdu, {"message_type", "update_rate_hz", "jpeg_quality", "cameras"},
                owner + ".pdu_config", error)) {
            return false;
        }
        if (pdu.contains("jpeg_quality")) {
            if (!pdu.at("jpeg_quality").is_number_integer()
                || pdu.at("jpeg_quality").get<int>() < 1 || pdu.at("jpeg_quality").get<int>() > 100) {
                return fail(error, owner + ".pdu_config.jpeg_quality must be an integer from 1 to 100");
            }
            output.jpeg_quality = pdu.at("jpeg_quality").get<int>();
        }
        if (!pdu.contains("cameras") || !pdu.at("cameras").is_array() || pdu.at("cameras").empty()) {
            return fail(error, owner + ".pdu_config.cameras must list the channels of each camera");
        }
        const auto& rig = *sensors.at(output.sensors.front());
        std::unordered_set<std::string> listed;
        for (const auto& item : pdu.at("cameras")) {
            RuntimeSensorOutputConfig::CameraChannels channels;
            const std::string where = owner + ".pdu_config.cameras";
            if (!item.is_object()) {
                return fail(error, where + " entries must be objects");
            }
            if (!only_keys(item, {"camera", "pdu_name", "camera_info_pdu_name"}, where, error)
                || !required_string(item, "pdu_name", channels.pdu_name, error, where)
                || !required_string(item, "camera_info_pdu_name", channels.camera_info_pdu_name, error, where)) {
                return false;
            }
            channels.camera = item.value("camera", "");
            if (std::find(rig.camera_names.begin(), rig.camera_names.end(), channels.camera)
                == rig.camera_names.end()) {
                return fail(error, where + ": sensor " + rig.name + " has no camera \"" + channels.camera + "\"");
            }
            if (!listed.insert(channels.camera).second) {
                return fail(error, where + ": camera \"" + channels.camera + "\" listed twice");
            }
            if (!verify_pdu_binding(context, output.pdu_robot, channels.pdu_name,
                    "sensor_msgs/CompressedImage", 1, error)
                || !verify_pdu_binding(context, output.pdu_robot, channels.camera_info_pdu_name,
                    "sensor_msgs/CameraInfo", 1, error)) {
                return false;
            }
            output.cameras.push_back(std::move(channels));
        }
        if (!pdu.contains("update_rate_hz") || !pdu.at("update_rate_hz").is_number()
            || !std::isfinite(pdu.at("update_rate_hz").get<double>())
            || pdu.at("update_rate_hz").get<double>() <= 0.0) {
            return fail(error, owner + ": update_rate_hz must be finite and positive");
        }
        output.update_rate_hz = pdu.at("update_rate_hz").get<double>();
        output.pdu_name = output.cameras.front().pdu_name;
        return true;
    }
    if (!only_keys(pdu, image
                ? std::unordered_set<std::string>{"pdu_name", "camera_info_pdu_name", "message_type", "update_rate_hz"}
                : std::unordered_set<std::string>{"pdu_name", "message_type", "update_rate_hz"},
            owner + ".pdu_config", error)
        || !required_string(pdu, "pdu_name", output.pdu_name, error, owner + ".pdu_config")) {
        return false;
    }
    // ROS: an image goes with its CameraInfo on a second channel, same stamp.
    if (image
        && (!required_string(pdu, "camera_info_pdu_name", output.camera_info_pdu_name, error,
                owner + ".pdu_config")
            || !verify_pdu_binding(context, output.pdu_robot, output.camera_info_pdu_name,
                "sensor_msgs/CameraInfo", 1, error))) {
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

    // Sensors that need to know where the world is on the earth.
    for (const auto& sensor_config : definition.sensors) {
        const std::string owner = "sensor " + sensor_config.name;
        if ((sensor_config.type == "gps" || sensor_config.type == "baro")
            && !definition.sensor_world.has_origin) {
            return fail(error, owner + ": a " + sensor_config.type
                + " needs runtime config sensors.world.origin (where the world is on the earth)");
        }
        if (sensor_config.type == "mag" && !definition.sensor_world.has_magnetic_field) {
            return fail(error, owner + ": a magnetometer needs runtime config"
                " sensors.world.magnetic_field (the earth field at the world)");
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
