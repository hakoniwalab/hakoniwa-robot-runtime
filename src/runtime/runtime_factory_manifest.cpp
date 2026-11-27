#include "runtime/runtime_factory_internal.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime::detail {

namespace {

bool load_number_triplet(
    const json& object,
    const char* field,
    std::array<double, 3>& output,
    std::string* error)
{
    if (!object.contains(field) || !object.at(field).is_array()
        || object.at(field).size() != output.size()) {
        return fail(error,
            std::string("initial body pose requires a three-number array: ")
            + field);
    }
    for (std::size_t index = 0; index < output.size(); ++index) {
        const auto& value = object.at(field).at(index);
        if (!value.is_number()) {
            return fail(error,
                std::string("initial body pose contains a non-number: ")
                + field);
        }
        output[index] = value.get<double>();
        if (!std::isfinite(output[index])) {
            return fail(error,
                std::string("initial body pose contains a non-finite value: ")
                + field);
        }
    }
    return true;
}

bool load_initial_body_poses(
    const json& runtime_root,
    RuntimeDefinition& definition,
    std::string* error)
{
    if (!runtime_root.contains("initial_body_poses")) {
        return true;
    }
    const auto& poses = runtime_root.at("initial_body_poses");
    if (!poses.is_array()) {
        return fail(error, "initial_body_poses must be an array");
    }
    std::unordered_set<std::string> freejoints;
    for (const auto& pose : poses) {
        RuntimeInitialBodyPoseConfig config;
        std::array<double, 3> position {};
        if (!required_string(
                pose, "mjcf_freejoint", config.mjcf_freejoint,
                error, "initial body pose")
            || !load_number_triplet(pose, "position_m", position, error)
            || !load_number_triplet(
                pose, "orientation_rpy_rad", config.rpy_rad, error)) {
            return false;
        }
        if (!freejoints.insert(config.mjcf_freejoint).second) {
            return fail(error,
                "duplicate initial body pose freejoint: "
                + config.mjcf_freejoint);
        }
        config.position = {position[0], position[1], position[2]};
        definition.initial_body_poses.push_back(std::move(config));
    }
    return true;
}

bool load_person_vehicles(
    const json& runtime_root,
    RuntimeDefinition& definition,
    std::string* error)
{
    if (!runtime_root.contains("person_vehicles")) {
        return true;
    }
    const auto& vehicles = runtime_root.at("person_vehicles");
    if (!vehicles.is_array()) {
        return fail(error, "person_vehicles must be an array");
    }
    std::unordered_set<std::string> names;
    std::unordered_set<std::string> freejoints;
    for (const auto& vehicle : vehicles) {
        RuntimePersonVehicleConfig config;
        if (!required_string(vehicle, "name", config.name, error, "person vehicle")
            || !required_string(
                vehicle, "mjcf_freejoint", config.mjcf_freejoint,
                error, "person vehicle")) {
            return false;
        }
        if (!names.insert(config.name).second) {
            return fail(error, "duplicate person vehicle name: " + config.name);
        }
        if (!freejoints.insert(config.mjcf_freejoint).second) {
            return fail(error,
                "duplicate person vehicle freejoint: " + config.mjcf_freejoint);
        }
        if (!vehicle.contains("seats") || !vehicle.at("seats").is_object()
            || vehicle.at("seats").empty()) {
            return fail(error,
                "person vehicle seats must be a non-empty object: " + config.name);
        }
        for (const auto& [seat_name, value] : vehicle.at("seats").items()) {
            if (seat_name.empty() || !value.is_array() || value.size() != 3) {
                return fail(error,
                    "person vehicle seat requires a name and three-number array: "
                    + config.name + "/" + seat_name);
            }
            RuntimePersonSeatConfig seat;
            seat.name = seat_name;
            double* coordinates[] {&seat.offset.x, &seat.offset.y, &seat.offset.z};
            for (std::size_t index = 0; index < 3; ++index) {
                if (!value.at(index).is_number()) {
                    return fail(error,
                        "person vehicle seat contains a non-number: "
                        + config.name + "/" + seat_name);
                }
                *coordinates[index] = value.at(index).get<double>();
                if (!std::isfinite(*coordinates[index])) {
                    return fail(error,
                        "person vehicle seat contains a non-finite value: "
                        + config.name + "/" + seat_name);
                }
            }
            config.seats.push_back(std::move(seat));
        }
        definition.person_vehicles.push_back(std::move(config));
    }
    return true;
}

bool load_person_initial_poses(
    const json& runtime_root,
    RuntimeDefinition& definition,
    std::string* error)
{
    if (!runtime_root.contains("person_initial_poses")) {
        return true;
    }
    const auto& poses = runtime_root.at("person_initial_poses");
    if (!poses.is_array()) {
        return fail(error, "person_initial_poses must be an array");
    }
    std::unordered_set<std::string> names;
    for (const auto& pose : poses) {
        RuntimePersonInitialPoseConfig config;
        if (!required_string(
                pose, "name", config.name, error, "person initial pose")) {
            return false;
        }
        if (!names.insert(config.name).second) {
            return fail(error,
                "duplicate person initial pose name: " + config.name);
        }
        if (!pose.contains("yaw_rad") || !pose.at("yaw_rad").is_number()) {
            return fail(error,
                "person initial pose requires numeric yaw_rad: " + config.name);
        }
        config.yaw_rad = pose.at("yaw_rad").get<double>();
        if (!std::isfinite(config.yaw_rad)) {
            return fail(error,
                "person initial pose yaw_rad must be finite: " + config.name);
        }
        definition.person_initial_poses.push_back(std::move(config));
    }
    return true;
}

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN) && HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN
bool load_service_plugin(const json& robot, const fs::path& base_path,
    RuntimeServiceConfig& config, std::string* error)
{
    const auto& name = config.pdu_robot;
    const auto& plugin = robot.at("plugin");
    if (!plugin.is_object()) {
        return fail(error, "robot " + name + " needs a plugin object");
    }
    for (const auto& [field, value] : plugin.items()) {
        if (field != "library" && field != "config") {
            return fail(error, "unknown plugin field " + field + " in robot " + name);
        }
    }
    std::string library;
    if (!required_string(plugin, "library", library, error, "robot " + name + " plugin")) {
        return false;
    }
    const auto path = resolve(base_path, library);
    if (!regular_file(path, error)) {
        return false;
    }
    config.plugin_library = path.string();
    config.plugin_config_json = plugin.contains("config") ? plugin.at("config").dump() : "{}";
    if (robot.contains("priority")) {
        const auto& value = robot.at("priority");
        if (!value.is_number_integer()
            || (value.is_number_unsigned()
                && value.get<std::uint64_t>() > std::numeric_limits<std::int32_t>::max())
            || value.get<std::int64_t>() < std::numeric_limits<std::int32_t>::min()
            || value.get<std::int64_t>() > std::numeric_limits<std::int32_t>::max()) {
            return fail(error, "robot " + name + " priority must be a 32-bit integer");
        }
        config.priority = value.get<std::int32_t>();
    }
    return true;
}
#endif

bool load_robot_services(const json& robot, const RuntimeServiceConfig& robot_config,
    std::unordered_set<std::string>& service_names, RuntimeDefinition& definition, std::string* error)
{
    if (!robot.contains("services") || !robot.at("services").is_array()) {
        return fail(error, "robot " + robot_config.pdu_robot + " services must be an array");
    }
    for (const auto& service : robot.at("services")) {
        auto config = robot_config;
        if (!required_string(service, "name", config.name, error, "service")) {
            return false;
        }
        for (const auto& [field, value] : service.items()) {
            if (field == "lockstep") {
                if (!value.is_boolean()) {
                    return fail(error, "service " + config.name + " lockstep must be a boolean");
                }
                config.lockstep = value.get<bool>();
            } else if (field == "type") {
                // The generator consumes this hint. Runtime uses the type registered with Core.
                const auto* type = value.get_ptr<const std::string*>();
                const auto separator = type ? type->find('/') : std::string::npos;
                if (separator == std::string::npos || separator == 0 || separator + 1 == type->size()
                    || type->find('/', separator + 1) != std::string::npos) {
                    return fail(error, "service " + config.name + " type must be package/Service");
                }
            } else if (field != "name") {
                return fail(error, "unknown field " + field + " in service " + config.name);
            }
        }
        if (!service_names.insert(config.name).second) {
            return fail(error, "duplicate service name: " + config.name);
        }
        definition.services.push_back(std::move(config));
    }
    return true;
}

bool load_robots(std::string_view robots_json, [[maybe_unused]] const fs::path& base_path,
    RuntimeDefinition& definition, std::string* error)
{
    const auto robots = json::parse(robots_json, nullptr, false);
    if (!robots.is_array()) {
        return fail(error, "manifest robots must be an array");
    }

    std::unordered_set<std::string> robot_names;
    std::unordered_set<std::string> service_names;
    for (const auto& robot : robots) {
        RuntimeServiceConfig config {};
        if (!required_string(robot, "name", config.pdu_robot, error, "robot")) {
            return false;
        }
        if (!robot_names.insert(config.pdu_robot).second) {
            return fail(error, "duplicate robot name: " + config.pdu_robot);
        }

        const bool has_plugin = robot.contains("plugin");
        for (const auto& [field, value] : robot.items()) {
            if (field != "name" && field != "services" && field != "builtin" && field != "plugin"
                && !(has_plugin && field == "priority")) {
                return fail(error, "unknown field " + field + " in robot " + config.pdu_robot);
            }
        }
        if (has_plugin == robot.contains("builtin")) {
            return fail(error, "robot " + config.pdu_robot + " must specify exactly one of plugin or builtin");
        }
        if (has_plugin) {
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN) && HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN
            config.type = RuntimeServiceType::Plugin;
            if (!load_service_plugin(robot, base_path, config, error)) {
                return false;
            }
#else
            return fail(error, "service plugins are disabled: " + config.pdu_robot);
#endif
        } else {
            std::string builtin;
            if (!required_string(robot, "builtin", builtin, error, "robot " + config.pdu_robot)) {
                return false;
            }
            if (builtin == "ackermann_step") {
                config.type = RuntimeServiceType::AckermannStep;
            } else if (builtin == "person_step") {
                config.type = RuntimeServiceType::PersonStep;
            } else {
                return fail(error, "unknown builtin " + builtin + " for robot " + config.pdu_robot);
            }
        }
        if (!load_robot_services(robot, config, service_names, definition, error)) {
            return false;
        }
    }
    return true;
}

} // namespace

bool fail(std::string* error, std::string message)
{
    if (error != nullptr) {
        *error = std::move(message);
    }
    return false;
}

bool read_json(
    const fs::path& path,
    json& output,
    std::string* error)
{
    std::ifstream stream(path);
    if (!stream.is_open()) {
        return fail(error,
            "failed to open JSON file: " + path.string());
    }
    try {
        stream >> output;
    } catch (const std::exception& exception) {
        return fail(error,
            "failed to parse JSON file " + path.string()
            + ": " + exception.what());
    }
    return true;
}

bool required_string(
    const json& object,
    const char* field,
    std::string& output,
    std::string* error,
    const std::string_view owner)
{
    if (!object.is_object()
        || !object.contains(field)
        || !object.at(field).is_string()
        || object.at(field).get_ref<const std::string&>().empty()) {
        return fail(error,
            std::string(owner)
            + " field is missing or invalid: "
            + field);
    }
    output = object.at(field).get<std::string>();
    return true;
}

fs::path resolve(
    const fs::path& base,
    const std::string& value)
{
    const fs::path path(value);
    return (path.is_absolute() ? path : base / path).lexically_normal();
}

bool regular_file(
    const fs::path& path,
    std::string* error)
{
    std::error_code ec;
    if (!fs::is_regular_file(path, ec)) {
        return fail(error,
            "referenced file does not exist: " + path.string());
    }
    return true;
}

static bool load_pdu_types(
    RuntimeParseContext& context,
    const std::string& robot,
    json& types,
    std::string* error)
{
    json definition;
    if (!read_json(context.pdu_definition_path, definition, error)) {
        return false;
    }
    if (!definition.contains("paths")
        || !definition.at("paths").is_array()
        || !definition.contains("robots")
        || !definition.at("robots").is_array()) {
        return fail(error,
            "PDU definition requires paths and robots arrays");
    }

    std::string types_id;
    for (const auto& item : definition.at("robots")) {
        if (item.value("name", "") == robot) {
            types_id = item.value("pdutypes_id", "");
            break;
        }
    }
    if (types_id.empty()) {
        return fail(error,
            "PDU robot is not defined: " + robot);
    }

    std::string types_path;
    for (const auto& item : definition.at("paths")) {
        if (item.value("id", "") == types_id) {
            types_path = item.value("path", "");
            break;
        }
    }
    if (types_path.empty()) {
        return fail(error,
            "PDU types path is not defined for ID: " + types_id);
    }

    return read_json(
        resolve(context.pdu_definition_path.parent_path(), types_path),
        types,
        error);
}

#if !(defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR)
// A build without sensor support must not silently ignore sensors.
static bool reject_sensor_components(
    const RuntimeParseContext& context,
    std::string* error)
{
    for (const auto& component : context.components) {
        const auto kind = component.value("kind", "");
        if (kind == "sensor"
            || (kind == "state_output"
                && component.value("type", "") == "sensor_output")) {
            return fail(error,
                "component " + component.value("id", std::string("?"))
                + " is a sensor, but this application is built without sensor support"
                  " (HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR)");
        }
    }
    if (context.runtime_root.is_object() && context.runtime_root.contains("sensors")) {
        return fail(error,
            "runtime config has \"sensors\", but this application is built without"
            " sensor support (HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR)");
    }
    return true;
}
#endif

#if !(defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN) && HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN)
static bool reject_plugin_controller_components(
    const RuntimeParseContext& context,
    std::string* error)
{
    for (const auto& component : context.components) {
        if (component.value("kind", "") == "controller"
            && component.value("type", "") == "plugin") {
            return fail(error,
                "component " + component.value("id", std::string("?"))
                + " is a controller plugin, but this application is built without"
                  " controller plugin support"
                  " (HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN)");
        }
    }
    return true;
}
#endif

static bool validate_component_headers(
    const std::vector<json>& components,
    std::string* error)
{
    std::unordered_set<std::string> component_ids;
    for (const auto& component : components) {
        std::string id;
        std::string kind;
        std::string type;
        std::string config;
        if (!required_string(component, "id", id, error, "component")
            || !required_string(component, "kind", kind, error, "component")
            || !required_string(component, "type", type, error, "component")
            || !required_string(component, "config", config, error, "component")) {
            return false;
        }
        if (!component_ids.insert(id).second) {
            return fail(error,
                "duplicate component ID: " + id);
        }
    }
    return true;
}

bool verify_pdu_binding(
    RuntimeParseContext& context,
    const std::string& pdu_robot,
    const std::string& pdu_name,
    const std::string& expected_type,
    const std::size_t minimum_size,
    std::string* error)
{
    auto [iterator, inserted] =
        context.pdu_types.try_emplace(pdu_robot);
    if (inserted && !load_pdu_types(
            context,
            pdu_robot,
            iterator->second,
            error)) {
        return false;
    }
    if (!iterator->second.is_array()) {
        return fail(error,
            "PDU types root must be an array for robot: "
            + pdu_robot);
    }
    for (const auto& channel : iterator->second) {
        if (channel.value("name", "") != pdu_name) {
            continue;
        }
        if (channel.value("type", "") != expected_type) {
            return fail(error,
                "PDU channel type must be " + expected_type + ": "
                + pdu_robot + "/" + pdu_name);
        }
        if (!channel.contains("pdu_size")
            || !channel.at("pdu_size").is_number_unsigned()
            || channel.at("pdu_size").get<std::size_t>() < minimum_size) {
            return fail(error,
                "PDU channel size must be at least "
                + std::to_string(minimum_size)
                + " bytes for " + expected_type + ": "
                + pdu_robot + "/" + pdu_name);
        }
        return true;
    }
    return fail(error,
        "PDU channel is not defined: "
        + pdu_robot + "/" + pdu_name);
}

} // namespace hakoniwa::robot_runtime::runtime::detail

namespace hakoniwa::robot_runtime::runtime {

bool resolve_runtime_definition(
    const RuntimeFactoryInput& input,
    RuntimeDefinition& output,
    std::string* error_message)
{
    if (input.manifest_base_path.empty()
        || input.model_path.empty()
        || input.pdu_definition_path.empty()
        || input.runtime_config_path.empty()
        || input.components_json.empty()) {
        return detail::fail(
            error_message,
            "RuntimeFactory input contains unresolved manifest data");
    }

    detail::RuntimeParseContext context;
    context.base_path = input.manifest_base_path;
    context.pdu_definition_path = input.pdu_definition_path;

    detail::json components_root;
    try {
        components_root = detail::json::parse(input.components_json);
    } catch (const std::exception& exception) {
        return detail::fail(
            error_message,
            "failed to parse delegated manifest components: "
            + std::string(exception.what()));
    }
    if (!components_root.is_array()) {
        return detail::fail(
            error_message,
            "delegated manifest components must be an array");
    }
    context.components.reserve(components_root.size());
    for (const auto& component : components_root) {
        context.components.push_back(component);
    }

    if (!detail::read_json(
            input.runtime_config_path,
            context.runtime_root,
            error_message)) {
        return false;
    }
    if (!detail::validate_component_headers(
            context.components,
            error_message)) {
        return false;
    }

    RuntimeDefinition definition;
    definition.model_path = input.model_path;
    if (!detail::load_robots(input.robots_json, context.base_path, definition, error_message)
        || !detail::load_initial_body_poses(
            context.runtime_root, definition, error_message)
        || !detail::load_person_initial_poses(
            context.runtime_root, definition, error_message)
        || !detail::load_person_vehicles(
            context.runtime_root, definition, error_message)
        || !detail::load_actuator_definitions(
            context, definition, error_message)
        || !detail::load_trajectory_definitions(
            context, definition, error_message)
        || !detail::load_manual_definitions(
            context, definition, error_message)
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN) && HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN
        || !detail::load_plugin_controller_definitions(
            context, definition, error_message)
#else
        || !detail::reject_plugin_controller_components(context, error_message)
#endif
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MOBILE_BASE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MOBILE_BASE
        || !detail::load_ackermann_definitions(
            context, definition, error_message)
#endif
        || !detail::load_state_output_definitions(
            context, definition, error_message)
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR
        || !detail::load_sensor_definitions(
            context, definition, error_message)
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN) && HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN
        || !detail::resolve_plugin_controller_pdus(definition, error_message)
#endif
#else
        || !detail::reject_sensor_components(context, error_message)
#endif
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
        || !detail::load_mirror_definitions(
            context, definition, error_message)
        || !detail::load_geom_friction_definitions(
            context, definition, error_message)
#endif
        ) {
        return false;
    }

    output = std::move(definition);
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

} // namespace hakoniwa::robot_runtime::runtime
