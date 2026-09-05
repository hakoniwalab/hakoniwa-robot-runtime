#include "runtime/runtime_factory_internal.hpp"

#include <fstream>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime::detail {

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
    if (!detail::load_actuator_definitions(
            context, definition, error_message)
        || !detail::load_trajectory_definitions(
            context, definition, error_message)
        || !detail::load_manual_definitions(
            context, definition, error_message)
        || !detail::load_state_output_definitions(
            context, definition, error_message)) {
        return false;
    }

    output = std::move(definition);
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

} // namespace hakoniwa::robot_runtime::runtime
