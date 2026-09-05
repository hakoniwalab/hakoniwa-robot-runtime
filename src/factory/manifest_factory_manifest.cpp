#include "factory/manifest_factory_internal.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <string_view>
#include <utility>

namespace hakoniwa::robot_runtime::factory::detail {
namespace {

using json = nlohmann::json;
namespace fs = std::filesystem;

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

} // namespace

bool resolve_manifest(
    const std::string& manifest_path,
    ResolvedManifest& output,
    std::string* error_message)
{
    if (manifest_path.empty()) {
        return fail(error_message,
            "manifest path must not be empty");
    }

    const fs::path normalized =
        fs::absolute(manifest_path).lexically_normal();
    if (!regular_file(normalized, error_message)) {
        return false;
    }

    json root;
    if (!read_json(normalized, root, error_message)) {
        return false;
    }
    if (!root.is_object()) {
        return fail(error_message,
            "manifest root must be an object: "
            + normalized.string());
    }

    ResolvedManifest resolved;
    std::string model;
    std::string pdu_definition;
    std::string endpoint;
    std::string runtime_config;
    if (!required_string(root, "name", resolved.name,
            error_message, "manifest")
        || !required_string(root, "model", model,
            error_message, "manifest")
        || !required_string(root, "pdu_def", pdu_definition,
            error_message, "manifest")
        || !required_string(root, "endpoint", endpoint,
            error_message, "manifest")
        || !required_string(root, "runtime_config", runtime_config,
            error_message, "manifest")) {
        return false;
    }

    if (!root.contains("components")
        || !root.at("components").is_array()) {
        return fail(error_message,
            "manifest components must be an array");
    }

    const fs::path base = normalized.parent_path();
    const fs::path model_path = resolve(base, model);
    const fs::path pdu_definition_path =
        resolve(base, pdu_definition);
    const fs::path endpoint_path = resolve(base, endpoint);
    const fs::path runtime_config_path =
        resolve(base, runtime_config);

    if (!regular_file(model_path, error_message)
        || !regular_file(pdu_definition_path, error_message)
        || !regular_file(endpoint_path, error_message)
        || !regular_file(runtime_config_path, error_message)) {
        return false;
    }

    resolved.pdu_definition_path = pdu_definition_path.string();
    resolved.endpoint_path = endpoint_path.string();
    resolved.runtime_input = {
        base.string(),
        model_path.string(),
        pdu_definition_path.string(),
        runtime_config_path.string(),
        root.at("components").dump(),
    };

    output = std::move(resolved);
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

} // namespace hakoniwa::robot_runtime::factory::detail
