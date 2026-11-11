#include "runtime/sensor/sensor_catalog.hpp"

#include <fstream>
#include <utility>

namespace hakoniwa::robot_runtime::runtime::sensor {

namespace {

using json = nlohmann::json;

bool fail(std::string* error, std::string message)
{
    if (error != nullptr) {
        *error = std::move(message);
    }
    return false;
}

std::string join(const std::vector<std::string>& items)
{
    std::string text;
    for (const auto& item : items) {
        if (!text.empty()) {
            text += ", ";
        }
        text += item;
    }
    return text;
}

bool is_valid_path(const std::string& path)
{
    if (path.empty() || path.front() == '.' || path.back() == '.') {
        return false;
    }
    return path.find("..") == std::string::npos;
}

// The path is supported itself, or one of its ancestors is.
bool is_covered(const std::vector<std::string>& supported, const std::string& path)
{
    for (const auto& entry : supported) {
        if (entry == path) {
            return true;
        }
        if (path.size() > entry.size() && path.compare(0, entry.size(), entry) == 0
            && path[entry.size()] == '.') {
            return true;
        }
    }
    return false;
}

// Some supported path lies below this one, so its children must be checked.
bool has_supported_descendant(const std::vector<std::string>& supported, const std::string& path)
{
    const auto prefix = path + ".";
    for (const auto& entry : supported) {
        if (entry.size() > prefix.size() && entry.compare(0, prefix.size(), prefix) == 0) {
            return true;
        }
    }
    return false;
}

void collect_unsupported(
    const std::vector<std::string>& supported,
    const json& object,
    const std::string& prefix,
    std::vector<std::string>& output)
{
    for (const auto& [key, value] : object.items()) {
        const auto path = prefix.empty() ? key : prefix + "." + key;
        if (prefix.empty() && (key == "type" || key == "$schema")) {
            continue;
        }
        if (is_covered(supported, path)) {
            continue;
        }
        if (value.is_object() && has_supported_descendant(supported, path)) {
            collect_unsupported(supported, value, path, output);
            continue;
        }
        output.push_back(path);
    }
}

} // namespace

bool parse_sensor_catalog(const json& value, SensorCatalog& output, std::string* error)
{
    SensorCatalog catalog;
    if (!value.is_object() || !value.contains("implementations")
        || !value.at("implementations").is_object()) {
        return fail(error, "sensor catalog requires an \"implementations\" object");
    }
    for (const auto& [name, implementation] : value.at("implementations").items()) {
        if (name.empty()) {
            return fail(error, "sensor catalog has an empty implementation name");
        }
        if (!implementation.is_object() || !implementation.contains("sensors")
            || !implementation.at("sensors").is_object()) {
            return fail(error, "sensor catalog implementation \"" + name
                + "\" requires a \"sensors\" object");
        }
        auto& sensors = catalog.implementations[name];
        for (const auto& [type, sensor] : implementation.at("sensors").items()) {
            if (!sensor.is_object() || !sensor.contains("keys")
                || !sensor.at("keys").is_array()) {
                return fail(error, "sensor catalog entry \"" + name + "/" + type
                    + "\" requires a \"keys\" array");
            }
            auto& keys = sensors[type];
            for (const auto& key : sensor.at("keys")) {
                if (!key.is_string() || !is_valid_path(key.get<std::string>())) {
                    return fail(error, "sensor catalog entry \"" + name + "/" + type
                        + "\" has an invalid key path");
                }
                keys.push_back(key.get<std::string>());
            }
        }
    }
    output = std::move(catalog);
    return true;
}

bool load_sensor_catalog(
    const std::filesystem::path& path,
    SensorCatalog& output,
    std::string* error)
{
    std::ifstream stream(path);
    if (!stream.is_open()) {
        return fail(error, "cannot open sensor catalog: " + path.string());
    }
    json value;
    try {
        stream >> value;
    } catch (const json::exception& exception) {
        return fail(error, "cannot parse sensor catalog " + path.string() + ": "
            + exception.what());
    }
    std::string reason;
    if (!parse_sensor_catalog(value, output, &reason)) {
        return fail(error, path.string() + ": " + reason);
    }
    return true;
}

std::vector<std::string> find_unsupported_keys(
    const std::vector<std::string>& supported_paths,
    const json& definition)
{
    std::vector<std::string> unsupported;
    if (definition.is_object()) {
        collect_unsupported(supported_paths, definition, "", unsupported);
    }
    return unsupported;
}

bool validate_sensor_definition(
    const SensorCatalog& catalog,
    const std::set<std::string>& linked_implementations,
    const std::string& implementation,
    const json& definition,
    const std::string& source,
    std::string* error)
{
    const auto found = catalog.implementations.find(implementation);
    if (found == catalog.implementations.end()) {
        std::vector<std::string> known;
        for (const auto& [name, sensors] : catalog.implementations) {
            known.push_back(name);
        }
        return fail(error, source + ": unknown sensor implementation \""
            + implementation + "\" (known: " + join(known) + ")");
    }
    if (!definition.is_object() || !definition.contains("type")
        || !definition.at("type").is_string()) {
        return fail(error, source + ": sensor definition requires a string \"type\"");
    }
    const auto type = definition.at("type").get<std::string>();
    const auto sensor = found->second.find(type);
    if (sensor == found->second.end()) {
        return fail(error, source + ": sensor implementation \"" + implementation
            + "\" does not provide sensor type \"" + type + "\"");
    }
    const auto unsupported = find_unsupported_keys(sensor->second, definition);
    if (!unsupported.empty()) {
        return fail(error, source + ": " + implementation + "/" + type
            + " does not support: " + join(unsupported));
    }
    if (linked_implementations.count(implementation) == 0) {
        return fail(error, source + ": sensor implementation \"" + implementation
            + "\" is in the catalog but not linked into this application");
    }
    return true;
}

} // namespace hakoniwa::robot_runtime::runtime::sensor
