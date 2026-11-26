#include "runtime/service/robot_plugin_declaration.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace hakoniwa::robot_runtime::runtime {
namespace {

using nlohmann::json;

struct PlantOperation {
    std::string_view name;
    bool PlantCapabilities::*supported;
};

constexpr PlantOperation plant_operations[] {
    {"surface_friction", &PlantCapabilities::surface_friction},
    {"body_motion", &PlantCapabilities::body_motion},
    {"humanoid_motion", &PlantCapabilities::humanoid_motion},
};

std::vector<std::string> read_strings(const json& values, std::string_view field)
{
    if (!values.is_array()) {
        throw std::invalid_argument(std::string(field) + " must be an array of strings");
    }
    std::vector<std::string> result;
    for (const auto& value : values) {
        if (!value.is_string()) {
            throw std::invalid_argument(std::string(field) + " must contain only strings");
        }
        result.push_back(value.get<std::string>());
    }
    return result;
}

std::string read_declaration_file(const std::filesystem::path& path)
{
    if (!std::filesystem::is_regular_file(path)) {
        throw std::runtime_error("missing robot plugin declaration file: " + path.string());
    }
    std::ifstream stream(path);
    if (!stream) {
        throw std::runtime_error("failed to open robot plugin declaration file: " + path.string());
    }
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

} // namespace

RobotPluginDeclaration parse_robot_plugin_declaration(std::string_view text)
{
    json document;
    try {
        document = json::parse(text);
    } catch (const json::exception& error) {
        throw std::invalid_argument("invalid robot plugin declaration JSON: " + std::string(error.what()));
    }
    if (!document.is_object()) {
        throw std::invalid_argument("robot plugin declaration JSON must be an object");
    }
    for (const auto& field : document.items()) {
        if (field.key() != "service_types" && field.key() != "plant_operations") {
            throw std::invalid_argument("unknown robot plugin declaration field " + field.key());
        }
    }
    if (!document.contains("service_types")) {
        throw std::invalid_argument("robot plugin declaration requires service_types");
    }

    RobotPluginDeclaration declaration {read_strings(document.at("service_types"), "service_types"), {}};
    if (document.contains("plant_operations")) {
        for (const auto& name : read_strings(document.at("plant_operations"), "plant_operations")) {
            const auto operation = std::ranges::find(plant_operations, name, &PlantOperation::name);
            if (operation == std::end(plant_operations)) {
                throw std::invalid_argument("unknown plant_operations entry " + name);
            }
            declaration.plant_operations.*(operation->supported) = true;
        }
    }
    return declaration;
}

std::filesystem::path robot_plugin_declaration_path(const std::filesystem::path& library)
{
    auto path = library;
    return path.replace_extension(".plugin.json");
}

std::vector<std::string> validate_robot_plugin(std::string_view robot, const RobotPluginDeclaration& declaration,
    const std::vector<RobotServiceSpec>& services, const PlantCapabilities& plant)
{
    const std::string prefix = "robot " + std::string(robot) + ": ";
    std::vector<std::string> problems;
    for (const auto& service : services) {
        if (std::ranges::find(declaration.service_types, service.type) == declaration.service_types.end()) {
            problems.push_back(prefix + "service " + service.name + " has undeclared type " + service.type);
        }
    }
    for (const auto& operation : plant_operations) {
        if (declaration.plant_operations.*(operation.supported) && !(plant.*(operation.supported))) {
            problems.push_back(prefix + "Plant does not support operation " + std::string(operation.name));
        }
    }
    return problems;
}

void require_robot_plugin_declaration(const plugin::PluginLibrary& library, const ServicePluginConfig& config,
    const PlantCapabilities& plant)
{
    if (library.is_builtin() && !library.builtin_declaration_json()) {
        return;
    }
    const auto source = library.is_builtin() ? library.path() : robot_plugin_declaration_path(library.path()).string();
    const auto text = library.is_builtin() ? *library.builtin_declaration_json() : read_declaration_file(source);
    RobotPluginDeclaration declaration;
    try {
        declaration = parse_robot_plugin_declaration(text);
    } catch (const std::invalid_argument& error) {
        throw std::invalid_argument(source + ": " + error.what());
    }
    const auto problems = validate_robot_plugin(config.robot, declaration, config.services, plant);
    if (!problems.empty()) {
        std::string message = "invalid robot plugin declaration: " + source;
        for (const auto& problem : problems) {
            message += "\n  " + problem;
        }
        throw std::invalid_argument(message);
    }
}

} // namespace hakoniwa::robot_runtime::runtime
