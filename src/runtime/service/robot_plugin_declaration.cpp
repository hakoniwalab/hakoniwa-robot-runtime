#include "runtime/service/robot_plugin_declaration.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
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
    const auto& source = library.path();
    const auto* text = library.service_table().declaration_json;
    if (text == nullptr) {
        throw std::invalid_argument(source + ": missing robot plugin declaration in service table");
    }
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
