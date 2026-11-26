#pragma once

#include "runtime/service/service_plugin_adapter.hpp"

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

struct RobotPluginDeclaration {
    std::vector<std::string> service_types;
    PlantCapabilities plant_operations;
};

RobotPluginDeclaration parse_robot_plugin_declaration(std::string_view json);
std::filesystem::path robot_plugin_declaration_path(const std::filesystem::path& library);
std::vector<std::string> validate_robot_plugin(std::string_view robot, const RobotPluginDeclaration& declaration,
    const std::vector<RobotServiceSpec>& services, const PlantCapabilities& plant);

void require_robot_plugin_declaration(const plugin::PluginLibrary& library, const ServicePluginConfig& config,
    const PlantCapabilities& plant);

} // namespace hakoniwa::robot_runtime::runtime
