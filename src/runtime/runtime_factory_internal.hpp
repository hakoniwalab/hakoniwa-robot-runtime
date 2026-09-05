#pragma once

#include "runtime/runtime_definition.hpp"
#include "runtime/runtime_factory_manifest.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace hakoniwa::robot_runtime::runtime::detail {

using json = nlohmann::json;
namespace fs = std::filesystem;

struct RuntimeParseContext {
    fs::path base_path;
    fs::path pdu_definition_path;
    json runtime_root;
    std::vector<json> components;
    std::unordered_map<std::string, json> pdu_types;
};

[[nodiscard]] bool fail(std::string* error, std::string message);
[[nodiscard]] bool read_json(
    const fs::path& path,
    json& output,
    std::string* error);
[[nodiscard]] bool required_string(
    const json& object,
    const char* field,
    std::string& output,
    std::string* error,
    std::string_view owner);
[[nodiscard]] fs::path resolve(
    const fs::path& base,
    const std::string& value);
[[nodiscard]] bool regular_file(
    const fs::path& path,
    std::string* error);
[[nodiscard]] bool verify_pdu_binding(
    RuntimeParseContext& context,
    const std::string& pdu_robot,
    const std::string& pdu_name,
    const std::string& expected_type,
    std::size_t minimum_size,
    std::string* error);

[[nodiscard]] bool load_actuator_definitions(
    RuntimeParseContext& context,
    RuntimeDefinition& definition,
    std::string* error);
[[nodiscard]] bool load_trajectory_definitions(
    RuntimeParseContext& context,
    RuntimeDefinition& definition,
    std::string* error);
[[nodiscard]] bool load_manual_definitions(
    RuntimeParseContext& context,
    RuntimeDefinition& definition,
    std::string* error);
[[nodiscard]] bool load_state_output_definitions(
    RuntimeParseContext& context,
    RuntimeDefinition& definition,
    std::string* error);

} // namespace hakoniwa::robot_runtime::runtime::detail
