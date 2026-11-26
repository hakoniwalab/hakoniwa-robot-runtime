// Stage 2c-1: the robot plugin interface itself is checked by test_robot_plugin.cpp.
// This test keeps the manifest side: how a plugin service row is read.
#undef NDEBUG

#include "runtime/runtime_factory_manifest.hpp"
#include "plugin/plugin_library.hpp"

#include <array>
#include <cassert>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>

#include <nlohmann/json.hpp>

using namespace hakoniwa::robot_runtime;
using namespace hakoniwa::robot_runtime::runtime;

namespace {

void expect_load_error(const char* path, std::string_view message)
{
    try {
        (void)plugin::PluginLibrary::open(path, HAKO_PLUGIN_KIND_SERVICE);
        assert(false);
    } catch (const std::runtime_error& error) {
        const std::string_view text = error.what();
        assert(text.find(std::filesystem::path(path).filename().string()) != text.npos);
        assert(text.find(message) != text.npos);
    }
}

RuntimeServiceConfig parse_manifest(std::filesystem::path path, const char* wrong_kind)
{
    const auto base = path.parent_path();
    const auto fixture = std::filesystem::current_path();
    const auto runtime_path = fixture / "service-test-runtime.json";
    const auto pdu_path = fixture / "service-test-pdus.json";
    const auto actuator_path = fixture / "service-test-actuator.json";
    std::ofstream(runtime_path) << R"({"actuators":{"joint":{"command_timeout_sec":0.1}}})";
    std::ofstream(pdu_path) << R"({
        "paths":[{"id":"types","path":"service-test-types.json"}],
        "robots":[{"name":"robot","pdutypes_id":"types"}]
    })";
    std::ofstream(fixture / "service-test-types.json")
        << R"([{"name":"command","type":"std_msgs/Float64","pdu_size":32}])";
    std::ofstream(actuator_path) << R"({
        "spec":{"joint_name":"joint","type":"position","limit":{"lower":-10,"upper":10}},
        "mjcf_binding":{},
        "pdu_config":{"pdu_name":"command","message_type":"std_msgs/Float64","update_rate_hz":50}
    })";
    const auto actuator = nlohmann::json::array({{
        {"id", "joint"}, {"kind", "actuator"}, {"type", "joint_position_actuator"},
        {"config", actuator_path.string()}, {"pdu_robot", "robot"},
    }});
    const auto entries = [&](const std::string& library) {
        using nlohmann::json;
        return json::array({
            {{"name", "any/route"}, {"type", "plugin"}, {"pdu_robot", "robot"},
                {"priority", 16}, {"plugin", {{"library", library}, {"config", {{"mode", "test"}}}}}},
            {{"name", "CarService/AckermannStep/car"}, {"type", "ackermann_step"}, {"pdu_robot", "car"}},
            {{"name", "PersonService/PersonStep/person"}, {"type", "person_step"}, {"pdu_robot", "person"}},
        });
    };
    RuntimeFactoryInput input {base.string(), "test.xml", pdu_path.string(), runtime_path.string(),
        actuator.dump(), entries(path.filename().string()).dump()};
    RuntimeDefinition definition;
    std::string error;
    assert(resolve_runtime_definition(input, definition, &error));
    assert(definition.services.size() == 3);
    const auto config = definition.services.front();
    assert(config.type == RuntimeServiceType::Plugin && config.priority == 16);
    assert(config.plugin_library == path && config.plugin_config_json == "{\"mode\":\"test\"}");

    input.services_json = entries(std::filesystem::path(wrong_kind).filename().string()).dump();
    assert(resolve_runtime_definition(input, definition, &error));
    expect_load_error(definition.services.front().plugin_library.c_str(), "needs a service plugin");
    auto invalid = entries(path.filename().string());
    invalid[0]["priority"] = 2147483648LL;
    input.services_json = invalid.dump();
    assert(!resolve_runtime_definition(input, definition, &error));
    assert(error.find("priority") != error.npos);
    return config;
}

} // namespace

int main(int argc, char** argv)
{
    // argv: the robot fixture plugin, a plugin of another kind
    assert(argc == 3);
    (void)parse_manifest(argv[1], argv[2]);
}
