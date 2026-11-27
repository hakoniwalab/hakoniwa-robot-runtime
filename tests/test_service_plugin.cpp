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
#include <iostream>
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
    // Stage 2c-5: the manifest lists robots; each robot names its plugin (shared library or
    // builtin) and its services in order. The service index is the position in that list.
    const auto robots = [&](const std::string& library) {
        using nlohmann::json;
        return json::array({
            {{"name", "robot"}, {"priority", 16},
                {"plugin", {{"library", library}, {"config", {{"mode", "test"}}}}},
                {"services", json::array({
                    {{"name", "any/route"}},
                    {{"name", "any/stop"}, {"lockstep", false}, {"type", "t/Stop"}},
                })}},
            {{"name", "car"}, {"builtin", "ackermann_step"},
                {"services", json::array({{{"name", "CarService/AckermannStep/car"}, {"lockstep", true}}})}},
            {{"name", "person"}, {"builtin", "person_step"},
                {"services", json::array({{{"name", "PersonService/PersonStep/person"}}})}},
        });
    };
    RuntimeFactoryInput input {base.string(), "test.xml", pdu_path.string(), runtime_path.string(),
        actuator.dump(), robots(path.filename().string()).dump()};
    RuntimeDefinition definition;
    std::string error;
    assert(resolve_runtime_definition(input, definition, &error));
    // The robots become the same internal service rows as before, in manifest order.
    assert(definition.services.size() == 4);
    const auto config = definition.services.front();
    assert(config.name == "any/route" && config.pdu_robot == "robot");
    assert(config.type == RuntimeServiceType::Plugin && config.priority == 16);
    assert(config.plugin_library == path && config.plugin_config_json == "{\"mode\":\"test\"}");
    assert(definition.services[1].name == "any/stop" && definition.services[1].pdu_robot == "robot");
    assert(definition.services[1].type == RuntimeServiceType::Plugin && definition.services[1].priority == 16);
    assert(definition.services[2].type == RuntimeServiceType::AckermannStep);
    assert(definition.services[2].pdu_robot == "car" && definition.services[2].lockstep);
    assert(definition.services[3].type == RuntimeServiceType::PersonStep);
    assert(definition.services[3].pdu_robot == "person" && !definition.services[3].lockstep);

    input.robots_json = robots(std::filesystem::path(wrong_kind).filename().string()).dump();
    assert(resolve_runtime_definition(input, definition, &error));
    expect_load_error(definition.services.front().plugin_library.c_str(), "needs a service plugin");

    const auto expect_refused = [&](nlohmann::json manifest_robots, std::string_view part) {
        input.robots_json = manifest_robots.dump();
        assert(!resolve_runtime_definition(input, definition, &error));
        if (error.find(part) == std::string::npos) {
            std::cerr << "unexpected error: " << error << " (wanted: " << part << ")\n";
            assert(false);
        }
    };
    auto invalid = robots(path.filename().string());
    invalid[0]["priority"] = 2147483648LL;
    expect_refused(invalid, "priority");
    invalid = robots(path.filename().string());
    invalid[1]["builtin"] = "teleporter";
    expect_refused(invalid, "teleporter");
    invalid = robots(path.filename().string());
    invalid[1]["plugin"] = {{"library", path.filename().string()}};
    expect_refused(invalid, "builtin");  // a robot names a builtin or a plugin, not both
    invalid = robots(path.filename().string());
    invalid[2]["services"][0]["name"] = "any/route";
    expect_refused(invalid, "any/route");  // service names are unique
    invalid = robots(path.filename().string());
    invalid[0]["name"] = "car";
    expect_refused(invalid, "car");  // robot names are unique
    invalid = robots(path.filename().string());
    invalid[0]["services"][0]["type"] = "plugin";
    expect_refused(invalid, "type");  // only a package/Service hint is accepted
    return config;
}

} // namespace

int main(int argc, char** argv)
{
    // argv: the robot fixture plugin, a plugin of another kind
    assert(argc == 3);
    (void)parse_manifest(argv[1], argv[2]);
}
