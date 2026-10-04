#include "runtime/runtime_factory_manifest.hpp"

#include <nlohmann/json.hpp>

#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

using namespace hakoniwa::robot_runtime::runtime;
namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {

void write_json(const fs::path& path, const json& value)
{
    std::ofstream stream(path);
    assert(stream.is_open());
    stream << value.dump(2) << '\n';
}

class Fixture {
public:
    Fixture()
        : root_(fs::temp_directory_path()
            / "hakoniwa-robot-runtime-geom-friction-config-test")
    {
        fs::remove_all(root_);
        fs::create_directories(root_);
        write_json(root_ / "pdutypes.json", json::array({
            {{"channel_id", 0}, {"pdu_size", 32}, {"name", "tire_friction"},
                {"type", "std_msgs/Float64"}},
            {{"channel_id", 1}, {"pdu_size", 72}, {"name", "pos"},
                {"type", "geometry_msgs/Twist"}},
            {{"channel_id", 2}, {"pdu_size", 32}, {"name", "joint_target"},
                {"type", "std_msgs/Float64"}},
        }));
        write_json(root_ / "pdudef.json", {
            {"paths", json::array({{
                {"id", "car-types"}, {"path", "pdutypes.json"}}})},
            {"robots", json::array({{
                {"name", "Car-3"}, {"pdutypes_id", "car-types"}}})},
        });
        // A Runtime manifest needs at least one scalar actuator.
        write_json(root_ / "runtime.json", {
            {"actuators", {{"joint1", {{"command_timeout_sec", 0.1}}}}},
        });
        write_json(root_ / "actuator.json", {
            {"spec", {
                {"joint_name", "joint1"},
                {"type", "position"},
                {"limit", {{"lower", -1.0}, {"upper", 1.0}}},
            }},
            {"mjcf_binding", {{"actuator_name", "joint1_actuator"}}},
            {"pdu_config", {
                {"pdu_name", "joint_target"},
                {"message_type", "std_msgs/Float64"},
                {"update_rate_hz", 100.0},
            }},
        });
    }

    ~Fixture()
    {
        fs::remove_all(root_);
    }

    [[nodiscard]] json valid_config() const
    {
        return {
            {"schema_version", 1},
            {"spec", {
                {"geoms", json::array({
                    "car_3_front_left_tire", "car_3_front_right_tire"})},
            }},
            {"input", {
                {"pdu_name", "tire_friction"},
                {"message_type", "std_msgs/Float64"},
            }},
        };
    }

    /** Resolves a manifest whose components use the given friction configs. */
    bool resolve(
        const json& first,
        RuntimeDefinition& definition,
        std::string& error,
        const json& second = nullptr) const
    {
        json components = json::array({{
            {"id", "joint1"}, {"kind", "actuator"},
            {"type", "joint_position_actuator"},
            {"config", "actuator.json"}, {"pdu_robot", "Car-3"},
        }});
        write_json(root_ / "friction-1.json", first);
        components.push_back({
            {"id", "car-3-tire-friction"}, {"kind", "controller"},
            {"type", "geom_friction"},
            {"config", "friction-1.json"}, {"pdu_robot", "Car-3"},
        });
        if (!second.is_null()) {
            write_json(root_ / "friction-2.json", second);
            components.push_back({
                {"id", "car-3-tire-friction-2"}, {"kind", "controller"},
                {"type", "geom_friction"},
                {"config", "friction-2.json"}, {"pdu_robot", "Car-3"},
            });
        }
        const RuntimeFactoryInput input {
            root_.string(),
            (root_ / "model.xml").string(),
            (root_ / "pdudef.json").string(),
            (root_ / "runtime.json").string(),
            components.dump(),
        };
        return resolve_runtime_definition(input, definition, &error);
    }

private:
    fs::path root_;
};

void test_valid_config()
{
    Fixture fixture;
    RuntimeDefinition definition;
    std::string error;
    const bool resolved = fixture.resolve(fixture.valid_config(), definition, error);
    if (!resolved) {
        std::cerr << "unexpected rejection: " << error << '\n';
    }
    assert(resolved);
    assert(error.empty());
    assert(definition.geom_frictions.size() == 1);
    const auto& friction = definition.geom_frictions[0];
    assert(friction.component_id == "car-3-tire-friction");
    assert(friction.pdu_robot == "Car-3");
    assert(friction.pdu_name == "tire_friction");
    assert(friction.geoms.size() == 2);
    assert(friction.geoms[0] == "car_3_front_left_tire");
    // The directive component does not add an arbitrated actuator.
    assert(definition.actuators.size() == 1);
}

void test_unknown_keys_are_ignored()
{
    // Like the other component parsers, unknown keys are not interpreted.
    // Geom priority belongs to the model, so a stale "priority" is ignored.
    Fixture fixture;
    auto config = fixture.valid_config();
    config["spec"]["priority"] = 5;
    RuntimeDefinition definition;
    std::string error;
    assert(fixture.resolve(config, definition, error));
    assert(definition.geom_frictions.size() == 1);
    assert(definition.geom_frictions[0].geoms.size() == 2);
}

void expect_rejected(
    const Fixture& fixture,
    const json& config,
    const char* expected_error,
    const json& second = nullptr)
{
    RuntimeDefinition definition;
    std::string error;
    assert(!fixture.resolve(config, definition, error, second));
    assert(error.find(expected_error) != std::string::npos);
}

void test_invalid_values_are_rejected()
{
    Fixture fixture;

    auto empty_geoms = fixture.valid_config();
    empty_geoms["spec"]["geoms"] = json::array();
    expect_rejected(fixture, empty_geoms, "non-empty array");

    auto non_string_geom = fixture.valid_config();
    non_string_geom["spec"]["geoms"] = json::array({"tire", 3});
    expect_rejected(fixture, non_string_geom, "non-empty strings");

    auto empty_name = fixture.valid_config();
    empty_name["spec"]["geoms"] = json::array({""});
    expect_rejected(fixture, empty_name, "non-empty strings");

    auto wrong_message = fixture.valid_config();
    wrong_message["input"]["message_type"] = "geometry_msgs/Twist";
    expect_rejected(fixture, wrong_message, "std_msgs/Float64");

    auto wrong_channel_type = fixture.valid_config();
    wrong_channel_type["input"]["pdu_name"] = "pos";
    RuntimeDefinition definition;
    std::string error;
    assert(!fixture.resolve(wrong_channel_type, definition, error));

    auto unknown_pdu = fixture.valid_config();
    unknown_pdu["input"]["pdu_name"] = "missing";
    assert(!fixture.resolve(unknown_pdu, definition, error));

    auto bad_schema = fixture.valid_config();
    bad_schema["schema_version"] = 2;
    expect_rejected(fixture, bad_schema, "schema_version");

    auto missing_input = fixture.valid_config();
    missing_input.erase("input");
    expect_rejected(fixture, missing_input, "invalid geom friction config");

    // One geom cannot be driven by two friction streams.
    auto overlapping = fixture.valid_config();
    overlapping["spec"]["geoms"] = json::array({"car_3_front_left_tire"});
    expect_rejected(fixture, fixture.valid_config(),
        "more than one geom friction entry", overlapping);
}

} // namespace

int main()
{
    test_valid_config();
    test_unknown_keys_are_ignored();
    test_invalid_values_are_rejected();
    return 0;
}
