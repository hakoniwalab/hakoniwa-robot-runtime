#include "runtime/runtime_factory_manifest.hpp"

#include <nlohmann/json.hpp>

#include <cassert>
#include <filesystem>
#include <fstream>
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

void test_mirror_configuration_contract()
{
    const fs::path root =
        fs::temp_directory_path() / "hakoniwa-robot-runtime-mirror-config-test";
    fs::remove_all(root);
    fs::create_directories(root);

    const auto types_path = root / "pdutypes.json";
    write_json(types_path, json::array({
        {{"channel_id", 0}, {"pdu_size", 32}, {"name", "joint_target"},
            {"type", "std_msgs/Float64"}},
        {{"channel_id", 1}, {"pdu_size", 72}, {"name", "pos"},
            {"type", "geometry_msgs/Twist"}},
        {{"channel_id", 2}, {"pdu_size", 216}, {"name", "impulse"},
            {"type", "hako_msgs/ImpulseCollision"}},
        {{"channel_id", 5}, {"pdu_size", 72}, {"name", "velocity"},
            {"type", "geometry_msgs/Twist"}},
    }));
    const auto pdu_def_path = root / "pdudef.json";
    write_json(pdu_def_path, {
        {"paths", json::array({{
            {"id", "drone-types"}, {"path", types_path.filename().string()}}})},
        {"robots", json::array({{
            {"name", "Drone-1"}, {"pdutypes_id", "drone-types"}}})},
    });

    const auto runtime_path = root / "runtime.json";
    write_json(runtime_path, {
        {"actuators", {{"joint1", {{"command_timeout_sec", 0.1}}}}},
        {"initial_body_poses", json::array({{
            {"mjcf_freejoint", "car_1_freejoint"},
            {"position_m", json::array({7.5, -45.0, 6.25})},
            {"orientation_rpy_rad", json::array({0.0, 0.0, 0.75})},
        }})},
    });
    const auto actuator_path = root / "actuator.json";
    write_json(actuator_path, {
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
    const auto mirror_path = root / "mirror.json";
    write_json(mirror_path, {
        {"spec", {{"mirror_id", "Drone-1"}}},
        {"input", {
            {"pose", {{"pdu_name", "pos"},
                {"message_type", "geometry_msgs/Twist"}}},
            {"velocity", {{"pdu_name", "velocity"},
                {"message_type", "geometry_msgs/Twist"},
                {"frame", "body"}}},
        }},
        {"mjcf_binding", {
            {"freejoint", "drone_1_freejoint"},
            {"contact_bodies", json::array({{
                {"body_id", "Car-1"},
                {"mjcf_freejoint", "car_1_freejoint"},
            }})},
        }},
    });
    const auto impulse_path = root / "impulse.json";
    write_json(impulse_path, {
        {"spec", {{"mirror_component", "drone-1-mirror"}}},
        {"pdu_config", {
            {"pdu_name", "impulse"},
            {"message_type", "hako_msgs/ImpulseCollision"},
        }},
        {"policy", {
            {"restitution_coefficient", 0.3},
            {"relative_normal_speed_threshold_mps", 0.2},
            {"cooldown_sec", 0.1},
        }},
    });

    const json components = json::array({
        {
            {"id", "joint1"}, {"kind", "actuator"},
            {"type", "joint_position_actuator"},
            {"config", actuator_path.string()}, {"pdu_robot", "Drone-1"},
        },
        {
            {"id", "drone-1-mirror"}, {"kind", "controller"},
            {"type", "mirror_body"},
            {"config", mirror_path.string()}, {"pdu_robot", "Drone-1"},
        },
        {
            {"id", "drone-1-impulse"}, {"kind", "state_output"},
            {"type", "impulse_collision"},
            {"config", impulse_path.string()}, {"pdu_robot", "Drone-1"},
        },
    });
    RuntimeFactoryInput input {
        root.string(),
        (root / "model.xml").string(),
        pdu_def_path.string(),
        runtime_path.string(),
        components.dump(),
    };
    RuntimeDefinition definition;
    std::string error;
    assert(resolve_runtime_definition(input, definition, &error));
    assert(error.empty());
    assert(definition.initial_body_poses.size() == 1);
    assert(definition.initial_body_poses[0].mjcf_freejoint
        == "car_1_freejoint");
    assert(definition.initial_body_poses[0].position.z == 6.25);
    assert(definition.initial_body_poses[0].rpy_rad[2] == 0.75);
    assert(definition.mirror_bodies.size() == 1);
    assert(definition.mirror_bodies[0].mirror_id == "Drone-1");
    assert(definition.mirror_bodies[0].velocity_pdu_name == "velocity");
    assert(definition.mirror_bodies[0].velocity_frame
        == MirrorVelocityFrame::Body);
    assert(definition.mirror_bodies[0].contact_bodies.size() == 1);
    assert(definition.impulse_collision_outputs.size() == 1);
    assert(definition.impulse_collision_outputs[0].mirror_id == "Drone-1");
    assert(definition.impulse_collision_outputs[0].cooldown_usec == 100'000);

    std::ifstream invalid_stream(impulse_path);
    assert(invalid_stream.is_open());
    json invalid_impulse;
    invalid_stream >> invalid_impulse;
    invalid_impulse["spec"]["mirror_component"] = "missing-mirror";
    write_json(impulse_path, invalid_impulse);
    RuntimeDefinition rejected;
    assert(!resolve_runtime_definition(input, rejected, &error));
    assert(error.find("unknown Mirror component") != std::string::npos);

    fs::remove_all(root);
}

} // namespace

int main()
{
    test_mirror_configuration_contract();
    return 0;
}
