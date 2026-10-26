#include "hakoniwa/robot_runtime/adapters/endpoint/person_step_rpc_event_reader.hpp"
#include "hakoniwa/robot_runtime/adapters/physics/mujoco/mujoco_actuator_plant.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace hakoniwa::robot_runtime;
namespace fs = std::filesystem;

namespace {

constexpr double pi = 3.14159265358979323846;

constexpr double radians(const double degrees)
{
    return degrees * pi / 180.0;
}

void require(
    const bool condition,
    const char* expression,
    const int line)
{
    if (!condition) {
        std::ostringstream message;
        message << "test requirement failed at line " << line
                << ": " << expression;
        throw std::runtime_error(message.str());
    }
}

#define REQUIRE(expression) require((expression), #expression, __LINE__)

void add_person(
    std::ostream& model,
    const std::string& name,
    const double y,
    const double hip_height)
{
    model << "<body name=\"" << name << "/person\" pos=\"0 " << y
          << " 0\" gravcomp=\"1\">"
          << "<joint name=\"" << name
          << "/slide_x_joint\" type=\"slide\" axis=\"1 0 0\"/>"
          << "<joint name=\"" << name
          << "/slide_y_joint\" type=\"slide\" axis=\"0 1 0\"/>"
          << "<joint name=\"" << name
          << "/slide_z_joint\" type=\"slide\" axis=\"0 0 1\"/>"
          << "<joint name=\"" << name
          << "/turn_joint\" type=\"hinge\" axis=\"0 0 1\"/>"
          << "<geom name=\"" << name
          << "/person_collision\" type=\"capsule\" fromto=\"0 0 0.1 0 0 1\" size=\"0.1\" mass=\"20\" friction=\"0.01 0.001 0.0001\"/>";
    for (const auto joint : adapters::hakoniwa::person_limb_joint_names) {
        const double body_z = joint == "hip_left_joint"
                || joint == "hip_right_joint"
            ? hip_height : 0.0;
        model << "<body pos=\"0 0 " << body_z << "\"><joint name=\""
              << name << '/' << joint
              << "\" type=\"hinge\" axis=\"0 1 0\"/>"
              << "<geom type=\"sphere\" size=\"0.01\" group=\"1\" contype=\"0\" conaffinity=\"0\" mass=\"0.001\"/></body>";
    }
    model << "</body>";
}

void add_actuators(std::ostream& model, const std::string& name)
{
    model << "<velocity name=\"" << name
          << "/move_x\" joint=\"" << name
          << "/slide_x_joint\" kv=\"1000\" ctrlrange=\"-3 3\"/>"
          << "<velocity name=\"" << name
          << "/move_y\" joint=\"" << name
          << "/slide_y_joint\" kv=\"1000\" ctrlrange=\"-3 3\"/>"
          << "<position name=\"" << name
          << "/lift\" joint=\"" << name
          << "/slide_z_joint\" kp=\"30000\" kv=\"2683\" ctrlrange=\"-10 10\"/>"
          << "<velocity name=\"" << name
          << "/turn\" joint=\"" << name
          << "/turn_joint\" kv=\"100\" ctrlrange=\"-3 3\"/>";
}

fs::path write_model()
{
    const auto nonce = std::chrono::high_resolution_clock::now()
        .time_since_epoch().count();
    const auto path = fs::temp_directory_path()
        / ("hakoniwa-person-rpc-test-" + std::to_string(nonce) + ".xml");
    std::ofstream model(path);
    REQUIRE(model.is_open());
    model << R"(<mujoco model="person-rpc-test">
  <option timestep="0.01" gravity="0 0 -9.81" integrator="implicitfast"/>
  <worldbody>
    <geom name="ground" type="plane" size="20 20 0.1" friction="0.01 0.001 0.0001"/>)";
    add_person(model, "A", 0.0, 0.66);
    add_person(model, "B", 2.0, 0.462);
    model << "<body name=\"Car-1/body\" pos=\"0 0 0.2\" gravcomp=\"1\">"
          << "<freejoint name=\"Car-1/freejoint\"/>"
          << "<geom type=\"box\" size=\"0.5 0.3 0.2\" mass=\"100\" "
             "contype=\"0\" conaffinity=\"0\"/></body>";
    model << "<geom name=\"fixed_box\" type=\"box\" pos=\"4 0 0.5\" size=\"0.2 0.5 0.5\"/>";
    model << "<geom name=\"dismount_box\" type=\"box\" pos=\"4 1.15 0.5\" size=\"0.2 0.5 0.5\"/>";
    model << R"(</worldbody><actuator>)";
    add_actuators(model, "A");
    add_actuators(model, "B");
    model << "</actuator></mujoco>";
    return path;
}

runtime::RuntimeDefinition definition_with_vehicle(const fs::path& model)
{
    runtime::RuntimeDefinition definition;
    definition.model_path = model.string();
    runtime::RuntimePersonVehicleConfig vehicle;
    vehicle.name = "Car-1";
    vehicle.mjcf_freejoint = "Car-1/freejoint";
    vehicle.seats.push_back({"driver", {0.3, 0.4, 0.5}});
    definition.person_vehicles.push_back(std::move(vehicle));
    return definition;
}

runtime::Vector3State rotate_vector(
    const runtime::QuaternionState& q,
    const runtime::Vector3State& vector)
{
    const double x = q.x;
    const double y = q.y;
    const double z = q.z;
    const double w = q.w;
    return {
        (1.0 - 2.0 * (y * y + z * z)) * vector.x
            + 2.0 * (x * y - z * w) * vector.y
            + 2.0 * (x * z + y * w) * vector.z,
        2.0 * (x * y + z * w) * vector.x
            + (1.0 - 2.0 * (x * x + z * z)) * vector.y
            + 2.0 * (y * z - x * w) * vector.z,
        2.0 * (x * z - y * w) * vector.x
            + 2.0 * (y * z + x * w) * vector.y
            + (1.0 - 2.0 * (x * x + y * y)) * vector.z,
    };
}

runtime::Vector3State visual_hip(
    const adapters::hakoniwa::PersonStepState& state,
    const double hip_height)
{
    const auto offset = rotate_vector(
        state.orientation, {0.0, 0.0, hip_height});
    return {
        state.position.x + offset.x,
        state.position.y + offset.y,
        state.position.z + offset.z,
    };
}

double quaternion_yaw(const runtime::QuaternionState& q)
{
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double quaternion_pitch(const runtime::QuaternionState& q)
{
    return std::asin(std::clamp(
        2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
}

double quaternion_roll(const runtime::QuaternionState& q)
{
    return std::atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y));
}

class TemporaryModel final {
public:
    TemporaryModel() : path(write_model()) {}
    ~TemporaryModel() { std::error_code ignored; fs::remove(path, ignored); }

    TemporaryModel(const TemporaryModel&) = delete;
    TemporaryModel& operator=(const TemporaryModel&) = delete;

    fs::path path;
};

adapters::hakoniwa::PersonStepCommand command(
    double forward, double left, double yaw_rate,
    std::string animation = "auto", std::string ride = {})
{
    return {
        {forward, left, 0.0},
        {0.0, 0.0, yaw_rate},
        std::move(animation),
        std::move(ride),
    };
}

void test_codec()
{
    REQUIRE(adapters::hakoniwa::person_step_service_name("Alice")
        == "PersonService/PersonStep/Alice");
    auto expected = command(1.0, -0.25, 0.5, "walk");
    adapters::hakoniwa::PersonStepCommand decoded;
    REQUIRE(adapters::hakoniwa::decode_person_command(
        adapters::hakoniwa::encode_person_command(expected), decoded));
    REQUIRE(decoded.linear_velocity.x == 1.0);
    REQUIRE(decoded.linear_velocity.y == -0.25);
    REQUIRE(decoded.angular_velocity.z == 0.5);
    REQUIRE(decoded.animation == "walk");

    adapters::hakoniwa::PersonStepState state;
    state.result_code = 7;
    state.simulation_time_usec = 1234;
    state.position = {1.0, 2.0, 3.0};
    state.joint_positions[2] = 0.4;
    adapters::hakoniwa::PersonStepState state_decoded;
    REQUIRE(adapters::hakoniwa::decode_person_state(
        adapters::hakoniwa::encode_person_state(state), state_decoded));
    REQUIRE(state_decoded.result_code == 7);
    REQUIRE(state_decoded.simulation_time_usec == 1234);
    REQUIRE(state_decoded.position.y == 2.0);
    REQUIRE(state_decoded.joint_positions[2] == 0.4);
}

void test_motion_and_gait()
{
    const TemporaryModel temporary_model;
    runtime::RuntimeDefinition definition;
    definition.model_path = temporary_model.path.string();
    adapters::mujoco::MujocoActuatorPlant plant(definition);
    plant.bind_person("A");
    plant.bind_person("B");

    REQUIRE(plant.apply_person_command("A", command(1.0, 0.0, 0.0)) == 0);
    REQUIRE(plant.apply_person_command("B", command(0.0, 0.0, 0.5)) == 0);
    for (int step = 0; step < 100; ++step) (void)plant.step({});
    const auto a = plant.person_state("A");
    const auto b = plant.person_state("B");
    std::cerr << "PERSON_RPC_DIAGNOSTIC first_second"
              << " a.position=(" << a.position.x << ',' << a.position.y
              << ',' << a.position.z << ')'
              << " a.velocity=(" << a.linear_velocity.x << ','
              << a.linear_velocity.y << ')'
              << " b.yaw_quaternion_z=" << b.orientation.z << '\n';
    REQUIRE(a.position.x > 0.95 && a.position.x < 1.02);
    REQUIRE(std::fabs(a.position.y) < 0.05);
    REQUIRE(b.orientation.z > std::sin(0.4 / 2.0));
    REQUIRE(b.orientation.z < std::sin(0.6 / 2.0));
    REQUIRE(std::fabs(a.joint_positions[2]) > 1.0e-3
        || std::fabs(a.joint_positions[3]) > 1.0e-3);
    REQUIRE(std::fabs(b.joint_positions[2]) < 1.0e-9);
    REQUIRE(std::fabs(b.joint_positions[3]) < 1.0e-9);

    // Body-frame forward at +90 degrees moves along world +Y.
    (void)plant.reset();
    const int turn = mj_name2id(
        plant.model(), mjOBJ_JOINT, "A/turn_joint");
    REQUIRE(turn >= 0);
    plant.data()->qpos[plant.model()->jnt_qposadr[turn]] = pi / 2.0;
    mj_forward(plant.model(), plant.data());
    REQUIRE(plant.apply_person_command("A", command(1.0, 0.0, 0.0)) == 0);
    REQUIRE(plant.apply_person_command("B", command(0.0, 0.0, 0.0)) == 0);
    for (int step = 0; step < 100; ++step) (void)plant.step({});
    const auto rotated = plant.person_state("A");
    REQUIRE(rotated.position.y > 0.95);
    REQUIRE(std::fabs(rotated.position.x) < 0.1);

    // Stage 1 rejects ride without replacing the active walking command.
    REQUIRE(plant.apply_person_command(
        "A", command(0.0, 0.0, 0.0, "idle", "Car-1/driver")) != 0);
    const auto before = plant.person_state("A").position.y;
    for (int step = 0; step < 20; ++step) (void)plant.step({});
    REQUIRE(plant.person_state("A").position.y > before + 0.1);

    // The velocity target remains active, but contact prevents tunnelling
    // through the fixed obstacle (stage-1 collision option a).
    (void)plant.reset();
    REQUIRE(plant.apply_person_command("A", command(1.0, 0.0, 0.0)) == 0);
    for (int step = 0; step < 600; ++step) (void)plant.step({});
    const auto blocked = plant.person_state("A");
    REQUIRE(blocked.position.x > 3.0);
    REQUIRE(blocked.position.x < 3.75);

}

void test_gestures_and_sitting()
{
    const TemporaryModel temporary_model;
    runtime::RuntimeDefinition definition;
    definition.model_path = temporary_model.path.string();
    adapters::mujoco::MujocoActuatorPlant plant(definition);
    plant.bind_person("A");
    plant.bind_person("B");

    const auto steps = [&](const int count) {
        for (int step = 0; step < count; ++step) (void)plant.step({});
    };
    const auto reset = [&] {
        (void)plant.reset();
    };
    const auto animation = [&](const std::string& name,
                               const double forward = 0.0) {
        REQUIRE(plant.apply_person_command(
            "A", command(forward, 0.0, 0.0, name)) == 0);
    };

    // Every stage-2 animation is accepted. Unknown names retain the
    // people_plant.py behavior and become auto, rather than idle.
    for (const auto* name : {
             "auto", "walk", "idle", "wave", "talk", "sit", "sit_talk",
             "sit_drive", "sit_drowsy", "sit_alert", "sit_startled"}) {
        animation(name);
    }
    animation("unknown-animation", 1.0);
    steps(10);
    const auto automatic = plant.person_state("A");
    REQUIRE(std::fabs(automatic.joint_positions[2]) > 1.0e-3
        || std::fabs(automatic.joint_positions[3]) > 1.0e-3);

    reset();
    animation("wave");
    steps(200);
    const auto wave = plant.person_state("A");
    REQUIRE(wave.joint_positions[1] >= radians(-168.0));
    REQUIRE(wave.joint_positions[1] <= radians(-132.0));

    reset();
    animation("idle");
    steps(1);
    const double hip_before = plant.person_state("A").joint_positions[2];
    animation("sit");
    steps(1);
    const double hip_after = plant.person_state("A").joint_positions[2];
    REQUIRE(std::fabs(hip_after - hip_before) <= radians(2.4) + 1.0e-9);

    steps(49);
    const auto sit = plant.person_state("A");
    REQUIRE(std::fabs(sit.joint_positions[0] - radians(-25.0)) < radians(1.0));
    REQUIRE(std::fabs(sit.joint_positions[1] - radians(-25.0)) < radians(1.0));
    REQUIRE(std::fabs(sit.joint_positions[2] - radians(-90.0)) < radians(1.0));
    REQUIRE(std::fabs(sit.joint_positions[3] - radians(-90.0)) < radians(1.0));
    REQUIRE(std::fabs(sit.joint_positions[4] - radians(90.0)) < radians(1.0));
    REQUIRE(std::fabs(sit.joint_positions[5] - radians(90.0)) < radians(1.0));

    // The limbs reach their target in 0.5 s; allow the damped lift actuator
    // another 0.5 s to settle at the seat height.
    steps(50);

    const int slide_z = mj_name2id(
        plant.model(), mjOBJ_JOINT, "A/slide_z_joint");
    const int collision = mj_name2id(
        plant.model(), mjOBJ_GEOM, "A/person_collision");
    REQUIRE(slide_z >= 0);
    REQUIRE(collision >= 0);
    const double hip_height = plant.data()->qpos[
        plant.model()->jnt_qposadr[slide_z]] + 0.66;
    std::cerr << "PERSON_RPC_DIAGNOSTIC sit hip_height=" << hip_height
              << " root_z=" << plant.data()->qpos[
                     plant.model()->jnt_qposadr[slide_z]] << '\n';
    REQUIRE(std::fabs(hip_height - 0.44) < 0.02);
    REQUIRE(plant.model()->geom_contype[collision] == 0);
    REQUIRE(plant.model()->geom_conaffinity[collision] == 0);

    // A seated non-rider is out of collision and crosses the fixed box.
    animation("sit", 1.0);
    steps(600);
    REQUIRE(plant.person_state("A").position.x > 4.5);

    // Standing restores both lift and the model's original collision masks.
    animation("walk");
    steps(50);
    REQUIRE(std::fabs(plant.data()->qpos[
        plant.model()->jnt_qposadr[slide_z]]) < 0.02);
    REQUIRE(plant.model()->geom_contype[collision] == 1);
    REQUIRE(plant.model()->geom_conaffinity[collision] == 1);

    reset();
    animation("sit_drive");
    steps(50);
    const auto drive = plant.person_state("A");
    REQUIRE(std::fabs(drive.joint_positions[0] - radians(-65.0)) < radians(1.0));
    REQUIRE(std::fabs(drive.joint_positions[1] - radians(-65.0)) < radians(1.0));

    reset();
    animation("sit_alert");
    steps(50);
    const auto alert = plant.person_state("A");
    REQUIRE(std::fabs(alert.joint_positions[0] - radians(-65.0)) < radians(1.0));
    REQUIRE(std::fabs(alert.joint_positions[1] - radians(-65.0)) < radians(1.0));

    reset();
    animation("sit_startled");
    steps(50);
    const auto startled = plant.person_state("A");
    REQUIRE(std::fabs(startled.joint_positions[0] - radians(-85.0))
        < radians(1.0));
    REQUIRE(std::fabs(startled.joint_positions[1] - radians(-85.0))
        < radians(1.0));

    reset();
    animation("sit_drowsy");
    steps(200);
    const auto drowsy = plant.person_state("A");
    REQUIRE(drowsy.joint_positions[0] >= radians(-19.0) - 1.0e-9);
    REQUIRE(drowsy.joint_positions[0] <= radians(-11.0) + 1.0e-9);
    REQUIRE(drowsy.joint_positions[1] >= radians(-19.0) - 1.0e-9);
    REQUIRE(drowsy.joint_positions[1] <= radians(-11.0) + 1.0e-9);

    reset();
    animation("talk");
    steps(200);
    const auto talk = plant.person_state("A");
    REQUIRE(talk.joint_positions[1] >= radians(-49.0) - 1.0e-9);
    REQUIRE(talk.joint_positions[1] <= radians(-21.0) + 1.0e-9);
    REQUIRE(talk.joint_positions[0] >= radians(-29.0) - 1.0e-9);
    REQUIRE(talk.joint_positions[0] <= radians(-11.0) + 1.0e-9);

    reset();
    animation("sit_talk");
    steps(200);
    const auto sit_talk = plant.person_state("A");
    REQUIRE(std::fabs(sit_talk.joint_positions[2] - radians(-90.0))
        < radians(1.0));
    REQUIRE(std::fabs(sit_talk.joint_positions[4] - radians(90.0))
        < radians(1.0));
    REQUIRE(sit_talk.joint_positions[1] >= radians(-49.0) - 1.0e-9);
    REQUIRE(sit_talk.joint_positions[1] <= radians(-21.0) + 1.0e-9);

    // A scaled person's model-space hip height replaces HIP_M * scale.
    reset();
    REQUIRE(plant.apply_person_command(
        "B", command(0.0, 0.0, 0.0, "sit")) == 0);
    steps(100);
    const int child_slide_z = mj_name2id(
        plant.model(), mjOBJ_JOINT, "B/slide_z_joint");
    REQUIRE(child_slide_z >= 0);
    const double child_hip_height = plant.data()->qpos[
        plant.model()->jnt_qposadr[child_slide_z]] + 0.462;
    REQUIRE(std::fabs(child_hip_height - 0.44) < 0.02);
}

void test_vehicle_ride_and_posture()
{
    const TemporaryModel temporary_model;
    adapters::mujoco::MujocoActuatorPlant plant(
        definition_with_vehicle(temporary_model.path));
    plant.bind_person("A");
    const int vehicle_joint = mj_name2id(
        plant.model(), mjOBJ_JOINT, "Car-1/freejoint");
    const int collision = mj_name2id(
        plant.model(), mjOBJ_GEOM, "A/person_collision");
    REQUIRE(vehicle_joint >= 0);
    REQUIRE(collision >= 0);
    const int vehicle_qpos = plant.model()->jnt_qposadr[vehicle_joint];
    const int vehicle_qvel = plant.model()->jnt_dofadr[vehicle_joint];

    // Omitting the seat name selects driver. The response's visual hip is
    // exactly on the body-frame seat after the same physics step.
    REQUIRE(plant.apply_person_command(
        "A", command(0.0, 0.0, 0.0, "sit", "Car-1")) == 0);
    (void)plant.step({});
    auto state = plant.person_state("A");
    auto hip = visual_hip(state, 0.66);
    const int vehicle_body = plant.model()->jnt_bodyid[vehicle_joint];
    REQUIRE(std::fabs(hip.x - (plant.data()->xpos[3 * vehicle_body] + 0.3)) < 1.0e-6);
    REQUIRE(std::fabs(hip.y - (plant.data()->xpos[3 * vehicle_body + 1] + 0.4)) < 1.0e-6);
    REQUIRE(std::fabs(hip.z - (plant.data()->xpos[3 * vehicle_body + 2] + 0.5)) < 1.0e-6);
    REQUIRE(std::fabs(quaternion_yaw(state.orientation)) < 1.0e-6);
    REQUIRE(plant.model()->geom_contype[collision] == 0);
    REQUIRE(plant.model()->geom_conaffinity[collision] == 0);

    // A 5 m/s vehicle is followed without prediction or a one-step lag.
    for (int step = 0; step < 100; ++step) {
        plant.data()->qvel[vehicle_qvel] = 5.0;
        (void)plant.step({});
        state = plant.person_state("A");
        hip = visual_hip(state, 0.66);
        REQUIRE(std::fabs(
            hip.x - (plant.data()->xpos[3 * vehicle_body] + 0.3)) < 0.001);
        REQUIRE(std::fabs(
            hip.y - (plant.data()->xpos[3 * vehicle_body + 1] + 0.4)) < 0.001);
        REQUIRE(std::fabs(
            hip.z - (plant.data()->xpos[3 * vehicle_body + 2] + 0.5)) < 0.001);
    }

    // Rotating the vehicle rotates both its seat offset and the rider.
    plant.data()->qvel[vehicle_qvel] = 0.0;
    plant.data()->qpos[vehicle_qpos + 3] = std::cos(pi / 4.0);
    plant.data()->qpos[vehicle_qpos + 4] = 0.0;
    plant.data()->qpos[vehicle_qpos + 5] = 0.0;
    plant.data()->qpos[vehicle_qpos + 6] = std::sin(pi / 4.0);
    mj_forward(plant.model(), plant.data());
    (void)plant.step({});
    state = plant.person_state("A");
    hip = visual_hip(state, 0.66);
    REQUIRE(std::fabs(
        hip.x - (plant.data()->xpos[3 * vehicle_body] - 0.4)) < 0.001);
    REQUIRE(std::fabs(
        hip.y - (plant.data()->xpos[3 * vehicle_body + 1] + 0.3)) < 0.001);
    REQUIRE(std::fabs(quaternion_yaw(state.orientation) - pi / 2.0) < 0.001);

    // Whole-body posture is visual-only and approaches its target at 90 deg/s.
    REQUIRE(plant.apply_person_command(
        "A", command(0.0, 0.0, 0.0, "sit_drowsy", "Car-1/driver")) == 0);
    (void)plant.step({});
    state = plant.person_state("A");
    REQUIRE(std::fabs(quaternion_pitch(state.orientation)) <= radians(0.9) + 1.0e-6);
    for (int step = 0; step < 199; ++step) (void)plant.step({});
    state = plant.person_state("A");
    REQUIRE(quaternion_pitch(state.orientation) >= radians(16.0) - 1.0e-6);
    REQUIRE(quaternion_pitch(state.orientation) <= radians(28.0) + 1.0e-6);
    REQUIRE(std::fabs(quaternion_roll(state.orientation)) <= radians(5.0) + 1.0e-6);

    // Empty ride dismounts on the seat's outside at 0.75 + abs(0.4),
    // restores ground collision, and can then be stopped by a box.
    plant.data()->qpos[vehicle_qpos + 3] = 1.0;
    plant.data()->qpos[vehicle_qpos + 6] = 0.0;
    plant.data()->qpos[vehicle_qpos] = 0.0;
    plant.data()->qpos[vehicle_qpos + 1] = 0.0;
    mj_forward(plant.model(), plant.data());
    REQUIRE(plant.apply_person_command(
        "A", command(0.0, 0.0, 0.0, "idle", "")) == 0);
    (void)plant.step({});
    state = plant.person_state("A");
    REQUIRE(std::fabs(state.position.x - 0.3) < 0.001);
    REQUIRE(std::fabs(state.position.y - 1.15) < 0.001);
    REQUIRE(std::fabs(state.position.z) < 0.02);
    REQUIRE(plant.model()->geom_contype[collision] == 1);
    REQUIRE(plant.model()->geom_conaffinity[collision] == 1);
    REQUIRE(plant.apply_person_command(
        "A", command(1.0, 0.0, 0.0, "walk")) == 0);
    for (int step = 0; step < 600; ++step) (void)plant.step({});
    REQUIRE(plant.person_state("A").position.x < 3.75);

    // Unknown vehicle/seat requests do not replace the accepted standing state.
    (void)plant.reset();
    const auto standing = plant.person_state("A");
    REQUIRE(plant.apply_person_command(
        "A", command(0.0, 0.0, 0.0, "sit", "Missing/driver")) != 0);
    REQUIRE(plant.apply_person_command(
        "A", command(0.0, 0.0, 0.0, "sit", "Car-1/missing")) != 0);
    (void)plant.step({});
    const auto rejected = plant.person_state("A");
    REQUIRE(std::fabs(rejected.position.x - standing.position.x) < 0.001);
    REQUIRE(std::fabs(rejected.position.y - standing.position.y) < 0.001);
    REQUIRE(plant.model()->geom_contype[collision] == 1);
}

} // namespace

int main()
{
    test_codec();
    test_motion_and_gait();
    test_gestures_and_sitting();
    test_vehicle_ride_and_posture();
    return 0;
}
