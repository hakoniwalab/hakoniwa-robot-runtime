#include "hakoniwa/robot_runtime/adapters/endpoint/person_step_rpc_event_reader.hpp"
#include "hakoniwa/robot_runtime/adapters/physics/mujoco/mujoco_actuator_plant.hpp"

#include <mujoco/mujoco.h>

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

void add_person(std::ostream& model, const std::string& name, double y)
{
    model << "<body name=\"" << name << "/person\" pos=\"0 " << y
          << " 0\">"
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
        model << "<body><joint name=\"" << name << '/' << joint
              << "\" type=\"hinge\" axis=\"0 1 0\"/>"
              << "<geom type=\"sphere\" size=\"0.01\" contype=\"0\" conaffinity=\"0\" mass=\"0.001\"/></body>";
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
          << "/slide_z_joint\" kp=\"1000\" ctrlrange=\"-10 10\"/>"
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
    add_person(model, "A", 0.0);
    add_person(model, "B", 2.0);
    model << "<geom name=\"fixed_box\" type=\"box\" pos=\"4 0 0.5\" size=\"0.2 0.5 0.5\"/>";
    model << R"(</worldbody><actuator>)";
    add_actuators(model, "A");
    add_actuators(model, "B");
    model << "</actuator></mujoco>";
    return path;
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

} // namespace

int main()
{
    test_codec();
    test_motion_and_gait();
    return 0;
}
