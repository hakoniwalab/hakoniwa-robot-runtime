#include "hakoniwa/robot_runtime/adapters/physics/mujoco/mujoco_actuator_plant.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

using namespace hakoniwa::robot_runtime;
namespace fs = std::filesystem;

namespace {

constexpr double epsilon = 1.0e-9;

void test_initial_body_pose_survives_reset()
{
    const fs::path model_path =
        fs::temp_directory_path() / "hakoniwa-initial-body-pose-test.xml";
    {
        std::ofstream model(model_path);
        assert(model.is_open());
        model << R"(<mujoco model="initial-body-pose-test">
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body name="car" pos="0 0 0.42">
      <freejoint name="car_freejoint"/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>)";
    }

    runtime::RuntimeDefinition definition;
    definition.model_path = model_path.string();
    definition.initial_body_poses.push_back({
        "car_freejoint", {7.5, -45.0, 6.253847}, {0.0, 0.0, 0.75},
    });
    definition.multi_dof_state_outputs.push_back({
        .component_id = "vehicle_states",
        .bodies = {{"Car-1", "Car-1", "car_freejoint"}},
    });

    adapters::mujoco::MujocoActuatorPlant plant(definition);
    const auto assert_pose = [](const runtime::RobotState& state) {
        assert(state.bodies.size() == 1);
        const auto& body = state.bodies.front();
        assert(std::abs(body.position.x - 7.5) < epsilon);
        assert(std::abs(body.position.y + 45.0) < epsilon);
        assert(std::abs(body.position.z - 6.253847) < epsilon);
        assert(std::abs(body.orientation.z - std::sin(0.75 / 2.0)) < epsilon);
        assert(std::abs(body.orientation.w - std::cos(0.75 / 2.0)) < epsilon);
    };
    assert_pose(plant.read_state());
    assert_pose(plant.reset());
    fs::remove(model_path);
}

void test_mirror_without_input_body_motion_and_contact_extraction()
{
    const fs::path model_path =
        fs::temp_directory_path() / "hakoniwa-mirror-plant-test.xml";
    {
        std::ofstream model(model_path);
        assert(model.is_open());
        model << R"(<mujoco model="mirror-contact-test">
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <geom name="environment" type="plane" size="10 10 0.1"/>
    <body name="mirror" pos="-2 0 1">
      <freejoint name="mirror_freejoint"/>
      <geom name="mirror_geom" type="sphere" size="0.5" mass="1"/>
    </body>
    <body name="car" pos="0.8 0 1">
      <freejoint name="car_freejoint"/>
      <geom name="car_geom" type="sphere" size="0.5" mass="10"/>
    </body>
  </worldbody>
</mujoco>)";
    }

    runtime::RuntimeDefinition definition;
    definition.model_path = model_path.string();
    runtime::RuntimeMirrorBodyConfig mirror;
    mirror.component_id = "drone-1-mirror";
    mirror.mirror_id = "Drone-1";
    mirror.mjcf_freejoint = "mirror_freejoint";
    mirror.contact_bodies.push_back({"Car-1", "car_freejoint"});
    mirror.has_input = false;
    definition.mirror_bodies.push_back(mirror);
    definition.multi_dof_state_outputs.push_back({
        .component_id = "mirror_state",
        .bodies = {{"Drone-1", "Drone-1", "mirror_freejoint"}},
    });

    adapters::mujoco::MujocoActuatorPlant plant(definition);
    const auto assert_initial_state = [](const runtime::RobotState& state) {
        assert(state.bodies.size() == 1);
        const auto& body = state.bodies.front();
        assert(std::abs(body.position.x + 2.0) < epsilon);
        assert(std::abs(body.position.y) < epsilon);
        assert(std::abs(body.position.z - 1.0) < epsilon);
        assert(std::abs(body.orientation.w - 1.0) < epsilon);
        assert(std::abs(body.linear_velocity.x) < epsilon);
        assert(std::abs(body.angular_velocity.z) < epsilon);
    };
    plant.apply_operations({});
    assert_initial_state(plant.step({}));

    runtime::BodyMotionOperation command {
        .body = "other-mirror",
        .position = {0.1, 0.0, 1.0},
        .orientation = {0.0, 0.0, 0.5},
        .linear_velocity = {1.0, -2.0, 3.0},
        .angular_velocity = {0.0, 0.0, 0.75},
        .created_at_usec = 1'000,
    };
    plant.apply_operations({.body_motions = {command}});
    assert_initial_state(plant.step({}));

    command.body = "Drone-1";
    command.created_at_usec = 2'000;
    plant.apply_operations({.body_motions = {command}});
    const auto commanded = plant.read_state();
    assert(commanded.bodies.size() == 1);
    const auto& body = commanded.bodies.front();
    assert(std::abs(body.position.x - command.position.x) < epsilon);
    assert(std::abs(body.position.y - command.position.y) < epsilon);
    assert(std::abs(body.position.z - command.position.z) < epsilon);
    assert(std::abs(body.orientation.z - std::sin(command.orientation.yaw / 2.0)) < epsilon);
    assert(std::abs(body.orientation.w - std::cos(command.orientation.yaw / 2.0)) < epsilon);
    assert(std::abs(body.linear_velocity.x - command.linear_velocity.x) < epsilon);
    assert(std::abs(body.linear_velocity.y - command.linear_velocity.y) < epsilon);
    assert(std::abs(body.linear_velocity.z - command.linear_velocity.z) < epsilon);
    assert(std::abs(body.angular_velocity.z - command.angular_velocity.z) < epsilon);

    const auto state = plant.step({});
    assert(state.sample_time_usec == 3'000);
    assert(!state.mirror_contacts.empty());
    for (const auto& contact : state.mirror_contacts) {
        assert(contact.mirror_id == "Drone-1");
        assert(contact.local_body_id == "Car-1");
        assert(contact.target_mass > 0.0);
    }

    const auto reset = plant.reset(0);
    assert(reset.sample_time_usec == 0);
    assert_initial_state(reset);

    definition.mirror_bodies.front().mjcf_freejoint = "unknown_freejoint";
    bool rejected = false;
    try {
        (void)adapters::mujoco::MujocoActuatorPlant(definition);
    } catch (const std::invalid_argument& error) {
        rejected = std::string_view(error.what()) == "Mirror freejoint not found in MuJoCo model: unknown_freejoint";
    }
    assert(rejected);
    fs::remove(model_path);
}

} // namespace

int main()
{
    test_initial_body_pose_survives_reset();
    test_mirror_without_input_body_motion_and_contact_extraction();
    return 0;
}
