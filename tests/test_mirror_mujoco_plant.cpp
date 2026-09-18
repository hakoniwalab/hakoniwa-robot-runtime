#include "hakoniwa/robot_runtime/adapters/physics/mujoco/mujoco_actuator_plant.hpp"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <string>

using namespace hakoniwa::robot_runtime;
namespace fs = std::filesystem;

namespace {

void test_mirror_application_and_contact_extraction()
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
    <body name="mirror" pos="0 0 1">
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
    definition.mirror_bodies.push_back(mirror);

    adapters::mujoco::MujocoActuatorPlant plant(definition);
    runtime::MirrorBodyCommand command;
    command.mirror_id = "Drone-1";
    command.position = {0.0, 0.0, 1.0};
    command.orientation = {0.0, 0.0, 0.0};
    command.linear_velocity = {1.0, 0.0, 0.0};
    command.created_at_usec = 0;

    const auto state = plant.step({}, {command});
    assert(state.sample_time_usec == 1'000);
    assert(!state.mirror_contacts.empty());
    for (const auto& contact : state.mirror_contacts) {
        assert(contact.mirror_id == "Drone-1");
        assert(contact.local_body_id == "Car-1");
        assert(contact.target_mass > 0.0);
    }

    const auto reset = plant.reset(0);
    assert(reset.sample_time_usec == 0);
    fs::remove(model_path);
}

} // namespace

int main()
{
    test_mirror_application_and_contact_extraction();
    return 0;
}
