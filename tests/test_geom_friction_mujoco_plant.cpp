#undef NDEBUG

#include "hakoniwa/robot_runtime/adapters/physics/mujoco/mujoco_actuator_plant.hpp"

#if HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE
#include "adapters/endpoint/ackermann_step_service.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepRequestPacket.hpp"
#include "runtime/controller/geom_friction_controller.hpp"

#include <array>
#endif

#include <mujoco/mujoco.h>

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace hakoniwa::robot_runtime;
namespace fs = std::filesystem;

namespace {

const fs::path model_path =
    fs::temp_directory_path() / "hakoniwa-geom-friction-plant-test.xml";

void write_model(bool four_tires = false)
{
    std::ofstream model(model_path);
    assert(model.is_open());
    // The World plane has a high friction. The model (as the asset composer
    // does) gives the tire a higher geom priority, so the tire friction is
    // used in tire/ground contacts. The Runtime never changes priorities.
    model << R"(<mujoco model="geom-friction-test">
  <option timestep="0.001"/>
  <worldbody>
    <geom name="ground" type="plane" size="20 20 0.1" friction="1 0.005 0.0001"/>
    <body name="car" pos="0 0 0.1">
      <freejoint name="car_freejoint"/>
      <geom name="tire" type="box" size="0.1 0.1 0.1" mass="1"
            friction="1 0.02 0.003" priority="1"/>)";
    if (four_tires) {
        for (const auto* name : {"tire2", "tire3", "tire4"}) {
            model << R"(<geom type="sphere" size="0.02" mass="0.1"
                contype="0" conaffinity="0" name=")" << name << "\"/>\n";
        }
    }
    model << R"(
      <geom name="body_shell" type="sphere" size="0.05" pos="0 0 0.1" mass="0.1"
            contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>)";
}

runtime::RuntimeDefinition definition_with(std::vector<std::string> geoms)
{
    runtime::RuntimeDefinition definition;
    definition.model_path = model_path.string();
    runtime::RuntimeGeomFrictionConfig friction;
    friction.component_id = "car-tire-friction";
    friction.geoms = std::move(geoms);
    definition.geom_frictions.push_back(friction);
    return definition;
}

runtime::PlantDirectiveList friction_directive(
    const double value,
    const std::uint64_t created_at_usec = 0)
{
    return {std::make_shared<const runtime::GeomFrictionDirective>(
        std::vector<std::string> {"tire", "not_bound"},
        value,
        created_at_usec)};
}

void test_unknown_geom_is_a_configuration_error()
{
    bool rejected = false;
    try {
        adapters::mujoco::MujocoActuatorPlant plant(
            definition_with({"tire", "missing_tire"}));
    } catch (const std::invalid_argument& error) {
        rejected = std::string(error.what()).find("missing_tire")
            != std::string::npos;
    }
    assert(rejected);
}

void test_directive_and_reset()
{
    adapters::mujoco::MujocoActuatorPlant plant(definition_with({"tire"}));
    auto* model = plant.model();
    const int tire = mj_name2id(model, mjOBJ_GEOM, "tire");
    const int ground = mj_name2id(model, mjOBJ_GEOM, "ground");
    const int shell = mj_name2id(model, mjOBJ_GEOM, "body_shell");
    assert(tire >= 0 && ground >= 0 && shell >= 0);

    // Binding leaves the model's geom priorities untouched.
    assert(model->geom_priority[tire] == 1);
    assert(model->geom_priority[ground] == 0);
    assert(model->geom_priority[shell] == 0);

    plant.apply_directives(friction_directive(0.25));
    assert(model->geom_friction[3 * tire] == 0.25);
    // Torsional / rolling friction and other geoms are unchanged.
    assert(model->geom_friction[3 * tire + 1] == 0.02);
    assert(model->geom_friction[3 * tire + 2] == 0.003);
    assert(model->geom_friction[3 * ground] == 1.0);
    assert(model->geom_friction[3 * shell] == 1.0);

    // The value persists while no further directive arrives.
    (void)plant.step({});
    plant.apply_directives({});
    (void)plant.step({});
    assert(model->geom_friction[3 * tire] == 0.25);

    // The Plant guard ignores non-finite, negative, and future directives.
    plant.apply_directives(friction_directive(
        std::numeric_limits<double>::quiet_NaN()));
    plant.apply_directives(friction_directive(-1.0));
    plant.apply_directives(friction_directive(0.5, 1'000'000));
    assert(model->geom_friction[3 * tire] == 0.25);

    plant.apply_directives(friction_directive(0.6));
    assert(model->geom_friction[3 * tire] == 0.6);

    (void)plant.reset(0);
    assert(model->geom_friction[3 * tire] == 1.0);
    assert(model->geom_friction[3 * tire + 1] == 0.02);
    assert(model->geom_priority[tire] == 1);
}

double slide_distance(const bool lower_friction)
{
    adapters::mujoco::MujocoActuatorPlant plant(definition_with({"tire"}));
    // Let the box settle on the ground, then push it.
    for (int index = 0; index < 50; ++index) {
        (void)plant.step({});
    }
    auto* data = plant.data();
    const int joint = mj_name2id(plant.model(), mjOBJ_JOINT, "car_freejoint");
    const int qpos = plant.model()->jnt_qposadr[joint];
    const int qvel = plant.model()->jnt_dofadr[joint];
    const double start_x = data->qpos[qpos];
    data->qvel[qvel] = 2.0;
    if (lower_friction) {
        plant.apply_directives(friction_directive(0.05));
    }
    for (int index = 0; index < 500; ++index) {
        (void)plant.step({});
    }
    return data->qpos[qpos] - start_x;
}

void test_directive_friction_takes_effect_in_contacts()
{
    // The tire's model priority makes its friction win over the ground's
    // (equal priority would take max(1.0, 0.05) = 1.0), so the longer slide
    // shows that the directive changed the friction used by physics.
    const double default_distance = slide_distance(false);
    const double low_distance = slide_distance(true);
    assert(std::isfinite(default_distance) && std::isfinite(low_distance));
    assert(low_distance > default_distance + 0.3);
}

#if HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE
void test_ackermann_step_friction_applies_to_all_wheels_before_physics()
{
    using namespace adapters::hakoniwa;

    write_model(true);
    const std::vector<std::string> geoms {"tire", "tire2", "tire3", "tire4"};
    adapters::mujoco::MujocoActuatorPlant plant(definition_with(geoms));
    auto friction = std::make_shared<AckermannStepFrictionInput>();
    AckermannStepService service(std::make_shared<AckermannDriveRpcEventReader>(), "car", friction);
    runtime::ScalarPduCommandSource source("tire-input", plant.delta_time_usec(), friction);
    runtime::GeomFrictionController controller("tire-directive", "tire-input", geoms);

    auto assert_wheel_friction = [&](double expected) {
        const auto* model = plant.model();
        for (const auto& geom : geoms) {
            const int id = mj_name2id(model, mjOBJ_GEOM, geom.c_str());
            assert(id >= 0);
            assert(model->geom_friction[3 * id] == expected);
        }
    };
    const std::array samples {std::pair {0.5, 0.5}, std::pair {0.0, 0.5}, std::pair {0.9, 0.9}};
    assert_wheel_friction(1.0);
    std::uint64_t request_step = 0;
    std::uint64_t physics_step = 0;
    for (const auto& [requested, expected] : samples) {
        HakoCpp_AckermannStepRequestPacket message {};
        message.body.step = ++request_step;
        message.body.tire_friction = requested;
        std::array<std::byte, 1024> packet {};
        assert(hako::pdu::msgs::hako_srv_msgs::AckermannStepRequestPacket {}.cpp2pdu(
            message, reinterpret_cast<char*>(packet.data()), packet.size()) > 0);
        assert(service.on_request(packet));
        for (int substep = 0; substep < 4; ++substep, ++physics_step) {
            const runtime::RuntimeStepContext context {
                .simulation_time_usec = plant.read_state().sample_time_usec,
                .delta_time_usec = plant.delta_time_usec(),
                .step_index = physics_step,
            };
            const auto input = source.poll(context);
            const auto output = controller.update(input.input, context);
            const bool updates = substep == 0 && requested > 0.0;
            assert(input.has_new_input() == updates);
            assert(output.directives.size() == (updates ? 1 : 0));
            plant.apply_directives(output.directives);
            assert_wheel_friction(expected);
            (void)plant.step({});
        }
    }
}
#endif

} // namespace

int main()
{
    write_model();
    test_unknown_geom_is_a_configuration_error();
    test_directive_and_reset();
    test_directive_friction_takes_effect_in_contacts();
#if HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE
    test_ackermann_step_friction_applies_to_all_wheels_before_physics();
#endif
    fs::remove(model_path);
    return 0;
}
