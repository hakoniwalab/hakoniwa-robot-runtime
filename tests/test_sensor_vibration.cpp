// Tests of the force-torque vibration in the MuJoCo sensor stage: the stage passes
// the velocities of the mapped MJCF joints to the implementation's vibration
// model and adds its output to the truth before the force-torque adapter.
// Uses its own CHECK so the tests also run in Release builds.
#include "adapters/physics/mujoco/mujoco_sensor_stage.hpp"
#include "runtime/sensor/sensor_registry.hpp"

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace sa = ::hakoniwa::sensor_adapter;
namespace rt = ::hakoniwa::robot_runtime::runtime;
using ::hakoniwa::robot_runtime::adapters::mujoco::MujocoSensorStage;

HAKO_SENSOR_ADAPTER_DECLARE_IMPLEMENTATION(hakoniwa);
HAKO_SENSOR_HOST_MODEL_DECLARE(hakoniwa);

namespace {

int failures = 0;

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            ++failures;                                                         \
            std::cerr << __FILE__ << ":" << __LINE__ << ": CHECK failed: "      \
                      << #condition << std::endl;                               \
        }                                                                       \
    } while (false)

constexpr double kPi = 3.14159265358979323846;
constexpr std::uint64_t kStepUsec = 1000;

const char* kModel = R"(<mujoco>
  <option timestep="0.001">
    <flag contact="disable"/>
  </option>
  <worldbody>
    <body name="arm">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <geom type="box" size="0.05 0.05 0.05" mass="1"/>
      <body name="tool" pos="0.2 0 0">
        <geom type="box" size="0.02 0.02 0.02" mass="0.5"/>
      </body>
    </body>
    <body name="other">
      <joint name="ball" type="ball"/>
      <geom type="sphere" size="0.01" mass="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="m1" joint="j1"/>
  </actuator>
</mujoco>)";

std::string write_file(const std::string& name, const std::string& text)
{
    const auto path = std::filesystem::temp_directory_path() / ("hako_vibration_test_" + name);
    std::ofstream(path) << text;
    return path.string();
}

// Ideal force-torque sensor of the hakoniwa implementation (no error terms).
const std::string& ideal_definition()
{
    static const std::string path = write_file("ft.json",
        R"({"type": "force_torque", "update_rate": 1000,
            "range": {"fxy": 1000, "fz": 1000, "txy": 1000, "tz": 1000}})");
    return path;
}

// A fake vibration model: Fx = 10 * velocity, Tz = 0.5, counts resets.
struct FakeState {
    std::vector<double> velocities;
    int resets {0};
};
FakeState fake;

class FakeVibration final : public sa::IForceTorqueVibration {
public:
    bool step(std::uint64_t, const std::vector<double>& v, sa::Wrench& out, std::string& error) override
    {
        if (v.size() != 1) {
            error = "expected one velocity";
            return false;
        }
        fake.velocities.push_back(v[0]);
        out = {};
        out.force.x = 10.0 * v[0];
        out.torque.z = 0.5;
        return true;
    }
    bool reset(std::string&) override
    {
        ++fake.resets;
        return true;
    }
    std::vector<std::string> joint_names() const override { return {"model_joint"}; }
};

std::unique_ptr<sa::IForceTorqueVibration> create_fake(const sa::ForceTorqueVibrationSetup&, std::string&)
{
    return std::make_unique<FakeVibration>();
}

// A fake model with a load gain: refuses the step without loads, records the loads.
std::vector<double> fake_loads;
class FakeLoadVibration final : public sa::IForceTorqueVibration {
public:
    bool step(std::uint64_t, const std::vector<double>&, sa::Wrench&, std::string& error) override
    {
        error = "needs loads";
        return false;
    }
    bool step(std::uint64_t, const std::vector<double>& v, const std::vector<double>& load, sa::Wrench& out,
        std::string& error) override
    {
        if (v.size() != 1 || load.size() != 1) {
            error = "expected one velocity and one load";
            return false;
        }
        fake_loads.push_back(load[0]);
        out = {};
        return true;
    }
    bool needs_joint_load() const override { return true; }
    bool reset(std::string&) override { return true; }
    std::vector<std::string> joint_names() const override { return {"model_joint"}; }
};

std::unique_ptr<sa::IForceTorqueVibration> create_fake_load(const sa::ForceTorqueVibrationSetup&, std::string&)
{
    return std::make_unique<FakeLoadVibration>();
}

const sa::HostModelImplementation* fake_load_models()
{
    static const sa::HostModelImplementation models = [] {
        sa::HostModelImplementation value;
        value.name = "hakoniwa";
        value.create_force_torque_vibration = &create_fake_load;
        return value;
    }();
    return &models;
}

const sa::HostModelImplementation* fake_models()
{
    static const sa::HostModelImplementation models = [] {
        sa::HostModelImplementation value;
        value.name = "hakoniwa";
        value.create_force_torque_vibration = &create_fake;
        return value;
    }();
    return &models;
}

rt::RuntimeSensorConfig ft_sensor(const std::string& name)
{
    rt::RuntimeSensorConfig config;
    config.name = name;
    config.type = "force_torque";
    config.implementation = "hakoniwa";
    config.definition_path = ideal_definition();
    config.pdu_robot = "arm";
    config.mjcf_body = "tool";
    return config;
}

struct World {
    mjModel* model {nullptr};
    mjData* data {nullptr};
    World()
    {
        char error[512] = {0};
        mjSpec* spec = mj_parseXMLString(kModel, nullptr, error, sizeof(error));
        if (spec == nullptr) {
            throw std::runtime_error(error);
        }
        model = mj_compile(spec, nullptr);
        mj_deleteSpec(spec);
        data = mj_makeData(model);
    }
    ~World()
    {
        mj_deleteData(data);
        mj_deleteModel(model);
    }
};

rt::sensor::SensorImplementationRegistry registry_with(const sa::HostModelImplementation* models)
{
    rt::sensor::SensorImplementationRegistry registry;
    std::string error;
    if (!registry.add(HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(hakoniwa)(), &error)) {
        throw std::runtime_error(error);
    }
    if (models != nullptr && !registry.add_host_models(models, &error)) {
        throw std::runtime_error(error);
    }
    return registry;
}

std::string construction_error(const std::vector<rt::RuntimeSensorConfig>& sensors,
    const rt::sensor::SensorImplementationRegistry& registry)
{
    World world;
    try {
        MujocoSensorStage stage(world.model, world.data, sensors, 1, kStepUsec, registry);
    } catch (const std::invalid_argument& e) {
        return e.what();
    }
    return "";
}

void vibration_is_added_with_the_joint_velocity()
{
    fake = {};
    World world;
    auto with = ft_sensor("with");
    with.vibration_definition_path = ideal_definition();  // the fake reads nothing
    with.vibration_joints = {{"model_joint", "j1"}};
    const auto registry = registry_with(fake_models());
    MujocoSensorStage stage(world.model, world.data, {ft_sensor("plain"), with}, 1, kStepUsec, registry);
    rt::RobotState state;
    const double velocities[] = {0.0, 0.7, -1.3};
    for (int k = 0; k < 3; ++k) {
        world.data->qvel[world.model->jnt_dofadr[mj_name2id(world.model, mjOBJ_JOINT, "j1")]] = velocities[k];
        state.sample_time_usec = static_cast<std::uint64_t>(k) * kStepUsec;
        stage.step(state);
        CHECK(state.sensor_readings.size() == 2);
        const auto& plain = state.sensor_readings[0];
        const auto& vibrated = state.sensor_readings[1];
        CHECK(plain.has_wrench && vibrated.has_wrench);
        // Exactly the model's output on top of the same physics wrench.
        CHECK(std::fabs(vibrated.force[0] - plain.force[0] - 10.0 * velocities[k]) < 1e-9);
        CHECK(std::fabs(vibrated.force[1] - plain.force[1]) < 1e-12);
        CHECK(std::fabs(vibrated.force[2] - plain.force[2]) < 1e-12);
        CHECK(std::fabs(vibrated.torque[2] - plain.torque[2] - 0.5) < 1e-9);
    }
    CHECK(fake.velocities.size() == 3);
    for (int k = 0; k < 3 && k < static_cast<int>(fake.velocities.size()); ++k) {
        CHECK(fake.velocities[static_cast<std::size_t>(k)] == velocities[k]);
    }
    stage.reset();
    CHECK(fake.resets == 1);
}

void real_model_adds_an_order_line()
{
    // The hakoniwa vibration model: gear 10, order 1, amplitude 1 N on Fx;
    // 2 pi rad/s gives a 10 Hz sine of amplitude 1 N.
    World world;
    auto with = ft_sensor("with");
    with.vibration_definition_path = write_file("vib.json",
        R"({"type": "force_torque_vibration", "joints": [{"name": "a", "gear_ratio": 10,
            "orders": [{"order": 1, "amplitude": {"fx": 1}}]}]})");
    with.vibration_joints = {{"a", "j1"}};
    const auto registry = registry_with(HAKO_SENSOR_HOST_MODEL_ENTRY(hakoniwa)());
    MujocoSensorStage stage(world.model, world.data, {ft_sensor("plain"), with}, 1, kStepUsec, registry);
    rt::RobotState state;
    const int dof = world.model->jnt_dofadr[mj_name2id(world.model, mjOBJ_JOINT, "j1")];
    double peak = 0.0;
    int crossings = 0;
    double previous = 0.0;
    for (int k = 0; k < 1000; ++k) {
        world.data->qvel[dof] = 2.0 * kPi;
        state.sample_time_usec = static_cast<std::uint64_t>(k) * kStepUsec;
        stage.step(state);
        const double d = state.sensor_readings[1].force[0] - state.sensor_readings[0].force[0];
        peak = std::max(peak, std::fabs(d));
        if (k > 0 && (d < 0) != (previous < 0)) {
            ++crossings;
        }
        previous = d;
    }
    CHECK(std::fabs(peak - 1.0) < 2e-3);
    CHECK(crossings >= 19 && crossings <= 21);  // 10 Hz for 1 s
}

void loads_are_the_transmitted_torque()
{
    // A model with a load gain gets the torque the joint transmits (parent side on child side about the axis):
    // here the gravity torque of the arm and tool about the horizontal hinge, at constant speed. Expected value:
    // the generalized force mj_inverse needs at the same state with zero acceleration.
    fake_loads.clear();
    World world;
    auto with = ft_sensor("with");
    with.vibration_definition_path = ideal_definition();
    with.vibration_joints = {{"model_joint", "j1"}};
    const auto registry = registry_with(fake_load_models());
    MujocoSensorStage stage(world.model, world.data, {with}, 1, kStepUsec, registry);
    const int dof = world.model->jnt_dofadr[mj_name2id(world.model, mjOBJ_JOINT, "j1")];
    rt::RobotState state;
    std::vector<double> expected;
    mjData* copy = mj_makeData(world.model);
    for (int k = 0; k < 3; ++k) {
        world.data->qpos[0] = 0.3 * k;   // turned, at constant speed (no step in between: the applied acceleration is 0)
        world.data->qvel[dof] = 0.7;
        world.data->ctrl[0] = 5.0;       // the motor torque is not the load
        state.sample_time_usec = static_cast<std::uint64_t>(k) * kStepUsec;
        stage.step(state);
        mj_copyData(copy, world.model, world.data);
        mju_zero(copy->qacc, world.model->nv);
        copy->qvel[dof] = 0.0;           // the gravity part only (no Coriolis on a single hinge)
        mj_inverse(world.model, copy);
        expected.push_back(copy->qfrc_inverse[dof]);
    }
    mj_deleteData(copy);
    CHECK(fake_loads.size() == 3);
    // from the second step on the stage uses the applied acceleration (velocity change 0)
    for (std::size_t k = 1; k < fake_loads.size() && k < 3; ++k) {
        CHECK(std::fabs(fake_loads[k] - expected[k]) < 1e-9);
        CHECK(std::fabs(fake_loads[k]) > 0.5);  // the gravity torque is there (0.98 N m at k = 0)
        CHECK(std::fabs(fake_loads[k] - 5.0) > 1.0);
    }
}

void real_model_with_load_gain()
{
    // The hakoniwa model with load_gain: positive power (motor torque along the motion) gives the positive gain.
    World world;
    auto with = ft_sensor("with");
    with.vibration_definition_path = write_file("vib_load.json",
        R"({"type": "force_torque_vibration", "joints": [{"name": "a", "gear_ratio": 10,
            "load_gain": {"positive": 0.5, "negative": 2},
            "orders": [{"order": 1, "amplitude": {"fx": 1}}]}]})");
    with.vibration_joints = {{"a", "j1"}};
    const auto registry = registry_with(HAKO_SENSOR_HOST_MODEL_ENTRY(hakoniwa)());
    MujocoSensorStage stage(world.model, world.data, {ft_sensor("plain"), with}, 1, kStepUsec, registry);
    rt::RobotState state;
    const int dof = world.model->jnt_dofadr[mj_name2id(world.model, mjOBJ_JOINT, "j1")];
    double peak_positive = 0.0, peak_negative = 0.0;
    for (int k = 0; k < 600; ++k) {
        world.data->qvel[dof] = k < 300 ? 2.0 * kPi : -2.0 * kPi;  // the transmitted (gravity) torque keeps its sign,
        world.data->qpos[0] = 0.0;                                // so reversing the motion flips tau * qdot
        state.sample_time_usec = static_cast<std::uint64_t>(k) * kStepUsec;
        stage.step(state);
        const double d = std::fabs(state.sensor_readings[1].force[0] - state.sensor_readings[0].force[0]);
        (k < 300 ? peak_positive : peak_negative) = std::max(k < 300 ? peak_positive : peak_negative, d);
    }
    // The sign of the transmitted torque at qpos 0 decides which half has positive power.
    mjData* copy = mj_makeData(world.model);
    mj_inverse(world.model, copy);
    const bool first_half_positive = copy->qfrc_inverse[dof] > 0;  // tau * (+qdot) > 0
    mj_deleteData(copy);
    CHECK(std::fabs(peak_positive - (first_half_positive ? 0.5 : 2.0)) < 1e-2);
    CHECK(std::fabs(peak_negative - (first_half_positive ? 2.0 : 0.5)) < 1e-2);
}

void configuration_errors()
{
    const auto registry = registry_with(fake_models());
    auto missing = ft_sensor("ft");
    missing.vibration_definition_path = ideal_definition();
    CHECK(construction_error({missing}, registry).find("model_joint") != std::string::npos);

    auto unknown = missing;
    unknown.vibration_joints = {{"model_joint", "j1"}, {"extra", "j1"}};
    CHECK(construction_error({unknown}, registry).find("extra") != std::string::npos);

    auto no_joint = missing;
    no_joint.vibration_joints = {{"model_joint", "nope"}};
    CHECK(construction_error({no_joint}, registry).find("nope") != std::string::npos);

    auto ball = missing;
    ball.vibration_joints = {{"model_joint", "ball"}};
    CHECK(construction_error({ball}, registry).find("hinge or slide") != std::string::npos);

    auto ok = missing;
    ok.vibration_joints = {{"model_joint", "j1"}};
    CHECK(construction_error({ok}, registry).empty());
    CHECK(construction_error({ok}, registry_with(nullptr)).find("no force-torque vibration model")
        != std::string::npos);
}

void host_model_registry()
{
    rt::sensor::SensorImplementationRegistry registry;
    std::string error;
    CHECK(!registry.add_host_models(nullptr, &error));
    CHECK(registry.add_host_models(fake_models(), &error));
    CHECK(!registry.add_host_models(fake_models(), &error));
    CHECK(error.find("twice") != std::string::npos);
    CHECK(registry.find_host_models("hakoniwa") == fake_models());
    CHECK(registry.find_host_models("gazebo") == nullptr);
    registry.clear();
    CHECK(registry.find_host_models("hakoniwa") == nullptr);
}

} // namespace

int main()
{
    host_model_registry();
    vibration_is_added_with_the_joint_velocity();
    real_model_adds_an_order_line();
    configuration_errors();
    loads_are_the_transmitted_torque();
    real_model_with_load_gain();
    if (failures != 0) {
        std::cerr << failures << " check(s) failed" << std::endl;
        return 1;
    }
    std::cout << "sensor vibration tests passed" << std::endl;
    return 0;
}
