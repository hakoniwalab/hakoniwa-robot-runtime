// Issue #56: the MuJoCo sensor stage must take the truth with the acceleration
// the integrator applied, not mj_forward's explicit one. A hinge arm driven by a
// stiff position servo (kv 200) under implicitfast turns at constant speed; the
// force the tool exerts on the arm must be m (g - a) with a the centripetal
// acceleration only. Uses its own CHECK so the tests also run in Release builds.
#include "adapters/physics/mujoco/mujoco_sensor_stage.hpp"
#include "runtime/sensor/sensor_registry.hpp"

#include <mujoco/mujoco.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace sa = ::hakoniwa::sensor_adapter;
namespace rt = ::hakoniwa::robot_runtime::runtime;
using ::hakoniwa::robot_runtime::adapters::mujoco::MujocoSensorStage;

HAKO_SENSOR_ADAPTER_DECLARE_IMPLEMENTATION(hakoniwa);

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

const char* kModel = R"(<mujoco>
  <option timestep="0.001" integrator="%s"/>
  <worldbody>
    <body name="arm">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="0.5"/>
      <body name="tool" pos="0.3 0 0">
        <inertial pos="0 0 0" mass="1" diaginertia="0.001 0.001 0.001"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <general joint="hinge" gaintype="fixed" biastype="affine" gainprm="2000" biasprm="0 -2000 -200"/>
  </actuator>
</mujoco>)";

std::string ideal_definition()
{
    const auto path = std::filesystem::temp_directory_path() / "hako_truth_acceleration_ft.json";
    std::ofstream(path) << R"({"type": "force_torque", "update_rate": 1000,
        "range": {"fxy": 1000, "fz": 1000, "txy": 1000, "tz": 1000}})";
    return path.string();
}

// Largest error of the measured force against m (g - a_centripetal), in N,
// while the arm turns at 1 rad/s.
double worst_force_error(const char* integrator)
{
    char xml[2048];
    std::snprintf(xml, sizeof(xml), kModel, integrator);
    char error[512] = {0};
    mjSpec* spec = mj_parseXMLString(xml, nullptr, error, sizeof(error));
    if (spec == nullptr) {
        throw std::runtime_error(error);
    }
    mjModel* m = mj_compile(spec, nullptr);
    mj_deleteSpec(spec);
    mjData* d = mj_makeData(m);

    rt::sensor::SensorImplementationRegistry registry;
    std::string message;
    if (!registry.add(HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(hakoniwa)(), &message)) {
        throw std::runtime_error(message);
    }
    rt::RuntimeSensorConfig ft;
    ft.name = "ft";
    ft.type = "force_torque";
    ft.implementation = "hakoniwa";
    ft.definition_path = ideal_definition();
    ft.pdu_robot = "arm";
    ft.mjcf_body = "tool";
    MujocoSensorStage stage(m, d, {ft}, 1, 1000, registry);

    const double omega = 1.0;
    const int tool = mj_name2id(m, mjOBJ_BODY, "tool");
    rt::RobotState state;
    double worst = 0.0;
    for (int k = 0; k < 3000; ++k) {
        d->ctrl[0] = omega * (k + 1) * m->opt.timestep;
        mj_step(m, d);
        state.sample_time_usec = static_cast<std::uint64_t>(k) * 1000;
        stage.step(state);
        if (k < 2000) {
            continue;  // the servo settles to a constant speed (lag omega kv / kp)
        }
        // Expected: the tool (point mass at radius r about the y axis) at constant speed.
        const mjtNum* p = d->xipos + 3 * tool;
        const double radius[3] = {p[0], 0.0, p[2]};
        const double w = d->qvel[0];
        double load_world[3];  // m (g - a), a = -w^2 r
        for (int i = 0; i < 3; ++i) {
            load_world[i] = 1.0 * (m->opt.gravity[i] + w * w * radius[i]);
        }
        const mjtNum* rot = d->xmat + 9 * tool;  // world <- tool
        const auto& reading = state.sensor_readings[0];
        for (int i = 0; i < 3; ++i) {
            const double expected = rot[0 + i] * load_world[0] + rot[3 + i] * load_world[1] + rot[6 + i] * load_world[2];
            worst = std::max(worst, std::fabs(reading.force[static_cast<std::size_t>(i)] - expected));
        }
    }
    mj_deleteData(d);
    mj_deleteModel(m);
    return worst;
}

} // namespace

int main()
{
    const double implicitfast = worst_force_error("implicitfast");
    const double implicit = worst_force_error("implicit");
    const double euler = worst_force_error("Euler");
    std::cout << "worst force error [N]: implicitfast " << implicitfast << ", implicit " << implicit
              << ", Euler " << euler << std::endl;
    CHECK(implicitfast < 0.02);
    CHECK(implicit < 0.02);
    CHECK(euler < 0.02);
    if (failures != 0) {
        std::cerr << failures << " check(s) failed" << std::endl;
        return 1;
    }
    std::cout << "sensor truth acceleration tests passed" << std::endl;
    return 0;
}
