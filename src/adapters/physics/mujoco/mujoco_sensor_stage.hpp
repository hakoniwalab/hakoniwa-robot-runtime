#pragma once

#include "runtime/runtime_definition.hpp"
#include "runtime/sensor/sensor_registry.hpp"
#include "runtime/sensor/sensor_stage.hpp"

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace hakoniwa::sensor_adapter {
class IRayCaster;
}

struct mjData_;
struct mjModel_;
using mjData = mjData_;
using mjModel = mjModel_;

namespace hakoniwa::robot_runtime::adapters::mujoco {

/**
 * Sensor stage backed by MuJoCo: reads the truth of every configured sensor
 * from mjData after the physics step and passes it to the sensor adapter of
 * the chosen implementation.
 *
 * Joint sensors: the truth is the position and velocity of the 1-DOF MJCF
 * joint (hinge or slide) after mj_step, stamped with the state time. When a
 * sensor measures the joint torque, the actuator force and the torque
 * transmitted through the joint are taken at that same state: mj_step leaves
 * forces computed for the state before the step, so the stage runs mj_forward
 * and mj_rnePostConstraint once after the step. These recompute derived
 * quantities only; they change neither the state nor the next step.
 *
 * Force-torque sensors: mounted on an MJCF body with a position and a
 * roll/pitch/yaw orientation (degrees, R = Rz(yaw) Ry(pitch) Rx(roll)) in the
 * body frame. The truth is the wrench the tool side (that body's subtree)
 * exerts on the robot side, in the sensor axes, torque about the sensor
 * origin, at the state after the step.
 *
 * IMUs: mounted like force-torque sensors. The truth is the angular velocity
 * and the specific force (acceleration including -gravity, as an ideal
 * accelerometer measures) at the sensor point, in the sensor axes, at the
 * state after the step.
 *
 * 2D LiDARs: mounted like IMUs, usually on a housing body. The world's ray
 * caster casts the rays at the state after the step (the stage runs
 * mj_forward once after mj_step). The rays never hit the subtree of the
 * exclusion root, by default the mount body, so the robot stays visible.
 */
class MujocoSensorStage final : public runtime::ISensorStage {
public:
    /** Mount of a ray caster: body, pose in the body frame, exclusion root. */
    struct RayCasterMount {
        std::string body;
        std::array<double, 3> position_m {0.0, 0.0, 0.0};
        std::array<double, 4> orientation_wxyz {1.0, 0.0, 0.0, 0.0};
        std::string exclude_root_body;  ///< empty: the mount body
    };
    /** Creates a ray caster from the world (nullptr when the world has none). */
    using RayCasterFactory =
        std::function<std::shared_ptr<::hakoniwa::sensor_adapter::IRayCaster>(const RayCasterMount&)>;

    /**
     * Resolves every mount in the model and creates the adapters.
     * Throws std::invalid_argument naming the sensor when a mount does not
     * exist or does not fit the sensor type, or when an adapter cannot be made.
     */
    MujocoSensorStage(
        const mjModel* model,
        mjData* data,
        const std::vector<runtime::RuntimeSensorConfig>& sensors,
        std::uint64_t seed,
        std::uint64_t physics_step_usec,
        const runtime::sensor::SensorImplementationRegistry& registry,
        const RayCasterFactory& ray_caster_factory = {});
    ~MujocoSensorStage() override;

    void step(runtime::RobotState& state) override;
    void reset() override;

private:
    struct Sensor;
    void step_joint(Sensor& sensor, std::uint64_t local_time_ns);
    void step_force_torque(Sensor& sensor, std::uint64_t local_time_ns);
    void step_imu(Sensor& sensor, std::uint64_t local_time_ns);
    void step_lidar(Sensor& sensor, std::uint64_t local_time_ns);

    const mjModel* model_;
    mjData* data_;
    // A joint torque needs forces at the state after the step (see step()).
    bool needs_dynamics_ {false};
    // Rays need the geom poses of the state after the step.
    bool needs_kinematics_ {false};
    std::uint64_t physics_step_usec_;
    // Sensor adapters count time from 0 in whole physics steps. The stage
    // keeps that count and the Runtime time of its first step since reset.
    std::uint64_t steps_ {0};
    std::uint64_t first_step_time_usec_ {0};
    std::vector<std::unique_ptr<Sensor>> sensors_;
};

} // namespace hakoniwa::robot_runtime::adapters::mujoco
