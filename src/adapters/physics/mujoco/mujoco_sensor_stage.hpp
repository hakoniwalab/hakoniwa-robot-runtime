#pragma once

#include "runtime/runtime_definition.hpp"
#include "runtime/sensor/sensor_registry.hpp"
#include "runtime/sensor/sensor_stage.hpp"

#include <memory>
#include <vector>

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
 * Phase 1 supports joint sensors only. The truth is the joint position and
 * velocity of the 1-DOF MJCF joint (hinge or slide) after mj_step, stamped
 * with the state time; the joint torque is not computed yet.
 */
class MujocoSensorStage final : public runtime::ISensorStage {
public:
    /**
     * Resolves every mount in the model and creates the adapters.
     * Throws std::invalid_argument naming the sensor when a mount does not
     * exist or does not fit the sensor type, or when an adapter cannot be made.
     */
    MujocoSensorStage(
        const mjModel* model,
        const mjData* data,
        const std::vector<runtime::RuntimeSensorConfig>& sensors,
        std::uint64_t seed,
        std::uint64_t physics_step_usec,
        const runtime::sensor::SensorImplementationRegistry& registry);
    ~MujocoSensorStage() override;

    void step(runtime::RobotState& state) override;
    void reset() override;

private:
    struct Sensor;
    const mjModel* model_;
    const mjData* data_;
    std::uint64_t physics_step_usec_;
    // Sensor adapters count time from 0 in whole physics steps. The stage
    // keeps that count and the Runtime time of its first step since reset.
    std::uint64_t steps_ {0};
    std::uint64_t first_step_time_usec_ {0};
    std::vector<std::unique_ptr<Sensor>> sensors_;
};

} // namespace hakoniwa::robot_runtime::adapters::mujoco
