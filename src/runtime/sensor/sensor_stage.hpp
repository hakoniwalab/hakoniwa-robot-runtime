#pragma once

#include "runtime/types.hpp"

namespace hakoniwa::robot_runtime::runtime {

/**
 * The sensor stage of one Runtime step:
 *   arbiter -> actuator (Plant.step) -> sensor -> publisher.
 *
 * step() runs after every physics step (also every substep, because sensor
 * periods are counted in physics steps). It reads the truth from the physics
 * backend, passes it to the sensor implementations and writes the latest
 * measured values into state.sensor_readings, stamped with the state time.
 * Failures are reported by exception, like a failed Plant step.
 */
class ISensorStage {
public:
    virtual ~ISensorStage() = default;
    virtual void step(RobotState& state) = 0;
    /** Restores every sensor to its initial state, including its random streams. */
    virtual void reset() = 0;
};

} // namespace hakoniwa::robot_runtime::runtime
