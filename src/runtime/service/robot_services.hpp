#pragma once

#include "runtime/types.hpp"

#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

enum class AsyncServiceState {
    Idle,
    Doing,
    Canceling,
};

enum class AsyncServiceStepOutcome {
    Pending,
    Ok,
    Canceled,
    Error,
};

/** The framework sets the state and the robot sets the result for this step. */
struct RobotServiceSlot {
    AsyncServiceState state {AsyncServiceState::Idle};
    AsyncServiceStepOutcome result {AsyncServiceStepOutcome::Pending};
};

struct RobotStepResult {
    std::vector<ActuatorCommand> commands; // The controller sets source_id.
    std::string error {}; // Empty on success, otherwise the robot-level failure reason.
};

/** One robot's services, called synchronously from the simulation thread. */
class IRobotServices {
public:
    virtual ~IRobotServices() = default;

    /** False schedules an INVALID reply. The packet is borrowed for this call. */
    virtual bool on_request(std::size_t service, std::span<const std::byte> packet) = 0;
    virtual void on_cancel(std::size_t) {}
    virtual RobotStepResult on_step(const RobotState& state, const RuntimeStepContext& context,
        std::span<RobotServiceSlot> slots) = 0;

    /** Writes the reused response buffer after physics, preserving Core's header. */
    virtual bool on_response(std::size_t service, const RobotState& state,
        const RuntimeStepContext& context, std::span<std::byte> packet) = 0;
    virtual void reset() = 0;
};

} // namespace hakoniwa::robot_runtime::runtime
