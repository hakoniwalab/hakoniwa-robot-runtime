#pragma once


#include "runtime/types.hpp"

#include <cstddef>
#include <span>
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

struct AsyncServiceStepResult {
    AsyncServiceStepOutcome outcome {AsyncServiceStepOutcome::Pending};
    std::vector<ActuatorCommand> commands;
};

/** One service, called synchronously from the simulation thread. */
class IAsyncService {
public:
    virtual ~IAsyncService() = default;

    /** The request is borrowed for this call. False schedules an INVALID reply. */
    virtual bool on_request(std::span<const std::byte> request) = 0;
    /** Receives pre-physics state and returns this substep's commands and outcome. */
    virtual AsyncServiceStepResult on_step(
        const RobotState& state, AsyncServiceState server_state,
        const RuntimeStepContext& context) = 0;

    /**
     * Writes the body of a prepared response packet using post-physics state.
     * Core reuses the buffer, so write every body field and preserve its header.
     * The span is borrowed for this call. Rejected requests skip this hook.
     */
    virtual bool on_response(
        const RobotState& state, const RuntimeStepContext& context,
        std::span<std::byte> packet) = 0;

    virtual void on_cancel() {}
    virtual void reset() = 0;
};

} // namespace hakoniwa::robot_runtime::runtime
