#pragma once

#include <string_view>

namespace hakoniwa::robot_runtime::runner {

/**
 * An input the Runner waits for before each step in lockstep mode
 * (HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP, a run option).
 *
 * For PDU inputs Hakoniwa delivers receive callbacks at a step boundary, so a step can see
 * the previous command or the new one depending on when the writer ran: repeated
 * runs drift apart. A lockstep input instead checks the receive flag and, when it
 * is set, reads the PDU synchronously. Until a new value has arrived the Runner
 * neither steps nor advances its Hakoniwa time, so every step uses the command
 * written for it, in simulation-time order.
 *
 * The writer must write once at its initialization and then every step. Inputs
 * written only on change must not be lockstep inputs.
 */
class ILockstepInput {
public:
    virtual ~ILockstepInput() = default;
    /** True when this input permits the next step. Keep any staged input
     * available until the step consumes it or the input is canceled. Never blocks. */
    [[nodiscard]] virtual bool lockstep_ready() noexcept = 0;
    /** While paused, allow an input to withdraw a value it has already staged. */
    virtual void lockstep_poll_while_paused() noexcept {}
    /** Called at the asset's initialization, before simulation time moves: from
     * here on every step waits for a new value (the first step takes the value the
     * writer wrote at its own initialization). */
    virtual void lockstep_prepare() noexcept {}
    /** Called when Core discards a run. Existing PDU inputs reset in the Runtime. */
    virtual void lockstep_reset() noexcept {}
    /** A fatal input error, or an empty view if the input is healthy. */
    [[nodiscard]] virtual std::string_view lockstep_failure_reason() const noexcept { return {}; }
    /** Enable the deadline and pause for the entire batch. Untimed inputs
     * still count toward readiness when sharing a gate with a timed input. */
    [[nodiscard]] virtual bool lockstep_uses_deadline() const noexcept { return false; }
    /** PDU readers synchronize once before the first physics step. */
    [[nodiscard]] virtual bool lockstep_needs_initial_sync() const noexcept { return true; }
};

} // namespace hakoniwa::robot_runtime::runner
