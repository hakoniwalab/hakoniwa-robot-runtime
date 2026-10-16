#pragma once

namespace hakoniwa::robot_runtime::runner {

/**
 * An input the Runner waits for before each step in lockstep mode
 * (HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP, a run option).
 *
 * Hakoniwa delivers PDU receive callbacks at a step boundary, so a step can see
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
    /** True when the next step may run: a new value has been read, or none has
     * ever arrived. Never blocks. */
    [[nodiscard]] virtual bool lockstep_ready() noexcept = 0;
    /** Called at the asset's initialization, before simulation time moves: from
     * here on every step waits for a new value (the first step takes the value the
     * writer wrote at its own initialization). */
    virtual void lockstep_prepare() noexcept {}
};

} // namespace hakoniwa::robot_runtime::runner
