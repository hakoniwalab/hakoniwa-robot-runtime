#pragma once

#include "runtime/source/command_source.hpp"

#include <memory>
#include <string>
#include <string_view>

namespace hakoniwa::robot_runtime::runtime {

/**
 * Converts controller-specific input and current RobotState into command
 * candidates. It never writes a PDU or physical backend directly.
 *
 * A null input means that the bound source had no new sample this step. A
 * stateful controller may continue an active trajectory or apply its timeout
 * policy. Controllers that do not require an external CommandSource return an
 * empty source_id() and are called with a null input every step.
 *
 * The input is borrowed and immutable. Calls are serialized by
 * ActuatorRuntime; implementations need not be thread-safe.
 */
class IController {
public:
    virtual ~IController() = default;
    [[nodiscard]] virtual std::string_view id() const noexcept = 0;
    [[nodiscard]] virtual std::string_view source_id() const noexcept = 0;
    [[nodiscard]] virtual bool accepts(const IControllerInput& input) const noexcept = 0;
    [[nodiscard]] virtual ControllerOutput update(
        std::shared_ptr<const IControllerInput> input,
        const RobotState& current_state,
        const RuntimeStepContext& context) = 0;
    virtual void reset(const RobotState& current_state) = 0;

    /**
     * Start and stop events of the simulation (called by
     * ActuatorRuntime::start() / stop() from the simulation thread). The
     * defaults do nothing; built-in controllers keep their behaviour.
     */
    [[nodiscard]] virtual ComponentStatus on_start()
    {
        return {std::string(id()), ComponentState::Ready, {}};
    }
    [[nodiscard]] virtual ComponentStatus on_stop()
    {
        return {std::string(id()), ComponentState::Ready, {}};
    }

    /**
     * True when an Error status of this controller must end the run
     * (ActuatorRuntime throws from start(), step() and stop()). The default
     * keeps the existing behaviour: an Error status only removes the
     * controller's commands from the arbitration of that step.
     */
    [[nodiscard]] virtual bool error_stops_run() const noexcept
    {
        return false;
    }
};

} // namespace hakoniwa::robot_runtime::runtime
