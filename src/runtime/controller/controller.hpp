#pragma once

#include "runtime/source/command_source.hpp"

#include <memory>
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
};

} // namespace hakoniwa::robot_runtime::runtime
