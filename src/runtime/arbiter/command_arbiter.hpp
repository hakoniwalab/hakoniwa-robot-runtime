#pragma once

#include "runtime/types.hpp"

#include <vector>

namespace hakoniwa::robot_runtime::runtime {

/**
 * Selects one logical Controller/control group from the candidates produced in
 * the current Runtime step.
 *
 * Controllers generate candidates independently. The Arbiter applies only the
 * selection policy; it does not create commands, track control mode, or guard
 * physical command validity. Type/time/finite/limit/fallback checks remain the
 * Plant entrance responsibility.
 */
class ICommandArbiter {
public:
    virtual ~ICommandArbiter() = default;
    [[nodiscard]] virtual ArbitrationResult arbitrate(
        const std::vector<ControllerOutput>& controller_outputs,
        const RobotState& current_state,
        const RuntimeStepContext& context) const = 0;
};

} // namespace hakoniwa::robot_runtime::runtime
