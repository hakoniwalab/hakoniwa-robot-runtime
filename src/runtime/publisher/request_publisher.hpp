#pragma once

#include "runtime/types.hpp"

#include <string_view>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

/**
 * Completes asynchronous requests after each physical step, even when regular
 * state publication is suppressed for a physics substep. Request IDs and
 * Controller statuses are observations, not proof of motion completion.
 * The application owns transport, response contents, and cancellation policy.
 */
class IRequestPublisher {
public:
    virtual ~IRequestPublisher() = default;
    [[nodiscard]] virtual std::string_view id() const noexcept = 0;
    [[nodiscard]] virtual ComponentStatus publish(
        const RobotState& state,
        const RuntimeStepContext& context,
        const std::vector<ProcessedInput>& inputs,
        const ArbitrationResult& arbitration) noexcept = 0;
    virtual void reset() noexcept = 0;
};

} // namespace hakoniwa::robot_runtime::runtime
