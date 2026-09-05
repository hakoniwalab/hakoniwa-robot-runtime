#pragma once

#include "runtime/types.hpp"

#include <string_view>

namespace hakoniwa::robot_runtime::runtime {

/**
 * Publishes an immutable post-physics RobotState to an external boundary.
 *
 * `state.sample_time_usec` always equals `context.simulation_time_usec`.
 * Publishing must not modify Plant state or block simulation indefinitely.
 * Transport failure is returned as ComponentStatus and does not roll back the
 * already completed physical step.
 */
class IStatePublisher {
public:
    virtual ~IStatePublisher() = default;
    [[nodiscard]] virtual std::string_view id() const noexcept = 0;
    [[nodiscard]] virtual ComponentStatus publish(
        const RobotState& state,
        const RuntimeStepContext& context) noexcept = 0;
    virtual void reset() noexcept = 0;
};

} // namespace hakoniwa::robot_runtime::runtime
