#pragma once

#include "runtime/types.hpp"

#include <memory>
#include <string_view>

namespace hakoniwa::robot_runtime::runtime {

/**
 * Type-erased envelope for a controller-specific, strongly typed payload.
 *
 * Concrete Trajectory, Manual, and scalar inputs derive from this interface.
 * ActuatorRuntime uses only metadata and type_name; only the bound Controller
 * interprets the concrete payload. Objects returned by ICommandSource are
 * immutable and shared for the duration of one runtime step.
 */
class IControllerInput {
public:
    virtual ~IControllerInput() = default;
    [[nodiscard]] virtual const ControllerInputMetadata& metadata() const noexcept = 0;
    [[nodiscard]] virtual std::string_view type_name() const noexcept = 0;
};

struct CommandSourcePollResult {
    std::shared_ptr<const IControllerInput> input;
    ComponentStatus status;

    [[nodiscard]] bool has_new_input() const noexcept
    {
        return input != nullptr;
    }
};

/**
 * Synchronous Runtime boundary for one independent asynchronous input stream.
 *
 * One ICommandSource represents one external input channel/stream. Transport
 * reception may happen asynchronously (for example through an Endpoint receive
 * callback) and stage the latest value in a reader/mailbox. `poll()` does not
 * perform that transport receive; it non-blockingly consumes the value already
 * staged for this source and converts it into one Runtime input for the current
 * simulation step.
 *
 * No new/dirty input is represented by a null input and is not an error.
 * Implementations may retain transport state but must not combine multiple
 * independent channels or perform control, interpolation, arbitration, or
 * physical actuation. `poll()` must be deterministic with respect to simulation
 * time and must not block waiting for transport I/O.
 */
class ICommandSource {
public:
    virtual ~ICommandSource() = default;
    [[nodiscard]] virtual std::string_view id() const noexcept = 0;
    [[nodiscard]] virtual CommandSourcePollResult poll(
        const RuntimeStepContext& context) = 0;
    virtual void reset() noexcept = 0;
};

} // namespace hakoniwa::robot_runtime::runtime
