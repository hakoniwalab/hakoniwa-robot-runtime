#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

/** Physical command accepted by an actuator Plant. */
enum class ActuatorCommandType {
    Position,
    Velocity,
    Effort,
};

/**
 * Runtime-neutral actuator command.
 *
 * Position uses radians, velocity uses radians per second, and effort uses
 * newton-metres. `expires_at_usec` is expressed in simulation time and is an
 * exclusive boundary: a command is expired when step time is greater than or
 * equal to it. A missing value means that the command does not expire.
 */
struct ActuatorCommand {
    std::string actuator_id;
    ActuatorCommandType type {ActuatorCommandType::Position};
    double value {0.0};
    std::string source_id;
    std::uint64_t created_at_usec {0};
    std::optional<std::uint64_t> expires_at_usec;
};

/**
 * Immutable per-step context shared by Runtime components.
 *
 * Runtime does not maintain an independent simulation clock. The timing fields
 * are derived from the physical Plant: `simulation_time_usec` is the Plant time
 * at the beginning of the step, `delta_time_usec` is the Plant integration step,
 * and `step_index` is derived from those values for diagnostics/tests.
 *
 * Hakoniwa Asset timing is configured from the same Plant delta at startup, so
 * Asset-local Hakoniwa time and physical backend time advance on the same grid.
 * Wall-clock pacing is a separate Host concern.
 *
 * `previous_selected_control_id` and `previous_selected_commands` are feedback
 * from the previous completed arbitration step. ActuatorRuntime attaches that
 * feedback when it builds the context for the current Plant state.
 */
struct RuntimeStepContext {
    std::uint64_t simulation_time_usec {0};
    std::uint64_t delta_time_usec {0};
    std::uint64_t step_index {0};
    std::optional<std::string> previous_selected_control_id;
    std::vector<ActuatorCommand> previous_selected_commands;

    [[nodiscard]] constexpr bool is_valid() const noexcept
    {
        return delta_time_usec > 0;
    }
};

/** Scalar actuator state sampled at `simulation_time_usec`. */
struct ActuatorState {
    std::string actuator_id;
    double position_rad {0.0};
    double velocity_rad_per_sec {0.0};
    double effort_newton_metre {0.0};
    std::uint64_t simulation_time_usec {0};
};

/** Physical-state snapshot observed at the Plant's `sample_time_usec`. */
struct RobotState {
    std::vector<ActuatorState> actuators;
    std::uint64_t sample_time_usec {0};
};

/** Inclusive scalar command limits in the unit selected by command type. */
struct ActuatorLimits {
    double minimum {0.0};
    double maximum {0.0};

    [[nodiscard]] constexpr bool is_valid() const noexcept
    {
        return minimum <= maximum;
    }
};

enum class ComponentState {
    Ready,
    WaitingForInput,
    Degraded,
    Error,
    Stopped,
};

/** Runtime-visible component health; `detail` is diagnostic, not control data. */
struct ComponentStatus {
    std::string component_id;
    ComponentState state {ComponentState::Ready};
    std::string detail;

    [[nodiscard]] constexpr bool can_produce_commands() const noexcept
    {
        return state == ComponentState::Ready || state == ComponentState::Degraded;
    }
};

/** Metadata common to every controller-specific input payload. */
struct ControllerInputMetadata {
    std::string source_id;
    std::uint64_t sequence {0};
    std::uint64_t created_at_usec {0};
    std::optional<std::uint64_t> expires_at_usec;
};

struct ControllerOutput {
    std::string controller_id;
    ComponentStatus status;
    std::vector<ActuatorCommand> commands;
};

struct ArbitrationResult {
    std::vector<ActuatorCommand> selected_commands;
    std::vector<std::string> rejected_source_ids;
    std::vector<std::string> conflicting_actuator_ids;
    /** Logical Controller/control-group selected for this step, if any. */
    std::optional<std::string> selected_control_id;
};

} // namespace hakoniwa::robot_runtime::runtime
