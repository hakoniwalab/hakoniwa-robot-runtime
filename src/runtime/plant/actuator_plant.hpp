#pragma once

#include "runtime/types.hpp"
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
#include "runtime/plant/plant_directive.hpp"
#endif

#include <cstdint>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

/**
 * Physical boundary for actuator commands, robot state, and Plant-local time.
 *
 * The physical backend is the source of truth for this Asset's simulation time
 * and fixed step duration. `read_state()` returns the current physical state
 * stamped with that backend time. `step()` guards the selected ActuatorCommands,
 * advances the backend by exactly one physical step, and returns the updated
 * state stamped with the backend's new time.
 *
 * Hakoniwa Asset timing is configured from `delta_time_usec()` at startup, so
 * the Asset-local Hakoniwa time and physical backend time advance with the same
 * fixed step. ActuatorRuntime does not maintain a second independent clock.
 *
 * The Plant owns backend binding and the final command guard because it is the
 * boundary that knows the configured actuator IDs, command types, limits, and
 * current physical state. It must not arbitrate command sources or reinterpret
 * higher-level Controller semantics.
 */
class IActuatorPlant {
public:
    virtual ~IActuatorPlant() = default;

    /** Fixed physical integration step used to configure the Hakoniwa Asset. */
    [[nodiscard]] virtual std::uint64_t delta_time_usec() const noexcept = 0;

    /** Returns the current physical state and backend simulation time. */
    [[nodiscard]] virtual RobotState read_state() const = 0;

    /** Applies selected commands and advances exactly one physical step. */
    [[nodiscard]] virtual RobotState step(
        const std::vector<ActuatorCommand>& commands) = 0;

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
    /**
     * Applies the structurally separate Plant Directive path.
     *
     * ActuatorRuntime calls this once per step after arbitration and right
     * before step(commands), so directive effects (for example a Mirror pose or
     * a geom friction change) are in place before physics advances. The list
     * mixes directive kinds; each backend applier picks only its own kind with
     * directives_of<T>() and ignores unknown kinds. The default is a no-op so
     * Plant implementations without directive support stay source-compatible.
     */
    virtual void apply_directives(const PlantDirectiveList& directives)
    {
        (void)directives;
    }
#endif

    /**
     * Resets physical state while setting the backend's actual simulation time.
     * Core reset uses zero; local Viewer reset may preserve the current Asset time.
     */
    [[nodiscard]] virtual RobotState reset(
        std::uint64_t simulation_time_usec = 0) = 0;
};

} // namespace hakoniwa::robot_runtime::runtime
