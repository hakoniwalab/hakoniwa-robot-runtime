#pragma once

#include "runtime/controller/controller.hpp"
#include "runtime/source/joy_command_source.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

struct ManualActuatorBinding {
    std::string actuator_id;
    ActuatorLimits limits;
};

struct ManualAxisBinding {
    std::size_t axis_index {0};
    std::string actuator_id;
    double velocity_per_sec {0.0};
    bool invert {false};
};

struct ManualBank {
    std::string id;
    std::optional<std::size_t> select_button_index;
    std::vector<ManualAxisBinding> bindings;
};

/**
 * Interprets logical Joy input and generates Manual actuator candidates while
 * Manual Enable is active.
 *
 * When Manual takes control after another Controller, it seeds its position
 * targets from the previous selected command set in RuntimeStepContext. This
 * preserves command continuity without sharing mode/ownership state with any
 * other Controller. Missing/non-position handoff commands fall back to the
 * measured RobotState for the corresponding actuator.
 */
class ManualController final : public IController {
public:
    ManualController(
        std::string controller_id,
        std::string source_id,
        std::size_t manual_enable_button_index,
        std::size_t joy_axis_count,
        std::size_t joy_button_count,
        double deadzone,
        double expo,
        std::vector<ManualActuatorBinding> actuators,
        std::vector<ManualBank> banks);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] std::string_view source_id() const noexcept override;
    [[nodiscard]] bool accepts(const IControllerInput& input) const noexcept override;
    [[nodiscard]] ControllerOutput update(
        std::shared_ptr<const IControllerInput> input,
        const RobotState& current_state,
        const RuntimeStepContext& context) override;
    void reset(const RobotState& current_state) override;

private:
    [[nodiscard]] const ManualBank* select_bank(
        const JoyState& state, std::string& error) const;
    [[nodiscard]] std::unordered_map<std::string, double> measured_targets(
        const RobotState& state) const;
    [[nodiscard]] std::vector<ActuatorCommand> target_commands(
        const RuntimeStepContext& context) const;

    std::string controller_id_;
    std::string source_id_;
    std::size_t manual_enable_button_index_ {0};
    std::size_t joy_axis_count_ {0};
    std::size_t joy_button_count_ {0};
    double deadzone_ {0.0};
    double expo_ {0.0};
    std::vector<ManualActuatorBinding> actuators_;
    std::vector<ManualBank> banks_;
    std::optional<JoyState> latest_state_;
    std::optional<std::uint64_t> latest_expires_at_usec_;
    std::unordered_map<std::string, double> targets_;
};

} // namespace hakoniwa::robot_runtime::runtime
