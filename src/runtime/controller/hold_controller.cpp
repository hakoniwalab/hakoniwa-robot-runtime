#include "runtime/controller/hold_controller.hpp"

#include <algorithm>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

HoldController::HoldController(
    std::string controller_id,
    std::vector<HoldActuatorBinding> actuators)
    : controller_id_(std::move(controller_id))
    , actuators_(std::move(actuators))
{
    if (controller_id_.empty() || actuators_.empty()) {
        throw std::invalid_argument("Hold Controller requires an ID and actuators");
    }
    std::unordered_set<std::string> actuator_ids;
    for (const auto& actuator : actuators_) {
        if (actuator.actuator_id.empty()
            || !actuator_ids.insert(actuator.actuator_id).second) {
            throw std::invalid_argument(
                "Hold Controller actuator IDs must be unique");
        }
    }
}

std::string_view HoldController::id() const noexcept
{
    return controller_id_;
}

std::string_view HoldController::source_id() const noexcept
{
    return {};
}

bool HoldController::accepts(const IControllerInput& input) const noexcept
{
    (void)input;
    return false;
}

ControllerOutput HoldController::update(
    std::shared_ptr<const IControllerInput> input,
    const RobotState& current_state,
    const RuntimeStepContext& context)
{
    (void)input;
    const bool continuing_hold =
        context.previous_selected_control_id.has_value()
        && *context.previous_selected_control_id == controller_id_;
    if (!continuing_hold || targets_.size() != actuators_.size()) {
        capture_targets(
            current_state,
            context.previous_selected_commands);
    }

    std::vector<ActuatorCommand> commands;
    commands.reserve(actuators_.size());
    for (const auto& binding : actuators_) {
        commands.push_back({
            binding.actuator_id,
            ActuatorCommandType::Position,
            targets_.at(binding.actuator_id),
            {},
            context.simulation_time_usec,
            context.simulation_time_usec + context.delta_time_usec,
        });
    }
    return {
        controller_id_,
        {controller_id_, ComponentState::Ready, {}},
        std::move(commands),
    };
}

void HoldController::reset(const RobotState& current_state)
{
    capture_targets(current_state, {});
}

void HoldController::capture_targets(
    const RobotState& current_state,
    const std::vector<ActuatorCommand>& previous_commands)
{
    std::unordered_map<std::string, double> next_targets;
    next_targets.reserve(actuators_.size());
    for (const auto& binding : actuators_) {
        const auto previous = std::find_if(
            previous_commands.begin(), previous_commands.end(),
            [&](const ActuatorCommand& command) {
                return command.actuator_id == binding.actuator_id
                    && command.type == ActuatorCommandType::Position;
            });
        if (previous != previous_commands.end()) {
            next_targets.emplace(binding.actuator_id, previous->value);
            continue;
        }

        const auto state = std::find_if(
            current_state.actuators.begin(), current_state.actuators.end(),
            [&](const ActuatorState& actuator) {
                return actuator.actuator_id == binding.actuator_id;
            });
        if (state == current_state.actuators.end()) {
            throw std::runtime_error(
                "Hold Controller state is missing actuator: "
                + binding.actuator_id);
        }
        next_targets.emplace(binding.actuator_id, state->position_rad);
    }
    targets_ = std::move(next_targets);
}

} // namespace hakoniwa::robot_runtime::runtime
