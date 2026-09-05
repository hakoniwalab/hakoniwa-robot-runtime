#include "runtime/controller/manual_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {
namespace {

double shape_axis(const double value, const double deadzone, const double expo)
{
    const double magnitude = std::abs(value);
    if (magnitude <= deadzone) {
        return 0.0;
    }
    const double normalized = std::min(1.0, (magnitude - deadzone) / (1.0 - deadzone));
    const double shaped = (1.0 - expo) * normalized
        + expo * normalized * normalized * normalized;
    return std::copysign(shaped, value);
}

bool button_pressed(const JoyState& state, const std::size_t index) noexcept
{
    return index < state.buttons.size() && state.buttons[index] != 0;
}

} // namespace

ManualController::ManualController(
    std::string controller_id,
    std::string source_id,
    const std::size_t manual_enable_button_index,
    const std::size_t joy_axis_count,
    const std::size_t joy_button_count,
    const double deadzone,
    const double expo,
    std::vector<ManualActuatorBinding> actuators,
    std::vector<ManualBank> banks)
    : controller_id_(std::move(controller_id))
    , source_id_(std::move(source_id))
    , manual_enable_button_index_(manual_enable_button_index)
    , joy_axis_count_(joy_axis_count)
    , joy_button_count_(joy_button_count)
    , deadzone_(deadzone)
    , expo_(expo)
    , actuators_(std::move(actuators))
    , banks_(std::move(banks))
{
    if (controller_id_.empty() || source_id_.empty()
        || joy_axis_count_ == 0 || joy_button_count_ == 0
        || manual_enable_button_index_ >= joy_button_count_
        || actuators_.empty() || banks_.empty()) {
        throw std::invalid_argument("Manual Controller requires IDs, actuators, and banks");
    }
    if (!std::isfinite(deadzone_) || !std::isfinite(expo_)
        || deadzone_ < 0.0 || deadzone_ >= 1.0 || expo_ < 0.0 || expo_ > 1.0) {
        throw std::invalid_argument("Manual Controller deadzone/expo is invalid");
    }
    std::unordered_set<std::string> actuator_ids;
    for (const auto& actuator : actuators_) {
        if (actuator.actuator_id.empty() || !actuator.limits.is_valid()
            || !actuator_ids.insert(actuator.actuator_id).second) {
            throw std::invalid_argument("Manual Controller actuator binding is invalid");
        }
    }
    std::size_t default_banks = 0;
    std::unordered_set<std::string> bank_ids;
    for (const auto& bank : banks_) {
        if (bank.id.empty() || bank.bindings.empty() || !bank_ids.insert(bank.id).second) {
            throw std::invalid_argument("Manual Controller bank is invalid");
        }
        default_banks += bank.select_button_index.has_value() ? 0 : 1;
        for (const auto& binding : bank.bindings) {
            if (!actuator_ids.contains(binding.actuator_id)
                || !std::isfinite(binding.velocity_per_sec)
                || binding.velocity_per_sec <= 0.0) {
                throw std::invalid_argument("Manual Controller axis binding is invalid");
            }
        }
    }
    if (default_banks != 1) {
        throw std::invalid_argument("Manual Controller requires exactly one default bank");
    }
}

std::string_view ManualController::id() const noexcept
{
    return controller_id_;
}

std::string_view ManualController::source_id() const noexcept
{
    return source_id_;
}

bool ManualController::accepts(const IControllerInput& input) const noexcept
{
    return input.type_name() == "sensor_msgs/Joy";
}

const ManualBank* ManualController::select_bank(
    const JoyState& state, std::string& error) const
{
    const ManualBank* selected = nullptr;
    const ManualBank* default_bank = nullptr;
    for (const auto& bank : banks_) {
        if (!bank.select_button_index.has_value()) {
            default_bank = &bank;
        } else if (button_pressed(state, *bank.select_button_index)) {
            if (selected != nullptr) {
                error = "multiple Manual banks are selected";
                return nullptr;
            }
            selected = &bank;
        }
    }
    return selected != nullptr ? selected : default_bank;
}

std::unordered_map<std::string, double> ManualController::measured_targets(
    const RobotState& state) const
{
    std::unordered_map<std::string, double> measured;
    measured.reserve(actuators_.size());
    for (const auto& binding : actuators_) {
        const auto found = std::find_if(
            state.actuators.begin(), state.actuators.end(),
            [&](const ActuatorState& item) {
                return item.actuator_id == binding.actuator_id;
            });
        if (found == state.actuators.end()) {
            throw std::runtime_error(
                "Manual Controller state is missing actuator: " + binding.actuator_id);
        }
        measured.emplace(binding.actuator_id, found->position_rad);
    }
    return measured;
}

std::vector<ActuatorCommand> ManualController::target_commands(
    const RuntimeStepContext& context) const
{
    std::vector<ActuatorCommand> commands;
    commands.reserve(actuators_.size());
    for (const auto& actuator : actuators_) {
        commands.push_back({
            actuator.actuator_id,
            ActuatorCommandType::Position,
            targets_.at(actuator.actuator_id),
            source_id_,
            context.simulation_time_usec,
            context.simulation_time_usec + context.delta_time_usec,
        });
    }
    return commands;
}

ControllerOutput ManualController::update(
    std::shared_ptr<const IControllerInput> input,
    const RobotState& current_state,
    const RuntimeStepContext& context)
{
    ComponentStatus status {controller_id_, ComponentState::WaitingForInput, {}};
    if (input != nullptr) {
        const auto joy = std::dynamic_pointer_cast<const JoyInput>(input);
        if (joy == nullptr || joy->metadata().source_id != source_id_) {
            return {
                controller_id_,
                {controller_id_, ComponentState::Error, "invalid Joy controller input"},
                {},
            };
        }
        const auto& joy_state = joy->state();
        const bool valid_layout = joy_state.axes.size() == joy_axis_count_
            && joy_state.buttons.size() == joy_button_count_
            && std::all_of(joy_state.axes.begin(), joy_state.axes.end(),
                [](const float value) { return std::isfinite(value); });
        if (!valid_layout) {
            return {
                controller_id_,
                {controller_id_, ComponentState::Degraded,
                    "Joy arrays do not match the configured logical layout"},
                {},
            };
        }
        latest_state_ = joy->state();
        latest_expires_at_usec_ = joy->metadata().expires_at_usec;
        status.state = ComponentState::Ready;
    }

    const bool input_is_current = latest_state_.has_value()
        && latest_expires_at_usec_.has_value()
        && context.simulation_time_usec < *latest_expires_at_usec_;
    const bool manual_enable_pressed = input_is_current
        && button_pressed(*latest_state_, manual_enable_button_index_);
    if (!manual_enable_pressed) {
        return {controller_id_, std::move(status), {}};
    }

    const bool continuing_manual =
        context.previous_selected_control_id.has_value()
        && *context.previous_selected_control_id == controller_id_;
    if (!continuing_manual || targets_.size() != actuators_.size()) {
        targets_ = measured_targets(current_state);
        if (context.previous_selected_control_id.has_value()) {
            for (const auto& command : context.previous_selected_commands) {
                if (command.type != ActuatorCommandType::Position
                    || !std::isfinite(command.value)) {
                    continue;
                }
                const auto actuator = std::find_if(
                    actuators_.begin(), actuators_.end(),
                    [&](const ManualActuatorBinding& item) {
                        return item.actuator_id == command.actuator_id;
                    });
                if (actuator == actuators_.end()) {
                    continue;
                }
                targets_[command.actuator_id] = std::clamp(
                    command.value,
                    actuator->limits.minimum,
                    actuator->limits.maximum);
            }
        }
    }

    std::string bank_error;
    const auto* bank = select_bank(*latest_state_, bank_error);
    if (bank == nullptr) {
        return {
            controller_id_,
            {controller_id_, ComponentState::Degraded, std::move(bank_error)},
            {},
        };
    }

    const double delta_sec = static_cast<double>(context.delta_time_usec) / 1'000'000.0;
    for (const auto& binding : bank->bindings) {
        const double raw = binding.axis_index < latest_state_->axes.size()
            ? latest_state_->axes[binding.axis_index]
            : 0.0;
        const double direction = binding.invert ? -1.0 : 1.0;
        const auto actuator = std::find_if(
            actuators_.begin(), actuators_.end(),
            [&](const ManualActuatorBinding& item) {
                return item.actuator_id == binding.actuator_id;
            });
        const double next = targets_.at(binding.actuator_id)
            + shape_axis(raw, deadzone_, expo_)
                * binding.velocity_per_sec * direction * delta_sec;
        targets_[binding.actuator_id] = std::clamp(
            next, actuator->limits.minimum, actuator->limits.maximum);
    }
    status.state = ComponentState::Ready;
    return {controller_id_, std::move(status), target_commands(context)};
}

void ManualController::reset(const RobotState& current_state)
{
    latest_state_.reset();
    latest_expires_at_usec_.reset();
    targets_ = measured_targets(current_state);
}

} // namespace hakoniwa::robot_runtime::runtime
