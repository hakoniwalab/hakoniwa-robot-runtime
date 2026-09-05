#include "runtime/controller/scalar_pdu_command_controller.hpp"

#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

ScalarPduCommandController::ScalarPduCommandController(
    std::string controller_id,
    std::string source_id,
    std::string actuator_id,
    const ActuatorCommandType command_type)
    : controller_id_(std::move(controller_id))
    , source_id_(std::move(source_id))
    , actuator_id_(std::move(actuator_id))
    , command_type_(command_type)
{
    if (controller_id_.empty() || source_id_.empty() || actuator_id_.empty()) {
        throw std::invalid_argument(
            "scalar PDU controller IDs must not be empty");
    }
}

std::string_view ScalarPduCommandController::id() const noexcept
{
    return controller_id_;
}

std::string_view ScalarPduCommandController::source_id() const noexcept
{
    return source_id_;
}

bool ScalarPduCommandController::accepts(
    const IControllerInput& input) const noexcept
{
    return input.type_name() == "std_msgs/Float64";
}

ControllerOutput ScalarPduCommandController::update(
    std::shared_ptr<const IControllerInput> input,
    const RobotState& current_state,
    const RuntimeStepContext& context)
{
    (void)current_state;
    ControllerOutput output {
        controller_id_,
        {controller_id_, ComponentState::WaitingForInput, {}},
        {},
    };
    if (input != nullptr) {
        const auto scalar = std::dynamic_pointer_cast<const ScalarPduInput>(input);
        if (scalar == nullptr || scalar->metadata().source_id != source_id_) {
            output.status = {controller_id_, ComponentState::Error,
                "invalid scalar PDU controller input"};
            return output;
        }
        active_metadata_ = scalar->metadata();
        active_value_ = scalar->value();
    }
    if (!active_metadata_.has_value()
        || active_metadata_->created_at_usec > context.simulation_time_usec
        || (active_metadata_->expires_at_usec.has_value()
            && context.simulation_time_usec >= *active_metadata_->expires_at_usec)) {
        active_metadata_.reset();
        return output;
    }
    const auto& metadata = *active_metadata_;
    output.status.state = ComponentState::Ready;
    output.commands.push_back({
        actuator_id_,
        command_type_,
        active_value_,
        source_id_,
        metadata.created_at_usec,
        metadata.expires_at_usec,
    });
    return output;
}

void ScalarPduCommandController::reset(const RobotState& current_state)
{
    (void)current_state;
    active_metadata_.reset();
    active_value_ = 0.0;
}

} // namespace hakoniwa::robot_runtime::runtime
