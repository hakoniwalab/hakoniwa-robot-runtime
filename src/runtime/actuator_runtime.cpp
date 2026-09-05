#include "runtime/actuator_runtime.hpp"

#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {
namespace {

template <typename T>
void require_non_null(const std::shared_ptr<T>& value, const char* name)
{
    if (value == nullptr) {
        throw std::invalid_argument(std::string(name) + " must not be null");
    }
}

bool input_is_active(
    const ControllerInputMetadata& metadata,
    const RuntimeStepContext& context) noexcept
{
    return metadata.created_at_usec <= context.simulation_time_usec
        && (!metadata.expires_at_usec.has_value()
            || context.simulation_time_usec < *metadata.expires_at_usec);
}

} // namespace

ActuatorRuntime::ActuatorRuntime(
    std::vector<std::shared_ptr<ICommandSource>> sources,
    std::vector<std::shared_ptr<IController>> controllers,
    std::shared_ptr<ICommandArbiter> arbiter,
    std::shared_ptr<IActuatorPlant> plant,
    std::vector<std::shared_ptr<IStatePublisher>> publishers)
    : sources_(std::move(sources))
    , controllers_(std::move(controllers))
    , arbiter_(std::move(arbiter))
    , plant_(std::move(plant))
    , publishers_(std::move(publishers))
{
    require_non_null(arbiter_, "command arbiter");
    require_non_null(plant_, "actuator plant");
    if (plant_->delta_time_usec() == 0) {
        throw std::invalid_argument("actuator plant delta time must be greater than zero");
    }

    std::unordered_set<std::string_view> source_ids;
    source_ids.reserve(sources_.size());
    for (const auto& source : sources_) {
        require_non_null(source, "command source");
        if (source->id().empty() || !source_ids.emplace(source->id()).second) {
            throw std::invalid_argument("command source IDs must be non-empty and unique");
        }
    }

    std::unordered_set<std::string_view> controller_ids;
    controller_ids.reserve(controllers_.size());
    for (const auto& controller : controllers_) {
        require_non_null(controller, "controller");
        if (controller->id().empty()
            || !controller_ids.emplace(controller->id()).second) {
            throw std::invalid_argument("controller IDs must be non-empty and unique");
        }
        if (!controller->source_id().empty()
            && !source_ids.contains(controller->source_id())) {
            throw std::invalid_argument(
                "controller references an unknown command source: "
                + std::string(controller->source_id()));
        }
    }

    std::unordered_set<std::string_view> publisher_ids;
    publisher_ids.reserve(publishers_.size());
    for (const auto& publisher : publishers_) {
        require_non_null(publisher, "state publisher");
        if (publisher->id().empty()
            || !publisher_ids.emplace(publisher->id()).second) {
            throw std::invalid_argument("publisher IDs must be non-empty and unique");
        }
    }
}

RobotState ActuatorRuntime::read_current_state() const
{
    return plant_->read_state();
}

RuntimeStepContext ActuatorRuntime::prepare_step_context(
    const RobotState& current_state) const
{
    RuntimeStepContext context;
    context.simulation_time_usec = current_state.sample_time_usec;
    context.delta_time_usec = plant_->delta_time_usec();
    context.step_index = context.simulation_time_usec / context.delta_time_usec;
    context.previous_selected_control_id = previous_selected_control_id_;
    context.previous_selected_commands = previous_selected_commands_;
    return context;
}

ActuatorRuntime::ControllerInputMap ActuatorRuntime::poll_sources(
    const RuntimeStepContext& context,
    std::vector<ComponentStatus>& statuses)
{
    ControllerInputMap inputs;
    inputs.reserve(sources_.size());
    statuses.reserve(sources_.size());

    for (const auto& source : sources_) {
        auto polled = source->poll(context);
        if (polled.input != nullptr) {
            const auto& metadata = polled.input->metadata();
            if (metadata.source_id != source->id()) {
                throw std::runtime_error(
                    "command source returned input with a different source ID");
            }
            if (!input_is_active(metadata, context)) {
                polled.input.reset();
                polled.status.state = ComponentState::Degraded;
                polled.status.detail = "input is expired or from future simulation time";
            }
        }
        inputs.emplace(source->id(), polled.input);
        statuses.push_back(std::move(polled.status));
    }
    return inputs;
}

std::vector<ControllerOutput> ActuatorRuntime::update_controllers(
    const ControllerInputMap& inputs,
    const RobotState& current_state,
    const RuntimeStepContext& context,
    std::vector<ComponentStatus>& statuses)
{
    std::vector<ControllerOutput> outputs;
    outputs.reserve(controllers_.size());
    statuses.reserve(controllers_.size());

    for (const auto& controller : controllers_) {
        std::shared_ptr<const IControllerInput> input;
        if (!controller->source_id().empty()) {
            input = inputs.at(controller->source_id());
            if (input != nullptr && !controller->accepts(*input)) {
                input.reset();
            }
        }

        auto output = controller->update(input, current_state, context);
        if (output.controller_id != controller->id()) {
            throw std::runtime_error("controller output ID does not match controller ID");
        }
        for (const auto& command : output.commands) {
            if (command.source_id != controller->source_id()) {
                throw std::runtime_error(
                    "controller produced a command for a different source ID");
            }
        }
        statuses.push_back(output.status);
        outputs.push_back(std::move(output));
    }
    return outputs;
}

RobotState ActuatorRuntime::step_plant(
    const std::vector<ActuatorCommand>& commands)
{
    return plant_->step(commands);
}

RuntimeStepContext ActuatorRuntime::complete_step(
    const RobotState& next_state,
    const ArbitrationResult& arbitration)
{
    previous_selected_control_id_ = arbitration.selected_control_id;
    previous_selected_commands_ = arbitration.selected_commands;
    return prepare_step_context(next_state);
}

void ActuatorRuntime::publish_state(
    const RobotState& state,
    const RuntimeStepContext& context,
    std::vector<ComponentStatus>& statuses)
{
    statuses.reserve(publishers_.size());
    for (const auto& publisher : publishers_) {
        statuses.push_back(publisher->publish(state, context));
    }
}

RuntimeStepReport ActuatorRuntime::step()
{
    RuntimeStepReport report;

    // 1. Observe the current physical state and Plant-local simulation time.
    // The physical backend is the Asset-local time source of truth.
    const auto current_state = read_current_state();

    // 2. Build this step's context from Plant time plus previous control feedback.
    const auto context = prepare_step_context(current_state);
    report.completed_step = context;

    // 3. Consume the latest received value from each independent input channel.
    // Endpoint receive callbacks run asynchronously and stage the latest value;
    // poll_sources() only transfers that staged value into this Runtime step.
    const auto inputs = poll_sources(context, report.source_statuses);

    // 4. Let every Controller generate its candidate ActuatorCommands.
    const auto outputs = update_controllers(
        inputs, current_state, context, report.controller_statuses);

    // 5. Select one logical control strategy for this step.
    report.arbitration = arbiter_->arbitrate(outputs, current_state, context);

    // 6. Guard and apply the selected commands, then advance MuJoCo one step.
    const auto next_state = step_plant(report.arbitration.selected_commands);

    // 7. Carry selection feedback forward. The next time comes from Plant state.
    report.next_step = complete_step(next_state, report.arbitration);

    // 8. Publish the updated physical state using the Plant-derived context.
    publish_state(next_state, report.next_step, report.publisher_statuses);

    return report;
}

RobotState ActuatorRuntime::reset(const std::uint64_t simulation_time_usec)
{
    previous_selected_control_id_.reset();
    previous_selected_commands_.clear();
    auto state = plant_->reset(simulation_time_usec);
    for (const auto& source : sources_) {
        source->reset();
    }
    for (const auto& controller : controllers_) {
        controller->reset(state);
    }
    for (const auto& publisher : publishers_) {
        publisher->reset();
    }
    return state;
}

} // namespace hakoniwa::robot_runtime::runtime
