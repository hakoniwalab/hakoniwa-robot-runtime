#include "runtime/controller/plugin_controller.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {
namespace {

constexpr std::size_t error_buffer_size = 256;
constexpr std::uint64_t nanoseconds_per_microsecond = 1000;

const char* level_name(const std::int32_t level)
{
    switch (level) {
    case HAKO_PLUGIN_LOG_DEBUG:
        return "DEBUG";
    case HAKO_PLUGIN_LOG_INFO:
        return "INFO";
    case HAKO_PLUGIN_LOG_WARN:
        return "WARN";
    case HAKO_PLUGIN_LOG_ERROR:
        return "ERROR";
    default:
        return "LOG";
    }
}

/** Reads a plugin error buffer that may lack its terminating NUL. */
std::string error_text(char* buffer, const std::size_t size)
{
    buffer[size - 1] = '\0';
    if (buffer[0] == '\0') {
        return "no message";
    }
    return buffer;
}

} // namespace

PluginController::PluginController(PluginControllerOptions options)
    : controller_id_(std::move(options.controller_id))
    , library_(std::move(options.library))
    , actuator_ids_(std::move(options.actuator_ids))
    , physics_step_ns_(options.physics_step_ns)
    , joint_state_source_(options.joint_state_source)
{
    if (controller_id_.empty()) {
        throw std::invalid_argument("plugin Controller requires an ID");
    }
    if (library_ == nullptr) {
        throw std::invalid_argument(
            "plugin Controller requires a loaded plugin: " + controller_id_);
    }
    if (actuator_ids_.empty()) {
        throw std::invalid_argument(
            "plugin Controller requires joints: " + controller_id_);
    }
    std::unordered_set<std::string> unique_ids;
    for (const auto& actuator_id : actuator_ids_) {
        if (actuator_id.empty() || !unique_ids.insert(actuator_id).second) {
            throw std::invalid_argument(
                "plugin Controller joints must be non-empty and unique: "
                + controller_id_);
        }
    }
    if (physics_step_ns_ == 0) {
        throw std::invalid_argument(
            "plugin Controller requires a positive physics step: " + controller_id_);
    }
    if (joint_state_source_ != PluginJointStateSource::Truth) {
        // TODO: "observed" joint states come from the sensor stage (joint
        // sensors with noise and sample rate); not wired yet.
        throw std::invalid_argument(
            "plugin Controller joint_state_source \"observed\" is not supported yet: "
            + controller_id_);
    }

    table_ = &library_->controller_table();

    const std::size_t joint_count = actuator_ids_.size();
    position_.assign(joint_count, 0.0);
    velocity_.assign(joint_count, 0.0);
    effort_.assign(joint_count, 0.0);
    has_command_.assign(joint_count, 0);
    command_type_.assign(joint_count, 0);
    command_value_.assign(joint_count, 0.0);

    std::vector<const char*> joint_names;
    joint_names.reserve(joint_count);
    for (const auto& actuator_id : actuator_ids_) {
        joint_names.push_back(actuator_id.c_str());
    }
    const std::string config_path = options.plugin_config_path;

    HakoControllerCreateV1 args {};
    args.struct_size = sizeof(args);
    args.config_path = config_path.c_str();
    args.joint_count = static_cast<std::uint32_t>(joint_count);
    args.joint_names = joint_names.data();
    args.physics_step_ns = physics_step_ns_;
    // TODO: sensor PDUs (design 5.1). The channel list comes from the
    // component config once the sensor PDU plumbing exists.
    args.pdu_channel_count = 0;
    args.pdu_names = nullptr;
    args.pdu_types = nullptr;
    args.log = &PluginController::log_to_runtime;
    args.log_user = this;

    char error[error_buffer_size] = {};
    instance_ = table_->create(&args, error, sizeof(error));
    if (instance_ == nullptr) {
        throw std::runtime_error(
            "plugin Controller " + controller_id_ + ": create failed in "
            + library_->path() + ": " + error_text(error, sizeof(error)));
    }

    const std::uint64_t period_ns = table_->update_period_ns(instance_);
    if (period_ns == 0) {
        steps_per_update_ = 1;
    } else if (period_ns % physics_step_ns_ != 0) {
        table_->destroy(instance_);
        instance_ = nullptr;
        throw std::runtime_error(
            "plugin Controller " + controller_id_ + ": update period "
            + std::to_string(period_ns) + " ns is not a whole multiple of the physics step "
            + std::to_string(physics_step_ns_) + " ns");
    } else {
        steps_per_update_ = period_ns / physics_step_ns_;
    }

    last_status_ = {
        controller_id_,
        ComponentState::WaitingForInput,
        "no plugin update yet",
    };
    std::cerr << "[INFO] plugin Controller " << controller_id_
              << " loaded " << library_->plugin_id()
              << " from " << library_->path()
              << " (update every " << steps_per_update_ << " physics steps)"
              << std::endl;
}

PluginController::~PluginController()
{
    if (instance_ != nullptr) {
        table_->destroy(instance_);
        instance_ = nullptr;
    }
}

std::string_view PluginController::id() const noexcept
{
    return controller_id_;
}

std::string_view PluginController::source_id() const noexcept
{
    return {};
}

bool PluginController::accepts(const IControllerInput& input) const noexcept
{
    (void)input;
    return false;
}

std::uint64_t PluginController::steps_per_update() const noexcept
{
    return steps_per_update_;
}

ControllerOutput PluginController::update(
    std::shared_ptr<const IControllerInput> input,
    const RobotState& current_state,
    const RuntimeStepContext& context)
{
    (void)input;
    const std::uint64_t step_ns = context.delta_time_usec * nanoseconds_per_microsecond;
    if (step_ns != physics_step_ns_) {
        return fail(context,
            "physics step " + std::to_string(step_ns)
            + " ns differs from the step given to the plugin "
            + std::to_string(physics_step_ns_) + " ns");
    }
    if (context.step_index % steps_per_update_ != 0) {
        return repeat_last_output(context);
    }

    std::string state_error;
    if (!copy_joint_states(current_state, state_error)) {
        return fail(context, state_error);
    }
    const auto joint_count = static_cast<std::uint32_t>(actuator_ids_.size());
    const HakoJointStatesV1 states {
        joint_count,
        position_.data(),
        velocity_.data(),
        effort_.data(),
    };

    HakoTickV1 tick {};
    tick.struct_size = sizeof(tick);
    tick.sim_time_ns = context.simulation_time_usec * nanoseconds_per_microsecond;
    tick.step_index = context.step_index;
    tick.physics_step_ns = physics_step_ns_;
    const bool selected =
        context.previous_selected_control_id.has_value()
        && *context.previous_selected_control_id == controller_id_;
    tick.selected = selected ? 1 : 0;
    tick.reset = just_reset_ ? 1 : 0;

    std::fill(has_command_.begin(), has_command_.end(), std::uint8_t {0});
    std::fill(command_type_.begin(), command_type_.end(), std::uint8_t {0});
    std::fill(command_value_.begin(), command_value_.end(), 0.0);
    HakoJointCommandsV1 commands {
        joint_count,
        has_command_.data(),
        command_type_.data(),
        command_value_.data(),
    };

    // TODO: pass the sensor PDUs that changed since the last update
    // (design 5.1); the sensor PDU plumbing is a later step.
    const std::uint32_t pdu_count = 0;
    const HakoPduViewV1* pdus = nullptr;

    char error[error_buffer_size] = {};
    const std::int32_t result = table_->update(
        instance_, &tick, &states, pdu_count, pdus, &commands, error, sizeof(error));
    if (result != 0) {
        return fail(context, "plugin update failed: " + error_text(error, sizeof(error)));
    }

    std::vector<ActuatorCommand> next_commands;
    next_commands.reserve(actuator_ids_.size());
    for (std::size_t index = 0; index < actuator_ids_.size(); ++index) {
        if (has_command_[index] == 0) {
            continue;
        }
        ActuatorCommand command;
        command.actuator_id = actuator_ids_[index];
        command.value = command_value_[index];
        switch (command_type_[index]) {
        case HAKO_JOINT_COMMAND_POSITION:
            command.type = ActuatorCommandType::Position;
            break;
        case HAKO_JOINT_COMMAND_VELOCITY:
            command.type = ActuatorCommandType::Velocity;
            break;
        case HAKO_JOINT_COMMAND_EFFORT:
            command.type = ActuatorCommandType::Effort;
            break;
        default:
            return fail(context,
                "plugin returned an unknown command type "
                + std::to_string(command_type_[index])
                + " for " + actuator_ids_[index]);
        }
        next_commands.push_back(std::move(command));
    }

    just_reset_ = false;
    last_commands_ = std::move(next_commands);
    last_status_ = {controller_id_, ComponentState::Ready, {}};
    return repeat_last_output(context);
}

void PluginController::reset(const RobotState& current_state)
{
    (void)current_state;
    last_commands_.clear();
    just_reset_ = true;

    char error[error_buffer_size] = {};
    if (table_->reset(instance_, error, sizeof(error)) != 0) {
        const std::string detail =
            "plugin reset failed: " + error_text(error, sizeof(error));
        std::cerr << "[ERROR] plugin Controller " << controller_id_
                  << ": " << detail << std::endl;
        last_status_ = {controller_id_, ComponentState::Error, detail};
        return;
    }
    last_status_ = {
        controller_id_,
        ComponentState::WaitingForInput,
        "reset; waiting for the next plugin update",
    };
}

ComponentStatus PluginController::start()
{
    char error[error_buffer_size] = {};
    if (table_->on_start(instance_, error, sizeof(error)) != 0) {
        return {
            controller_id_,
            ComponentState::Error,
            "plugin on_start failed: " + error_text(error, sizeof(error)),
        };
    }
    return {controller_id_, ComponentState::Ready, {}};
}

ComponentStatus PluginController::stop()
{
    char error[error_buffer_size] = {};
    if (table_->on_stop(instance_, error, sizeof(error)) != 0) {
        return {
            controller_id_,
            ComponentState::Error,
            "plugin on_stop failed: " + error_text(error, sizeof(error)),
        };
    }
    return {controller_id_, ComponentState::Stopped, {}};
}

void PluginController::log_to_runtime(
    void* user,
    const std::int32_t level,
    const char* message)
{
    const auto* self = static_cast<const PluginController*>(user);
    std::cerr << "[" << level_name(level) << "] plugin Controller "
              << (self != nullptr ? self->controller_id_ : std::string("?"))
              << ": " << (message != nullptr ? message : "")
              << std::endl;
}

bool PluginController::copy_joint_states(
    const RobotState& current_state,
    std::string& error)
{
    // TODO: PluginJointStateSource::Observed reads the joint sensors instead.
    for (std::size_t index = 0; index < actuator_ids_.size(); ++index) {
        const auto& actuator_id = actuator_ids_[index];
        const auto state = std::find_if(
            current_state.actuators.begin(), current_state.actuators.end(),
            [&](const ActuatorState& actuator) {
                return actuator.actuator_id == actuator_id;
            });
        if (state == current_state.actuators.end()) {
            error = "state is missing actuator: " + actuator_id;
            return false;
        }
        position_[index] = state->position_rad;
        velocity_[index] = state->velocity_rad_per_sec;
        effort_[index] = state->effort_newton_metre;
    }
    return true;
}

ControllerOutput PluginController::repeat_last_output(
    const RuntimeStepContext& context) const
{
    ControllerOutput output;
    output.controller_id = controller_id_;
    output.status = last_status_;
    output.commands.reserve(last_commands_.size());
    for (const auto& command : last_commands_) {
        output.commands.push_back(stamped(command, context));
    }
    return output;
}

ControllerOutput PluginController::fail(
    const RuntimeStepContext& context,
    const std::string& detail)
{
    last_status_ = {controller_id_, ComponentState::Error, detail};
    return repeat_last_output(context);
}

ActuatorCommand PluginController::stamped(
    const ActuatorCommand& command,
    const RuntimeStepContext& context) const
{
    // A held command is valid for exactly this step, as the Hold Controller's.
    ActuatorCommand result = command;
    result.source_id.clear();
    result.created_at_usec = context.simulation_time_usec;
    result.expires_at_usec = context.simulation_time_usec + context.delta_time_usec;
    return result;
}

} // namespace hakoniwa::robot_runtime::runtime
