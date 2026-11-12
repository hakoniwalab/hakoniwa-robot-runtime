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
    , stop_on_error_(options.stop_on_error)
    , pdu_channels_(std::move(options.pdu_channels))
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

    std::unordered_set<std::string> unique_pdus;
    for (const auto& channel : pdu_channels_) {
        if (channel == nullptr) {
            throw std::invalid_argument(
                "plugin Controller PDU channel must not be null: " + controller_id_);
        }
        if (!unique_pdus.insert(channel->pdu_name()).second) {
            throw std::invalid_argument(
                "plugin Controller PDUs must be unique: " + channel->pdu_name());
        }
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

    std::vector<const char*> pdu_names;
    std::vector<const char*> pdu_types;
    pdu_names.reserve(pdu_channels_.size());
    pdu_types.reserve(pdu_channels_.size());
    for (const auto& channel : pdu_channels_) {
        pdu_names.push_back(channel->pdu_name().c_str());
        pdu_types.push_back(channel->pdu_type().c_str());
    }
    pdu_seen_versions_.assign(pdu_channels_.size(), 0);
    pdu_views_.reserve(pdu_channels_.size());

    HakoControllerCreateV1 args {};
    args.struct_size = sizeof(args);
    args.config_path = config_path.c_str();
    args.joint_count = static_cast<std::uint32_t>(joint_count);
    args.joint_names = joint_names.data();
    args.physics_step_ns = physics_step_ns_;
    args.pdu_channel_count = static_cast<std::uint32_t>(pdu_channels_.size());
    args.pdu_names = pdu_names.empty() ? nullptr : pdu_names.data();
    args.pdu_types = pdu_types.empty() ? nullptr : pdu_types.data();
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
              << " (update every " << steps_per_update_ << " physics steps, "
              << pdu_channels_.size() << " sensor PDUs)"
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
    if (!reset_error_.empty()) {
        return fail(context, reset_error_);
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

    collect_new_pdus();
    const auto pdu_count = static_cast<std::uint32_t>(pdu_views_.size());
    const HakoPduViewV1* pdus = pdu_views_.empty() ? nullptr : pdu_views_.data();

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
    reset_error_.clear();
    // Samples from before the reset are not passed; the writers clear
    // their channels when the publishers are reset.
    for (std::size_t index = 0; index < pdu_channels_.size(); ++index) {
        pdu_seen_versions_[index] = pdu_channels_[index]->version();
    }

    char error[error_buffer_size] = {};
    if (table_->reset(instance_, error, sizeof(error)) != 0) {
        reset_error_ = "plugin reset failed: " + error_text(error, sizeof(error));
        std::cerr << "[ERROR] plugin Controller " << controller_id_
                  << ": " << reset_error_ << std::endl;
        last_status_ = {controller_id_, ComponentState::Error, reset_error_};
        return;
    }
    last_status_ = {
        controller_id_,
        ComponentState::WaitingForInput,
        "reset; waiting for the next plugin update",
    };
}

ComponentStatus PluginController::on_start()
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

ComponentStatus PluginController::on_stop()
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

bool PluginController::error_stops_run() const noexcept
{
    return stop_on_error_;
}

void PluginController::collect_new_pdus()
{
    pdu_views_.clear();
    for (std::size_t index = 0; index < pdu_channels_.size(); ++index) {
        const auto& channel = *pdu_channels_[index];
        if (channel.version() == pdu_seen_versions_[index]) {
            continue;
        }
        pdu_seen_versions_[index] = channel.version();
        if (!channel.has_sample()) {
            continue;
        }
        HakoPduViewV1 view {};
        view.channel = static_cast<std::uint32_t>(index);
        view.size = static_cast<std::uint32_t>(channel.data().size());
        view.data = channel.data().data();
        view.sample_time_ns = channel.sample_time_usec() * nanoseconds_per_microsecond;
        pdu_views_.push_back(view);
    }
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
