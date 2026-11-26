#include "runtime/service/service_plugin_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {
namespace {

AsyncServiceStepOutcome step_outcome(int32_t result)
{
    switch (result) {
    case HAKO_SERVICE_STEP_PENDING: return AsyncServiceStepOutcome::Pending;
    case HAKO_SERVICE_STEP_OK: return AsyncServiceStepOutcome::Ok;
    case HAKO_SERVICE_STEP_CANCELED: return AsyncServiceStepOutcome::Canceled;
    default: return AsyncServiceStepOutcome::Error;
    }
}

uint32_t service_state(AsyncServiceState state)
{
    switch (state) {
    case AsyncServiceState::Doing: return HAKO_SERVICE_STATE_DOING;
    case AsyncServiceState::Canceling: return HAKO_SERVICE_STATE_CANCELING;
    case AsyncServiceState::Idle: return HAKO_SERVICE_STATE_IDLE;
    }
    return HAKO_SERVICE_STATE_IDLE;
}

} // namespace

ServicePlugin::ServicePlugin(std::shared_ptr<plugin::PluginLibrary> library, std::string service_name,
    const std::string& config_json, std::vector<std::string> actuator_ids,
    std::uint64_t physics_step_ns, std::size_t metadata_size, std::size_t header_size)
    : library_(std::move(library)), table_(library_->service_table())
    , service_name_(std::move(service_name)), controller_id_(service_name_ + ":controller")
    , actuator_ids_(std::move(actuator_ids)), metadata_size_(metadata_size), header_size_(header_size)
    , position_(actuator_ids_.size()), velocity_(position_.size()), effort_(position_.size())
    , command_value_(position_.size()), has_command_(position_.size()), command_type_(position_.size())
{
    std::vector<const char*> names;
    names.reserve(actuator_ids_.size());
    for (const auto& id : actuator_ids_) {
        names.push_back(id.c_str());
    }
    const HakoServiceCreateV1 args {
        sizeof(HakoServiceCreateV1), service_name_.c_str(), config_json.c_str(),
        static_cast<uint32_t>(names.size()), names.data(), physics_step_ns,
    };
    std::array<char, 512> error {};
    instance_ = table_.create(&args, error.data(), error.size());
    if (!instance_) {
        error.back() = '\0';
        throw std::runtime_error("service " + service_name_ + " (" + library_->path()
            + ") create failed: " + (error[0] ? error.data() : "no error detail"));
    }
}

ServicePlugin::~ServicePlugin()
{
    table_.destroy(instance_);
}

bool ServicePlugin::on_request(std::span<const std::byte> request)
{
    return table_.on_request(instance_, request.data(), request.size()) == 1;
}

AsyncServiceStepResult ServicePlugin::on_step(const RobotState& state, AsyncServiceState server_state,
    const RuntimeStepContext& context)
{
    if (!copy_states(state)) {
        return {AsyncServiceStepOutcome::Error, {}};
    }
    const auto time = tick(context, first_step_);
    const HakoJointStatesV1 states {
        static_cast<uint32_t>(position_.size()), position_.data(), velocity_.data(), effort_.data(),
    };
    auto commands = prepare_commands();
    const auto outcome = step_outcome(
        table_.on_step(instance_, &time, &states, service_state(server_state), &commands));
    AsyncServiceStepResult result {outcome, {}};
    if (outcome != AsyncServiceStepOutcome::Error && !collect_commands(context, result.commands)) {
        return {AsyncServiceStepOutcome::Error, {}};
    }
    return result;
}

bool ServicePlugin::on_response(const RobotState& state, const RuntimeStepContext& context,
    std::span<std::byte> packet)
{
    if (packet.size() < metadata_size_ || packet.size() - metadata_size_ < header_size_
        || !copy_states(state)) {
        return false;
    }
    const auto time = tick(context, first_response_);
    const HakoJointStatesV1 states {
        static_cast<uint32_t>(position_.size()), position_.data(), velocity_.data(), effort_.data(),
    };
    const auto payload = packet.subspan(metadata_size_ + header_size_);
    const HakoServiceReplyV1 reply {
        packet.data(), metadata_size_, packet.data() + metadata_size_, header_size_,
        payload.data(), payload.size(),
    };
    return table_.on_response(instance_, &time, &states, &reply) == 1;
}

void ServicePlugin::on_cancel()
{
    if (table_.on_cancel) {
        table_.on_cancel(instance_);
    }
}

void ServicePlugin::reset()
{
    table_.reset(instance_);
    first_step_ = first_response_ = first_update_ = true;
}

bool ServicePlugin::has_update() const noexcept
{
    return table_.update != nullptr;
}

ControllerOutput ServicePlugin::update(const RobotState& state, const RuntimeStepContext& context)
{
    ControllerOutput output {controller_id_, {controller_id_, ComponentState::Ready, {}}, {}};
    if (!copy_states(state)) {
        output.status = {controller_id_, ComponentState::Error, "missing joint state"};
        return output;
    }
    const auto time = tick(context, first_update_);
    const HakoJointStatesV1 states {
        static_cast<uint32_t>(position_.size()), position_.data(), velocity_.data(), effort_.data(),
    };
    auto commands = prepare_commands();
    std::array<char, 512> error {};
    if (table_.update(instance_, &time, &states, 0, nullptr, &commands, error.data(), error.size()) != 0) {
        error.back() = '\0';
        output.status = {controller_id_, ComponentState::Error, error.data()};
    } else if (!collect_commands(context, output.commands)) {
        output.status = {controller_id_, ComponentState::Error, "invalid joint command"};
        output.commands.clear();
    }
    return output;
}

bool ServicePlugin::copy_states(const RobotState& state)
{
    for (std::size_t i = 0; i < actuator_ids_.size(); ++i) {
        const auto found = std::find_if(state.actuators.begin(), state.actuators.end(),
            [&](const auto& joint) { return joint.actuator_id == actuator_ids_[i]; });
        if (found == state.actuators.end()) {
            return false;
        }
        position_[i] = found->position_rad;
        velocity_[i] = found->velocity_rad_per_sec;
        effort_[i] = found->effort_newton_metre;
    }
    return true;
}

HakoJointCommandsV1 ServicePlugin::prepare_commands()
{
    std::fill(has_command_.begin(), has_command_.end(), 0);
    std::fill(command_type_.begin(), command_type_.end(), 0);
    std::fill(command_value_.begin(), command_value_.end(), 0);
    return {static_cast<uint32_t>(actuator_ids_.size()), has_command_.data(),
        command_type_.data(), command_value_.data()};
}

bool ServicePlugin::collect_commands(
    const RuntimeStepContext& context, std::vector<ActuatorCommand>& output) const
{
    for (std::size_t i = 0; i < actuator_ids_.size(); ++i) {
        if (!has_command_[i]) {
            continue;
        }
        ActuatorCommandType type;
        switch (command_type_[i]) {
        case HAKO_JOINT_COMMAND_POSITION: type = ActuatorCommandType::Position; break;
        case HAKO_JOINT_COMMAND_VELOCITY: type = ActuatorCommandType::Velocity; break;
        case HAKO_JOINT_COMMAND_EFFORT: type = ActuatorCommandType::Effort; break;
        default: return false;
        }
        if (!std::isfinite(command_value_[i])) {
            return false;
        }
        output.push_back({actuator_ids_[i], type, command_value_[i], service_name_,
            context.simulation_time_usec, std::nullopt});
    }
    return true;
}

HakoTickV1 ServicePlugin::tick(const RuntimeStepContext& context, bool& first_call) const
{
    return {sizeof(HakoTickV1), context.simulation_time_usec * 1000, context.step_index,
        context.delta_time_usec * 1000,
        static_cast<uint8_t>(context.previous_selected_control_id == controller_id_),
        static_cast<uint8_t>(std::exchange(first_call, false))};
}

ServicePluginController::ServicePluginController(
    std::shared_ptr<IController> rpc, std::shared_ptr<ServicePlugin> plugin)
    : rpc_(std::move(rpc)), plugin_(std::move(plugin))
{
}

ControllerOutput ServicePluginController::update(std::shared_ptr<const IControllerInput> input,
    const RobotState& state, const RuntimeStepContext& context)
{
    auto rpc = rpc_->update(std::move(input), state, context);
    auto periodic = plugin_->update(state, context);
    if (rpc.status.state == ComponentState::Error) {
        return rpc;
    }
    if (periodic.status.state == ComponentState::Error) {
        if (rpc.commands.empty()) {
            return periodic;
        }
        rpc.status = {rpc.controller_id, ComponentState::Degraded, std::move(periodic.status.detail)};
        return rpc;
    }

    for (auto& command : periodic.commands) {
        const auto overlaps_rpc = [&](const auto& candidate) {
            return candidate.actuator_id == command.actuator_id;
        };
        if (std::none_of(rpc.commands.begin(), rpc.commands.end(), overlaps_rpc)) {
            rpc.commands.push_back(std::move(command));
        }
    }
    if (!rpc.commands.empty()) {
        rpc.status.state = ComponentState::Ready;
    }
    return rpc;
}

} // namespace hakoniwa::robot_runtime::runtime
