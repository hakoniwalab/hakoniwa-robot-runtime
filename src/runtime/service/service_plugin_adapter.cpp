#include "runtime/service/service_plugin_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {
namespace {

constexpr std::uint32_t kOperationCapacity = 16;

AsyncServiceStepOutcome step_outcome(std::uint8_t result)
{
    switch (result) {
    case HAKO_SERVICE_RESULT_PENDING: return AsyncServiceStepOutcome::Pending;
    case HAKO_SERVICE_RESULT_OK: return AsyncServiceStepOutcome::Ok;
    case HAKO_SERVICE_RESULT_CANCELED: return AsyncServiceStepOutcome::Canceled;
    default: return AsyncServiceStepOutcome::Error;
    }
}

std::uint8_t service_state(AsyncServiceState state)
{
    switch (state) {
    case AsyncServiceState::Doing: return HAKO_SERVICE_STATE_DOING;
    case AsyncServiceState::Canceling: return HAKO_SERVICE_STATE_CANCELING;
    case AsyncServiceState::Idle: return HAKO_SERVICE_STATE_IDLE;
    }
    return HAKO_SERVICE_STATE_IDLE;
}

std::uint8_t command_type(ActuatorCommandType type)
{
    switch (type) {
    case ActuatorCommandType::Position: return HAKO_JOINT_COMMAND_POSITION;
    case ActuatorCommandType::Velocity: return HAKO_JOINT_COMMAND_VELOCITY;
    case ActuatorCommandType::Effort: return HAKO_JOINT_COMMAND_EFFORT;
    }
    throw std::invalid_argument("invalid actuator command type");
}

std::vector<const char*> string_views(const std::vector<std::string>& strings)
{
    std::vector<const char*> views;
    views.reserve(strings.size());
    for (const auto& string : strings) {
        views.push_back(string.c_str());
    }
    return views;
}

void log_to_runtime(void* user, std::int32_t level, const char* message)
{
    const auto& robot = *static_cast<const std::string*>(user);
    std::fprintf(stderr, "[%d] service plugin %s: %s\n", static_cast<int>(level),
        robot.c_str(), message ? message : "");
}

HakoVector3V1 to_plugin(const Vector3State& value)
{
    return {value.x, value.y, value.z};
}

HakoQuaternionV1 to_plugin(const QuaternionState& value)
{
    return {value.x, value.y, value.z, value.w};
}

Vector3State from_plugin(const HakoVector3V1& value)
{
    return {value.x, value.y, value.z};
}

bool collect_operations(const HakoRobotOutputV1& output, std::uint64_t time_usec, PlantOperations& operations)
{
    if (output.friction_count > kOperationCapacity || output.body_motion_count > kOperationCapacity
        || output.humanoid_motion_count > kOperationCapacity) {
        return false;
    }
    for (std::uint32_t i = 0; i < output.friction_count; ++i) {
        const auto& value = output.frictions[i];
        if (value.surface_count != 0 && !value.surfaces) {
            return false;
        }
        SurfaceFrictionOperation operation {{}, value.sliding_friction, time_usec};
        for (std::uint32_t j = 0; j < value.surface_count; ++j) {
            if (!value.surfaces[j]) {
                return false;
            }
            operation.surfaces.emplace_back(value.surfaces[j]);
        }
        operations.surface_frictions.push_back(std::move(operation));
    }
    for (std::uint32_t i = 0; i < output.body_motion_count; ++i) {
        const auto& value = output.body_motions[i];
        if (!value.body) {
            return false;
        }
        operations.body_motions.push_back({value.body, from_plugin(value.position),
            {value.orientation.roll, value.orientation.pitch, value.orientation.yaw},
            from_plugin(value.linear_velocity), from_plugin(value.angular_velocity), time_usec});
    }
    for (std::uint32_t i = 0; i < output.humanoid_motion_count; ++i) {
        const auto& value = output.humanoid_motions[i];
        if (!value.humanoid) {
            return false;
        }
        operations.humanoid_motions.push_back({value.humanoid,
            from_plugin(value.linear_velocity), from_plugin(value.angular_velocity),
            value.animation ? value.animation : "", value.ride ? value.ride : ""});
    }
    return true;
}

} // namespace

ServicePlugin::ServicePlugin(std::shared_ptr<plugin::PluginLibrary> library, ServicePluginConfig config)
    : library_(std::move(library)), table_(library_->service_table()), config_(std::move(config))
    , controller_id_(config_.robot + ":controller")
    , bodies_(config_.bodies.size()), humanoids_(config_.humanoids.size()), slots_(config_.services.size())
    , position_(config_.actuators.size()), velocity_(position_.size()), effort_(position_.size())
    , command_value_(position_.size()), has_command_(position_.size()), command_type_(position_.size())
{
    std::vector<HakoRobotActuatorV1> actuators;
    actuators.reserve(config_.actuators.size());
    for (const auto& actuator : config_.actuators) {
        actuators.push_back({actuator.id.c_str(), command_type(actuator.command_type),
            actuator.limits.minimum, actuator.limits.maximum});
    }
    const auto bodies = string_views(config_.bodies);
    const auto humanoids = string_views(config_.humanoids);
    std::vector<HakoRobotServiceV1> services;
    services.reserve(config_.services.size());
    for (const auto& service : config_.services) {
        services.push_back({service.name.c_str(), service.type.c_str(), static_cast<std::uint8_t>(service.lockstep)});
    }
    const HakoRobotCreateV1 args {
        sizeof(HakoRobotCreateV1), config_.robot.c_str(),
        static_cast<std::uint32_t>(actuators.size()), actuators.data(),
        static_cast<std::uint32_t>(bodies.size()), bodies.data(),
        static_cast<std::uint32_t>(humanoids.size()), humanoids.data(),
        0, nullptr,
        static_cast<std::uint32_t>(services.size()), services.data(),
        config_.config_json.c_str(), config_.physics_step_ns, log_to_runtime, &config_.robot,
    };
    std::array<char, 512> error {};
    instance_ = table_.create(&args, error.data(), error.size());
    if (!instance_) {
        error.back() = '\0';
        throw std::runtime_error("service robot " + config_.robot + " (" + library_->path()
            + ") create failed: " + (error[0] ? error.data() : "no error detail"));
    }
}

ServicePlugin::~ServicePlugin()
{
    table_.destroy(instance_);
}

bool ServicePlugin::on_request(std::size_t service, std::span<const std::byte> request)
{
    const HakoServiceRequestV1 view {request.data(), request.size()};
    return table_.on_request(instance_, static_cast<std::uint32_t>(service), &view) == 1;
}

RobotStepResult ServicePlugin::on_step(const RobotState& state, const RuntimeStepContext& context,
    std::span<RobotServiceSlot> slots)
{
    const auto fail = [&](std::string reason) -> RobotStepResult {
        for (auto& slot : slots) {
            if (slot.state != AsyncServiceState::Idle) {
                slot.result = AsyncServiceStepOutcome::Error;
            }
        }
        return {{}, std::move(reason), {}};
    };
    if (slots.size() != slots_.size()) {
        return fail("service plugin slot count does not match the robot's services");
    }
    if (auto error = copy_states(state); !error.empty()) {
        return fail(std::move(error));
    }
    for (std::size_t i = 0; i < slots.size(); ++i) {
        slots_[i] = {service_state(slots[i].state), HAKO_SERVICE_RESULT_PENDING};
    }

    const auto time = tick(context, first_step_);
    const auto states = state_view();
    std::array<HakoSurfaceFrictionV1, kOperationCapacity> frictions {};
    std::array<HakoBodyMotionV1, kOperationCapacity> body_motions {};
    std::array<HakoHumanoidMotionV1, kOperationCapacity> humanoid_motions {};
    HakoRobotOutputV1 output {
        prepare_commands(),
        kOperationCapacity, 0, frictions.data(),
        kOperationCapacity, 0, body_motions.data(),
        kOperationCapacity, 0, humanoid_motions.data(),
    };
    std::array<char, 512> error {};
    if (table_.on_step(instance_, &time, &states, slots_.data(), static_cast<std::uint32_t>(slots_.size()),
            &output, error.data(), error.size()) != 0) {
        error.back() = '\0';
        return fail(error[0] ? error.data() : "service plugin on_step failed");
    }

    RobotStepResult result;
    if (!collect_commands(context, result.commands)) {
        return fail("invalid joint command");
    }
    if (!collect_operations(output, context.simulation_time_usec, result.operations)) {
        return fail("invalid service plugin plant operations");
    }
    for (std::size_t i = 0; i < slots.size(); ++i) {
        if (slots[i].state != AsyncServiceState::Idle) {
            slots[i].result = step_outcome(slots_[i].result);
        }
    }
    return result;
}

bool ServicePlugin::on_response(std::size_t service, const RobotState& state, const RuntimeStepContext& context,
    std::span<std::byte> packet)
{
    if (!copy_states(state).empty()) {
        return false;
    }
    const auto time = tick(context, first_response_);
    const auto states = state_view();
    const HakoServiceResponseV1 view {packet.data(), packet.size()};
    return table_.on_response(instance_, static_cast<std::uint32_t>(service), &time, &states, &view) == 1;
}

void ServicePlugin::on_cancel(std::size_t service)
{
    if (table_.on_cancel) {
        table_.on_cancel(instance_, static_cast<std::uint32_t>(service));
    }
}

void ServicePlugin::reset()
{
    table_.reset(instance_);
    first_step_ = true;
    first_response_ = true;
}

std::string ServicePlugin::copy_states(const RobotState& state)
{
    for (std::size_t i = 0; i < config_.actuators.size(); ++i) {
        const auto& id = config_.actuators[i].id;
        const auto found = std::find_if(state.actuators.begin(), state.actuators.end(),
            [&](const auto& joint) { return joint.actuator_id == id; });
        if (found == state.actuators.end()) {
            return "missing joint state: " + id;
        }
        position_[i] = found->position_rad;
        velocity_[i] = found->velocity_rad_per_sec;
        effort_[i] = found->effort_newton_metre;
    }
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        const auto& name = config_.bodies[i];
        const auto found = std::find_if(state.bodies.begin(), state.bodies.end(),
            [&](const auto& body) { return body.body_id == name; });
        if (found == state.bodies.end()) {
            return "missing body state: " + name;
        }
        bodies_[i] = {to_plugin(found->position), to_plugin(found->orientation),
            to_plugin(found->linear_velocity), to_plugin(found->angular_velocity)};
    }
    for (std::size_t i = 0; i < humanoids_.size(); ++i) {
        const auto& name = config_.humanoids[i];
        const auto found = std::find_if(state.humanoids.begin(), state.humanoids.end(),
            [&](const auto& humanoid) { return humanoid.name == name; });
        if (found == state.humanoids.end()) {
            return "missing humanoid state: " + name;
        }
        humanoids_[i] = {to_plugin(found->position), to_plugin(found->orientation),
            to_plugin(found->linear_velocity), to_plugin(found->angular_velocity),
            static_cast<std::uint32_t>(found->joint_positions.size()), found->joint_positions.data(),
            found->last_motion_result};
    }
    return {};
}

HakoRobotStateV1 ServicePlugin::state_view() const
{
    return {sizeof(HakoRobotStateV1),
        {static_cast<std::uint32_t>(position_.size()), position_.data(), velocity_.data(), effort_.data()},
        static_cast<std::uint32_t>(bodies_.size()), bodies_.data(),
        static_cast<std::uint32_t>(humanoids_.size()), humanoids_.data(), 0, nullptr};
}

HakoJointCommandsV1 ServicePlugin::prepare_commands()
{
    std::fill(has_command_.begin(), has_command_.end(), 0);
    std::fill(command_type_.begin(), command_type_.end(), 0);
    std::fill(command_value_.begin(), command_value_.end(), 0);
    return {static_cast<std::uint32_t>(config_.actuators.size()), has_command_.data(),
        command_type_.data(), command_value_.data()};
}

bool ServicePlugin::collect_commands(
    const RuntimeStepContext& context, std::vector<ActuatorCommand>& output) const
{
    for (std::size_t i = 0; i < config_.actuators.size(); ++i) {
        if (!has_command_[i]) {
            continue;
        }
        ActuatorCommandType type;
        switch (command_type_[i]) {
        case HAKO_JOINT_COMMAND_POSITION:
            type = ActuatorCommandType::Position;
            break;
        case HAKO_JOINT_COMMAND_VELOCITY:
            type = ActuatorCommandType::Velocity;
            break;
        case HAKO_JOINT_COMMAND_EFFORT:
            type = ActuatorCommandType::Effort;
            break;
        default:
            return false;
        }
        if (!std::isfinite(command_value_[i])) {
            return false;
        }
        output.push_back({config_.actuators[i].id, type, command_value_[i], {},
            context.simulation_time_usec, std::nullopt});
    }
    return true;
}

HakoTickV1 ServicePlugin::tick(const RuntimeStepContext& context, bool& first_call) const
{
    return {sizeof(HakoTickV1), context.simulation_time_usec * 1000, context.step_index,
        context.delta_time_usec * 1000,
        static_cast<std::uint8_t>(context.previous_selected_control_id == controller_id_),
        static_cast<std::uint8_t>(std::exchange(first_call, false))};
}

} // namespace hakoniwa::robot_runtime::runtime
