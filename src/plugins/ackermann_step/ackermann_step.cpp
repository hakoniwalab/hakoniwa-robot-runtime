#include "plugins/ackermann_step/ackermann_step.hpp"
#include "plugins/packet_metadata.hpp"

#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepResponsePacket.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::plugins::ackermann_step {
namespace {

struct Geometry {
    double wheelbase_m;
    double track_width_m;
    double wheel_radius_m;
    double max_steering_angle_rad;
    double max_wheel_angular_velocity_rad_s;
};

struct Targets {
    double steering_left {0.0};
    double steering_right {0.0};
    double drive_left {0.0};
    double drive_right {0.0};
};

Targets resolve_targets(
    const AckermannDriveCommand& command,
    const Geometry& geometry)
{
    const double delta = std::clamp(
        command.steering_angle_rad,
        -geometry.max_steering_angle_rad,
        geometry.max_steering_angle_rad);
    const double base_wheel_velocity = std::clamp(
        command.speed_m_s / geometry.wheel_radius_m,
        -geometry.max_wheel_angular_velocity_rad_s,
        geometry.max_wheel_angular_velocity_rad_s);

    Targets targets {};
    targets.drive_left = base_wheel_velocity;
    targets.drive_right = base_wheel_velocity;
    if (std::abs(delta) < 1.0e-9) {
        return targets;
    }

    const double radius = geometry.wheelbase_m / std::tan(delta);
    targets.steering_left = std::atan2(
        geometry.wheelbase_m,
        radius - geometry.track_width_m / 2.0);
    targets.steering_right = std::atan2(
        geometry.wheelbase_m,
        radius + geometry.track_width_m / 2.0);
    if (delta < 0.0) {
        if (targets.steering_left > 0.0) {
            targets.steering_left -= std::numbers::pi;
        }
        if (targets.steering_right > 0.0) {
            targets.steering_right -= std::numbers::pi;
        }
    }
    targets.steering_left = std::clamp(
        targets.steering_left,
        -geometry.max_steering_angle_rad,
        geometry.max_steering_angle_rad);
    targets.steering_right = std::clamp(
        targets.steering_right,
        -geometry.max_steering_angle_rad,
        geometry.max_steering_angle_rad);

    const double left_scale = 1.0 - geometry.track_width_m / (2.0 * radius);
    const double right_scale = 1.0 + geometry.track_width_m / (2.0 * radius);
    targets.drive_left = std::clamp(
        base_wheel_velocity * left_scale,
        -geometry.max_wheel_angular_velocity_rad_s,
        geometry.max_wheel_angular_velocity_rad_s);
    targets.drive_right = std::clamp(
        base_wheel_velocity * right_scale,
        -geometry.max_wheel_angular_velocity_rad_s,
        geometry.max_wheel_angular_velocity_rad_s);
    return targets;
}

HakoCpp_AckermannStepResponse response_body(
    std::uint64_t step, std::uint64_t time_usec, const HakoBodyStateV1& state)
{
    HakoCpp_AckermannStepResponse body {};
    body.step = step;
    body.result_code = 0;
    body.sim_time_usec = time_usec;
    body.pose.translation.x = state.position.x;
    body.pose.translation.y = state.position.y;
    body.pose.translation.z = state.position.z;
    body.pose.rotation.x = state.orientation.x;
    body.pose.rotation.y = state.orientation.y;
    body.pose.rotation.z = state.orientation.z;
    body.pose.rotation.w = state.orientation.w;
    body.twist.linear.x = state.linear_velocity.x;
    body.twist.linear.y = state.linear_velocity.y;
    body.twist.linear.z = state.linear_velocity.z;
    body.twist.angular.x = state.angular_velocity.x;
    body.twist.angular.y = state.angular_velocity.y;
    body.twist.angular.z = state.angular_velocity.z;
    return body;
}

bool encode_response(std::uint64_t step, std::uint64_t time_usec, const HakoBodyStateV1& state,
    std::span<std::byte> packet)
{
    // Core initializes only the response header inside a buffer sized for the
    // full packet. A complete AckermannStep response has no dynamic heap.
    constexpr auto full_size = HAKO_PDU_META_DATA_SIZE()
        + HAKO_ALIGN_SIZE(sizeof(Hako_AckermannStepResponsePacket), HAKO_ALIGNMENT_SIZE);
    auto metadata = packet_metadata(packet, sizeof(Hako_ServiceResponseHeader));
    if (!metadata || packet.size() < full_size) {
        return false;
    }
    auto response = response_body(step, time_usec, state);
    Hako_AckermannStepResponse wire {};
    PduDynamicMemory memory;
    if (!cpp_cpp2pdu_AckermannStepResponse(response, wire, memory)) {
        return false;
    }
    if (metadata->heap_off < full_size) {
        metadata->heap_off = full_size;
        metadata->total_size = full_size;
        std::memcpy(packet.data(), &*metadata, sizeof(*metadata));
    }
    // Keep the Core service header byte for byte, including its request ID and result code.
    std::memcpy(packet.data() + HAKO_PDU_META_DATA_SIZE()
        + offsetof(Hako_AckermannStepResponsePacket, body), &wire, sizeof(wire));
    return true;
}

struct Car {
    Geometry geometry {};
    std::array<std::uint32_t, 4> actuators {};
    std::vector<std::string> surfaces;
    std::vector<const char*> surface_views;
    std::uint64_t timeout_usec {0};
    bool rpc {false};

    std::optional<AckermannDriveCommand> active;
    std::optional<AckermannDriveCommand> requested;
    std::optional<double> friction;
    std::uint64_t stamp_usec {0};
    std::uint64_t step {0};
};

Car& car(HakoRobotPlugin* robot)
{
    return *reinterpret_cast<Car*>(robot);
}

Geometry read_geometry(const nlohmann::json& config)
{
    const auto positive = [&](const char* field) {
        const double value = config.at(field).get<double>();
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument("invalid Ackermann controller configuration");
        }
        return value;
    };
    return {positive("wheelbase_m"), positive("track_width_m"), positive("wheel_radius_m"),
        positive("max_steering_angle_rad"), positive("max_wheel_angular_velocity_rad_s")};
}

HakoRobotPlugin* create(const HakoRobotCreateV1* args, char* error, std::size_t error_size)
{
    try {
        const auto config = nlohmann::json::parse(args->config_json);
        auto robot = std::make_unique<Car>();
        const auto mode = config.at("mode").get<std::string>();
        robot->rpc = mode == "rpc";
        if (robot->rpc) {
            if (args->service_count != 1 || args->pdu_count != 0 || args->body_count != 1) {
                throw std::invalid_argument("Ackermann RPC requires one service and one body");
            }
        } else if (mode != "topic" || args->service_count != 0 || args->pdu_count != 1) {
            throw std::invalid_argument("Ackermann topic mode requires one PDU input");
        }
        robot->timeout_usec = config.at("input_timeout_usec").get<std::uint64_t>();
        if (robot->timeout_usec == 0) {
            throw std::invalid_argument("Ackermann input timeout must be positive");
        }
        robot->geometry = read_geometry(config.at("geometry"));
        const auto& bindings = config.at("actuators");
        constexpr std::array names {"steering_left", "steering_right", "drive_left", "drive_right"};
        for (std::size_t i = 0; i < names.size(); ++i) {
            const auto binding = bindings.at(names[i]).get<std::string>();
            const auto type = i < 2 ? HAKO_JOINT_COMMAND_POSITION : HAKO_JOINT_COMMAND_VELOCITY;
            const std::span actuators(args->actuators, args->actuator_count);
            const auto found = std::find_if(actuators.begin(), actuators.end(),
                [&](const auto& actuator) { return actuator.id == binding && actuator.command_type == type; });
            if (found == actuators.end()) {
                throw std::invalid_argument("invalid Ackermann actuator binding: " + binding);
            }
            robot->actuators[i] = static_cast<std::uint32_t>(found - actuators.begin());
        }
        robot->surfaces = config.value("tire_surfaces", std::vector<std::string> {});
        for (const auto& surface : robot->surfaces) {
            robot->surface_views.push_back(surface.c_str());
        }
        return reinterpret_cast<HakoRobotPlugin*>(robot.release());
    } catch (const std::exception& failure) {
        std::snprintf(error, error_size, "%s", failure.what());
        return nullptr;
    }
}

void destroy(HakoRobotPlugin* robot)
{
    delete &car(robot);
}

int32_t on_request(HakoRobotPlugin* robot, std::uint32_t service, const HakoServiceRequestV1* request)
{
    auto& state = car(robot);
    if (!state.rpc || service != 0) {
        return 0;
    }
    const auto decoded = decode_request({static_cast<const std::byte*>(request->data), request->size});
    if (!decoded || (decoded->tire_friction > 0.0 && state.surfaces.empty())) {
        return 0;
    }
    if (decoded->tire_friction > 0.0) {
        state.friction = decoded->tire_friction;
    }
    state.requested = decoded->drive;
    state.step = decoded->step;
    return 1;
}

int32_t on_step(HakoRobotPlugin* robot, const HakoTickV1* tick, const HakoRobotStateV1* state,
    HakoServiceSlotV1* slots, std::uint32_t slot_count, HakoRobotOutputV1* output, char*, std::size_t)
{
    auto& instance = car(robot);
    const auto time_usec = tick->sim_time_ns / 1000;
    std::optional<AckermannDriveCommand> command;
    if (instance.rpc) {
        command = instance.requested;
        instance.requested.reset();
        if (slot_count != 0 && slots[0].state != HAKO_SERVICE_STATE_IDLE) {
            slots[0].result = HAKO_SERVICE_RESULT_OK;
        }
        if (instance.friction && output->friction_capacity != 0) {
            output->frictions[0] = {static_cast<std::uint32_t>(instance.surface_views.size()),
                instance.surface_views.data(), *instance.friction};
            output->friction_count = 1;
            instance.friction.reset();
        }
    } else {
        for (const auto& pdu : std::span(state->pdus, state->pdu_count)) {
            if (pdu.channel == 0) {
                command = decode_topic({static_cast<const std::byte*>(pdu.data), pdu.size});
            }
        }
    }
    if (command) {
        // Skip this step for an invalid new drive command, keeping the previous drive.
        if (!std::isfinite(command->steering_angle_rad) || !std::isfinite(command->speed_m_s)) {
            return 0;
        }
        if (instance.rpc || time_usec <= std::numeric_limits<std::uint64_t>::max() - instance.timeout_usec) {
            instance.active = command;
            instance.stamp_usec = time_usec;
        }
    }
    if (!instance.rpc && (time_usec < instance.stamp_usec
            || time_usec - instance.stamp_usec >= instance.timeout_usec)) {
        instance.active.reset();
    }
    if (!instance.active) {
        return 0;
    }
    const auto targets = resolve_targets(*instance.active, instance.geometry);
    const std::array values {targets.steering_left, targets.steering_right, targets.drive_left, targets.drive_right};
    for (std::size_t i = 0; i < values.size(); ++i) {
        const auto index = instance.actuators[i];
        output->commands.has_command[index] = 1;
        output->commands.command_type[index] = i < 2 ? HAKO_JOINT_COMMAND_POSITION : HAKO_JOINT_COMMAND_VELOCITY;
        output->commands.value[index] = values[i];
    }
    return 0;
}

int32_t on_response(HakoRobotPlugin* robot, std::uint32_t service, const HakoTickV1* tick,
    const HakoRobotStateV1* state, const HakoServiceResponseV1* response)
{
    return car(robot).rpc && service == 0 && state->body_count == 1
        && encode_response(car(robot).step, tick->sim_time_ns / 1000, state->bodies[0],
            {static_cast<std::byte*>(response->data), response->size});
}

void reset(HakoRobotPlugin* robot)
{
    auto& state = car(robot);
    state.active.reset();
    state.requested.reset();
    state.friction.reset();
    state.step = 0;
    state.stamp_usec = 0;
}

const HakoServicePluginV1 table {
    {sizeof(HakoServicePluginV1), HAKO_SERVICE_PLUGIN_ABI_VERSION, HAKO_PLUGIN_KIND_SERVICE,
        HAKO_SERVICE_PLUGIN_INTERFACE_REVISION, "ackermann_step"},
    create, destroy, on_request, nullptr, on_step, on_response, reset,
};

} // namespace

std::optional<Request> decode_request(std::span<const std::byte> request)
{
    constexpr auto body_offset = offsetof(Hako_AckermannStepRequestPacket, body);
    // Python stops at the last field instead of adding C++ trailing padding.
    constexpr auto body_size = offsetof(Hako_AckermannStepRequest, tire_friction)
        + sizeof(Hako_AckermannStepRequest::tire_friction);
    const auto metadata = packet_metadata(request, body_offset + body_size);
    if (!metadata) {
        return std::nullopt;
    }
    Hako_AckermannStepRequest wire {};
    std::memcpy(&wire, request.data() + metadata->base_off + body_offset, body_size);
    HakoCpp_AckermannStepRequest decoded {};
    cpp_pdu2cpp_AckermannStepRequest(nullptr, wire, decoded);
    if (!std::isfinite(decoded.tire_friction)) {
        return std::nullopt;
    }
    return Request {decoded.step, {
        decoded.drive.steering_angle, decoded.drive.steering_angle_velocity,
        decoded.drive.speed, decoded.drive.acceleration, decoded.drive.jerk,
    }, decoded.tire_friction};
}

bool valid_topic(std::span<const std::byte> pdu)
{
    return packet_metadata(pdu, sizeof(Hako_AckermannDrive)).has_value();
}

std::optional<AckermannDriveCommand> decode_topic(std::span<const std::byte> pdu)
{
    if (!valid_topic(pdu)) {
        return std::nullopt;
    }
    Hako_AckermannDrive wire {};
    std::memcpy(&wire, pdu.data() + HAKO_PDU_META_DATA_SIZE(), sizeof(wire));
    HakoCpp_AckermannDrive decoded {};
    cpp_pdu2cpp_AckermannDrive(nullptr, wire, decoded);
    return AckermannDriveCommand {
        decoded.steering_angle, decoded.steering_angle_velocity,
        decoded.speed, decoded.acceleration, decoded.jerk,
    };
}

HAKO_ROBOT_PLUGIN_ENTRY(ackermann_step, table)

} // namespace hakoniwa::robot_runtime::plugins::ackermann_step
