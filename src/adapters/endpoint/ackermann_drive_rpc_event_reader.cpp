#include "adapters/endpoint/ackermann_step_packet.hpp"

#include <array>
#include <cstring>
#include <stdexcept>
#include <type_traits>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {
namespace {

struct CommandWire {
    double steering_angle_rad;
    double steering_angle_velocity_rad_s;
    double speed_m_s;
    double acceleration_m_s2;
    double jerk_m_s3;
};

struct StateWire {
    std::uint64_t simulation_time_usec;
    std::array<double, 3> position;
    std::array<double, 4> orientation;
    std::array<double, 3> linear_velocity;
    std::array<double, 3> angular_velocity;
};

template <typename T>
std::vector<std::byte> pack(const T& value)
{
    static_assert(std::is_trivially_copyable_v<T>);
    std::vector<std::byte> result(sizeof(T));
    std::memcpy(result.data(), &value, sizeof(T));
    return result;
}

template <typename T>
bool unpack(std::span<const std::byte> payload, T& value) noexcept
{
    static_assert(std::is_trivially_copyable_v<T>);
    if (payload.size() != sizeof(T)) return false;
    std::memcpy(&value, payload.data(), sizeof(T));
    return true;
}

} // namespace

std::string make_robot_rpc_service_name(
    const std::string_view domain,
    const std::string_view operation,
    const std::string_view robot_name)
{
    if (domain.empty() || operation.empty() || robot_name.empty()) {
        throw std::invalid_argument(
            "RPC service domain, operation, and robot name must not be empty");
    }
    return std::string(domain) + "/" + std::string(operation) + "/"
        + std::string(robot_name);
}

std::string ackermann_step_service_name(const std::string_view vehicle_name)
{
    return make_robot_rpc_service_name(
        "CarService", "AckermannStep", vehicle_name);
}

void AckermannDriveRpcEventReader::apply(
    const runtime::AckermannDriveCommand& command) noexcept
{
    pending_ = command;
}

std::optional<runtime::AckermannDriveCommand>
AckermannDriveRpcEventReader::take_latest()
{
    auto result = pending_;
    pending_.reset();
    return result;
}

void AckermannDriveRpcEventReader::reset() noexcept
{
    pending_.reset();
}

std::vector<std::byte> encode_ackermann_command(
    const runtime::AckermannDriveCommand& command)
{
    return pack(CommandWire {
        command.steering_angle_rad,
        command.steering_angle_velocity_rad_s,
        command.speed_m_s,
        command.acceleration_m_s2,
        command.jerk_m_s3,
    });
}

bool decode_ackermann_command(
    const std::span<const std::byte> payload,
    runtime::AckermannDriveCommand& command) noexcept
{
    CommandWire wire {};
    if (!unpack(payload, wire)) return false;
    command = {
        wire.steering_angle_rad,
        wire.steering_angle_velocity_rad_s,
        wire.speed_m_s,
        wire.acceleration_m_s2,
        wire.jerk_m_s3,
    };
    return true;
}

std::vector<std::byte> encode_ackermann_state(const AckermannStepState& state)
{
    return pack(StateWire {
        state.simulation_time_usec,
        {state.position.x, state.position.y, state.position.z},
        {state.orientation.x, state.orientation.y,
            state.orientation.z, state.orientation.w},
        {state.linear_velocity.x, state.linear_velocity.y,
            state.linear_velocity.z},
        {state.angular_velocity.x, state.angular_velocity.y,
            state.angular_velocity.z},
    });
}

bool decode_ackermann_state(
    const std::span<const std::byte> payload,
    AckermannStepState& state) noexcept
{
    StateWire wire {};
    if (!unpack(payload, wire)) return false;
    state = {
        wire.simulation_time_usec,
        {wire.position[0], wire.position[1], wire.position[2]},
        {wire.orientation[0], wire.orientation[1],
            wire.orientation[2], wire.orientation[3]},
        {wire.linear_velocity[0], wire.linear_velocity[1],
            wire.linear_velocity[2]},
        {wire.angular_velocity[0], wire.angular_velocity[1],
            wire.angular_velocity[2]},
    };
    return true;
}

HakoCpp_AckermannStepResponse ackermann_step_response_body(
    std::uint64_t step, const AckermannStepState& state)
{
    HakoCpp_AckermannStepResponse body {};
    body.step = step;
    body.result_code = 0;
    body.sim_time_usec = state.simulation_time_usec;
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

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
