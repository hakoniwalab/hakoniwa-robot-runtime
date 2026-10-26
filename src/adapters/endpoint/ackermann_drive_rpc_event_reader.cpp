#include "hakoniwa/robot_runtime/adapters/endpoint/ackermann_drive_rpc_event_reader.hpp"

#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepResponsePacket.hpp"

#include <array>
#include <cstring>
#include <stdexcept>
#include <type_traits>
#include <utility>

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

std::shared_ptr<runner::hakoniwa::CoreAsyncRpcServiceChannel>
make_ackermann_step_service_channel(
    std::string vehicle_name,
    std::string asset_name)
{
    const auto service_name = ackermann_step_service_name(vehicle_name);
    auto decoder = [](std::span<const std::byte> packet,
                       runner::AsyncRpcRequest& request,
                       std::string& error) {
        HakoCpp_AckermannStepRequestPacket decoded {};
        if (!hako::pdu::msgs::hako_srv_msgs::AckermannStepRequestPacket {}.pdu2cpp(
                const_cast<char*>(reinterpret_cast<const char*>(packet.data())), decoded)) {
            error = "failed to decode AckermannStep request";
            return false;
        }
        request.step = decoded.body.step;
        request.payload = encode_ackermann_command({
            decoded.body.drive.steering_angle,
            decoded.body.drive.steering_angle_velocity,
            decoded.body.drive.speed,
            decoded.body.drive.acceleration,
            decoded.body.drive.jerk,
        });
        return true;
    };
    auto encoder = [](const runner::AsyncRpcResponse& response,
                       std::span<std::byte> packet,
                       std::string& error) {
        AckermannStepState state;
        if (!decode_ackermann_state(response.payload, state)) {
            error = "invalid AckermannStep response state";
            return false;
        }
        HakoCpp_AckermannStepResponsePacket encoded {};
        auto converter = hako::pdu::msgs::hako_srv_msgs::AckermannStepResponsePacket {};
        if (!converter.pdu2cpp(reinterpret_cast<char*>(packet.data()), encoded)) {
            error = "failed to initialize AckermannStep response";
            return false;
        }
        encoded.body.step = response.step;
        encoded.body.result_code = 0;
        encoded.body.sim_time_usec = state.simulation_time_usec;
        encoded.body.pose.translation.x = state.position.x;
        encoded.body.pose.translation.y = state.position.y;
        encoded.body.pose.translation.z = state.position.z;
        encoded.body.pose.rotation.x = state.orientation.x;
        encoded.body.pose.rotation.y = state.orientation.y;
        encoded.body.pose.rotation.z = state.orientation.z;
        encoded.body.pose.rotation.w = state.orientation.w;
        encoded.body.twist.linear.x = state.linear_velocity.x;
        encoded.body.twist.linear.y = state.linear_velocity.y;
        encoded.body.twist.linear.z = state.linear_velocity.z;
        encoded.body.twist.angular.x = state.angular_velocity.x;
        encoded.body.twist.angular.y = state.angular_velocity.y;
        encoded.body.twist.angular.z = state.angular_velocity.z;
        if (converter.cpp2pdu(
                encoded, reinterpret_cast<char*>(packet.data()),
                static_cast<int>(packet.size())) < 0) {
            error = "failed to encode AckermannStep response";
            return false;
        }
        return true;
    };
    return std::make_shared<runner::hakoniwa::CoreAsyncRpcServiceChannel>(
        std::move(vehicle_name), std::move(asset_name), service_name,
        std::move(decoder), std::move(encoder));
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
