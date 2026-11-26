#include "hakoniwa/robot_runtime/adapters/endpoint/person_step_rpc_event_reader.hpp"

#include "hakoniwa/robot_runtime/adapters/endpoint/ackermann_drive_rpc_event_reader.hpp"
#include <algorithm>
#include <array>
#include <cstring>
#include <type_traits>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {
namespace {

constexpr std::size_t person_text_capacity = 256;

struct CommandWire {
    std::array<double, 3> linear_velocity;
    std::array<double, 3> angular_velocity;
    std::array<char, person_text_capacity> animation;
    std::array<char, person_text_capacity> ride;
};

struct StateWire {
    std::uint32_t result_code;
    std::uint64_t simulation_time_usec;
    std::array<double, 3> position;
    std::array<double, 4> orientation;
    std::array<double, 3> linear_velocity;
    std::array<double, 3> angular_velocity;
    std::array<double, person_limb_joint_names.size()> joint_positions;
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

void copy_text(const std::string& source, std::array<char, person_text_capacity>& target)
{
    const auto count = std::min(source.size(), target.size() - 1);
    std::copy_n(source.data(), count, target.data());
    target[count] = '\0';
}

std::string read_text(const std::array<char, person_text_capacity>& source)
{
    const auto end = std::find(source.begin(), source.end(), '\0');
    return std::string(source.begin(), end);
}

} // namespace

std::vector<std::byte> encode_person_command(const PersonStepCommand& command)
{
    CommandWire wire {
        {command.linear_velocity.x, command.linear_velocity.y,
            command.linear_velocity.z},
        {command.angular_velocity.x, command.angular_velocity.y,
            command.angular_velocity.z},
        {},
        {},
    };
    copy_text(command.animation, wire.animation);
    copy_text(command.ride, wire.ride);
    return pack(wire);
}

bool decode_person_command(
    const std::span<const std::byte> payload,
    PersonStepCommand& command) noexcept
{
    CommandWire wire {};
    if (!unpack(payload, wire)) return false;
    command = {
        {wire.linear_velocity[0], wire.linear_velocity[1],
            wire.linear_velocity[2]},
        {wire.angular_velocity[0], wire.angular_velocity[1],
            wire.angular_velocity[2]},
        read_text(wire.animation),
        read_text(wire.ride),
    };
    return true;
}

std::vector<std::byte> encode_person_state(const PersonStepState& state)
{
    return pack(StateWire {
        state.result_code,
        state.simulation_time_usec,
        {state.position.x, state.position.y, state.position.z},
        {state.orientation.x, state.orientation.y,
            state.orientation.z, state.orientation.w},
        {state.linear_velocity.x, state.linear_velocity.y,
            state.linear_velocity.z},
        {state.angular_velocity.x, state.angular_velocity.y,
            state.angular_velocity.z},
        state.joint_positions,
    });
}

bool decode_person_state(
    const std::span<const std::byte> payload,
    PersonStepState& state) noexcept
{
    StateWire wire {};
    if (!unpack(payload, wire)) return false;
    state = {
        wire.result_code,
        wire.simulation_time_usec,
        {wire.position[0], wire.position[1], wire.position[2]},
        {wire.orientation[0], wire.orientation[1],
            wire.orientation[2], wire.orientation[3]},
        {wire.linear_velocity[0], wire.linear_velocity[1],
            wire.linear_velocity[2]},
        {wire.angular_velocity[0], wire.angular_velocity[1],
            wire.angular_velocity[2]},
        wire.joint_positions,
    };
    return true;
}

std::string person_step_service_name(const std::string_view person_name)
{
    return make_robot_rpc_service_name(
        "PersonService", "PersonStep", person_name);
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
