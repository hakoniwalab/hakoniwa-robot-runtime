#include "adapters/endpoint/person_step_packet.hpp"

#include "hakoniwa/robot_runtime/adapters/endpoint/ackermann_drive_rpc_event_reader.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_PersonStepRequestPacket.hpp"
#define hako_convert_pdu2cpp_array_string_varray hako_convert_pdu2ros_array_string_varray
#define hako_convert_cpp2pdu_array_string_varray hako_convert_ros2pdu_array_string_varray
#include "hako_srv_msgs/pdu_cpptype_conv_PersonStepResponsePacket.hpp"
#undef hako_convert_pdu2cpp_array_string_varray
#undef hako_convert_cpp2pdu_array_string_varray

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

bool read_person_step_packet_metadata(std::span<const std::byte> packet, std::size_t base_size,
    HakoPduMetaDataType& metadata)
{
    if (packet.size() < sizeof(metadata)) {
        return false;
    }
    std::memcpy(&metadata, packet.data(), sizeof(metadata));
    return metadata.magicno == HAKO_PDU_META_DATA_MAGICNO
        && metadata.version == HAKO_PDU_META_DATA_VERSION
        && metadata.base_off == HAKO_PDU_META_DATA_SIZE()
        && metadata.heap_off >= metadata.base_off + base_size
        && metadata.heap_off <= metadata.total_size
        && metadata.total_size <= packet.size();
}

bool encode_person_step_response(
    const std::string_view person_name, const std::uint64_t step, const PersonStepState& state,
    std::span<std::byte> packet)
{
    HakoPduMetaDataType metadata {};
    if (!read_person_step_packet_metadata(packet, sizeof(Hako_ServiceResponseHeader), metadata)) {
        return false;
    }
    HakoCpp_PersonStepResponse body {};
    body.step = step;
    body.result_code = state.result_code;
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
    body.joints.header.frame_id = "city_map";
    body.joints.header.stamp.sec =
        static_cast<std::int32_t>(state.simulation_time_usec / 1'000'000);
    body.joints.header.stamp.nanosec =
        static_cast<std::uint32_t>((state.simulation_time_usec % 1'000'000) * 1000);
    for (std::size_t index = 0;
         index < person_limb_joint_names.size(); ++index) {
        body.joints.name.push_back(
            std::string(person_name) + "/" + std::string(person_limb_joint_names[index]));
        body.joints.position.push_back(state.joint_positions[index]);
        body.joints.velocity.push_back(0.0);
        body.joints.effort.push_back(0.0);
    }
    Hako_PersonStepResponsePacket wire;
    std::memset(&wire, 0, sizeof(wire));
    PduDynamicMemory heap;
    if (!cpp_cpp2pdu_PersonStepResponse(body, wire.body, heap)) {
        return false;
    }

    constexpr auto body_offset = offsetof(Hako_PersonStepResponsePacket, body);
    metadata.heap_off = metadata.base_off + HAKO_ALIGN_SIZE(sizeof(wire), HAKO_ALIGNMENT_SIZE);
    metadata.total_size = metadata.heap_off
        + HAKO_ALIGN_SIZE(heap.get_total_size(), HAKO_ALIGNMENT_SIZE);
    if (packet.size() < metadata.total_size) {
        return false;
    }
    // Older generated converters leave bytes after fixed strings undefined.
    // Clear those bytes in the response body and preserve the Core header.
    auto& frame = wire.body.joints.header.frame_id;
    std::fill(std::find(std::begin(frame), std::end(frame), '\0'), std::end(frame), '\0');
    std::memcpy(packet.data(), &metadata, sizeof(metadata));
    std::fill(packet.begin() + metadata.base_off + sizeof(wire.header),
        packet.begin() + metadata.total_size, std::byte {0});
    std::memcpy(packet.data() + metadata.base_off + body_offset, &wire.body, sizeof(wire.body));

    auto* heap_bytes = reinterpret_cast<char*>(packet.data() + metadata.heap_off);
    heap.copy_to_pdu(heap_bytes);
    auto* names = heap_bytes + wire.body.joints._name_off;
    for (std::size_t i = 0; i < person_limb_joint_names.size(); ++i) {
        auto* text = names + i * sizeof(Hako_cstring);
        auto* end = text + sizeof(Hako_cstring);
        std::fill(std::find(text, end, '\0'), end, '\0');
    }
    return true;
}

bool encode_legacy_person_step_response(
    const std::string_view person_name, const runner::AsyncRpcResponse& response,
    std::span<std::byte> packet, std::string& error)
{
    PersonStepState state;
    if (!decode_person_state(response.payload, state)) {
        error = "invalid PersonStep response state";
        return false;
    }
    if (!encode_person_step_response(person_name, response.step, state, packet)) {
        error = "failed to encode PersonStep response";
        return false;
    }
    return true;
}

std::shared_ptr<runner::hakoniwa::CoreAsyncRpcServiceChannel>
make_person_step_service_channel(
    std::string person_name,
    std::string asset_name)
{
    const auto service_name = person_step_service_name(person_name);
    auto decoder = [](std::span<const std::byte> packet,
                       runner::AsyncRpcRequest& request,
                       std::string& error) {
        HakoCpp_PersonStepRequestPacket decoded {};
        if (!hako::pdu::msgs::hako_srv_msgs::PersonStepRequestPacket {}.pdu2cpp(
                const_cast<char*>(reinterpret_cast<const char*>(packet.data())), decoded)) {
            error = "failed to decode PersonStep request";
            return false;
        }
        request.step = decoded.body.step;
        request.payload = encode_person_command({
            {decoded.body.cmd_vel.linear.x, decoded.body.cmd_vel.linear.y,
                decoded.body.cmd_vel.linear.z},
            {decoded.body.cmd_vel.angular.x, decoded.body.cmd_vel.angular.y,
                decoded.body.cmd_vel.angular.z},
            decoded.body.animation,
            decoded.body.ride,
        });
        return true;
    };
    auto encoder = [person_name](const runner::AsyncRpcResponse& response,
                                  std::span<std::byte> packet,
                                  std::string& error) {
        return encode_legacy_person_step_response(person_name, response, packet, error);
    };
    return std::make_shared<runner::hakoniwa::CoreAsyncRpcServiceChannel>(
        std::move(person_name), std::move(asset_name), service_name,
        std::move(decoder), std::move(encoder));
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
