#include "hakoniwa/robot_runtime/plugin/service_plugin.h"
#include "plugins/packet_metadata.hpp"
#include "plugins/person_step/limb_names.hpp"

#include "hako_srv_msgs/pdu_ctype_PersonStepRequestPacket.h"
#define hako_convert_pdu2cpp_array_string_varray hako_convert_pdu2ros_array_string_varray
#define hako_convert_cpp2pdu_array_string_varray hako_convert_ros2pdu_array_string_varray
#include "hako_srv_msgs/pdu_cpptype_conv_PersonStepResponsePacket.hpp"
#undef hako_convert_pdu2cpp_array_string_varray
#undef hako_convert_cpp2pdu_array_string_varray

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

namespace hakoniwa::robot_runtime::plugins::person_step {
namespace {

bool encode_person_step_response(
    const std::string_view person_name, const std::uint64_t step, std::uint64_t time_usec,
    const HakoHumanoidStateV1& state,
    std::span<std::byte> packet)
{
    auto metadata = packet_metadata(packet, sizeof(Hako_ServiceResponseHeader));
    if (!metadata || state.limb_count != limb_names.size()) {
        return false;
    }
    HakoCpp_PersonStepResponse body {};
    body.step = step;
    body.result_code = state.last_motion_result;
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
    body.joints.header.frame_id = "city_map";
    body.joints.header.stamp.sec =
        static_cast<std::int32_t>(time_usec / 1'000'000);
    body.joints.header.stamp.nanosec =
        static_cast<std::uint32_t>((time_usec % 1'000'000) * 1000);
    for (std::size_t index = 0;
         index < limb_names.size(); ++index) {
        body.joints.name.push_back(
            std::string(person_name) + "/" + std::string(limb_names[index]));
        body.joints.position.push_back(state.limb_positions[index]);
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
    metadata->heap_off = metadata->base_off + HAKO_ALIGN_SIZE(sizeof(wire), HAKO_ALIGNMENT_SIZE);
    metadata->total_size = metadata->heap_off
        + HAKO_ALIGN_SIZE(heap.get_total_size(), HAKO_ALIGNMENT_SIZE);
    if (packet.size() < metadata->total_size) {
        return false;
    }
    // Older generated converters leave bytes after fixed strings undefined.
    // Clear those bytes in the response body and preserve the Core header.
    auto& frame = wire.body.joints.header.frame_id;
    std::fill(std::find(std::begin(frame), std::end(frame), '\0'), std::end(frame), '\0');
    std::memcpy(packet.data(), &*metadata, sizeof(*metadata));
    std::fill(packet.begin() + metadata->base_off + sizeof(wire.header),
        packet.begin() + metadata->total_size, std::byte {0});
    std::memcpy(packet.data() + metadata->base_off + body_offset, &wire.body, sizeof(wire.body));

    auto* heap_bytes = reinterpret_cast<char*>(packet.data() + metadata->heap_off);
    heap.copy_to_pdu(heap_bytes);
    auto* names = heap_bytes + wire.body.joints._name_off;
    for (std::size_t i = 0; i < limb_names.size(); ++i) {
        auto* text = names + i * sizeof(Hako_cstring);
        auto* end = text + sizeof(Hako_cstring);
        std::fill(std::find(text, end, '\0'), end, '\0');
    }
    return true;
}

struct Person {
    std::string name;
    std::string animation;
    std::string ride;
    HakoVector3V1 linear_velocity {};
    HakoVector3V1 angular_velocity {};
    std::uint64_t step {0};
    bool pending {false};
};

Person& person(HakoRobotPlugin* robot)
{
    return *reinterpret_cast<Person*>(robot);
}

std::string read_text(const char (&text)[HAKO_STRING_SIZE])
{
    return {text, std::find(text, text + HAKO_STRING_SIZE, '\0')};
}

HakoRobotPlugin* create(const HakoRobotCreateV1* args, char* error, std::size_t error_size)
{
    if (args->humanoid_count != 1 || args->humanoids[0][0] == '\0' || args->service_count != 1) {
        std::snprintf(error, error_size, "PersonStep requires one humanoid and one service");
        return nullptr;
    }
    try {
        auto robot = std::make_unique<Person>();
        robot->name = args->humanoids[0];
        return reinterpret_cast<HakoRobotPlugin*>(robot.release());
    } catch (const std::exception& failure) {
        std::snprintf(error, error_size, "%s", failure.what());
        return nullptr;
    }
}

void destroy(HakoRobotPlugin* robot)
{
    delete &person(robot);
}

int32_t on_request(HakoRobotPlugin* robot, std::uint32_t service, const HakoServiceRequestV1* request)
{
    constexpr auto body_offset = offsetof(Hako_PersonStepRequestPacket, body);
    // Python stops at the last wire field instead of rounding the packet up.
    constexpr auto body_size = offsetof(Hako_PersonStepRequest, ride) + sizeof(Hako_PersonStepRequest::ride);
    const auto bytes = static_cast<const std::byte*>(request->data);
    const auto metadata = packet_metadata({bytes, request->size}, body_offset + body_size);
    if (service != 0 || !metadata) {
        return 0;
    }
    Hako_PersonStepRequest wire {};
    std::memcpy(&wire, bytes + metadata->base_off + body_offset, body_size);
    auto animation = read_text(wire.animation);
    auto ride = read_text(wire.ride);
    auto& state = person(robot);
    state.animation = std::move(animation);
    state.ride = std::move(ride);
    state.linear_velocity = {wire.cmd_vel.linear.x, wire.cmd_vel.linear.y, wire.cmd_vel.linear.z};
    state.angular_velocity = {wire.cmd_vel.angular.x, wire.cmd_vel.angular.y, wire.cmd_vel.angular.z};
    state.step = wire.step;
    state.pending = true;
    return 1;
}

int32_t on_step(HakoRobotPlugin* robot, const HakoTickV1*, const HakoRobotStateV1*,
    HakoServiceSlotV1* slots, std::uint32_t slot_count, HakoRobotOutputV1* output, char*, std::size_t)
{
    auto& state = person(robot);
    if (slot_count != 0 && slots[0].state != HAKO_SERVICE_STATE_IDLE) {
        slots[0].result = HAKO_SERVICE_RESULT_OK;
        if (state.pending && output->humanoid_motion_capacity != 0) {
            output->humanoid_motions[0] = {state.name.c_str(), state.linear_velocity, state.angular_velocity,
                state.animation.c_str(), state.ride.c_str()};
            output->humanoid_motion_count = 1;
            state.pending = false;
        }
    }
    return 0;
}

int32_t on_response(HakoRobotPlugin* robot, std::uint32_t service, const HakoTickV1* tick,
    const HakoRobotStateV1* state, const HakoServiceResponseV1* response)
{
    const auto& instance = person(robot);
    return service == 0 && state->humanoid_count == 1
        && encode_person_step_response(instance.name, instance.step, tick->sim_time_ns / 1000, state->humanoids[0],
            {static_cast<std::byte*>(response->data), response->size});
}

void reset(HakoRobotPlugin* robot)
{
    auto& state = person(robot);
    state.step = 0;
    state.pending = false;
}

const HakoServicePluginV1 table {
    {sizeof(HakoServicePluginV1), HAKO_SERVICE_PLUGIN_ABI_VERSION, HAKO_PLUGIN_KIND_SERVICE,
        HAKO_SERVICE_PLUGIN_INTERFACE_REVISION, "person_step"},
    R"({"service_types":["hako_srv_msgs/PersonStep"],"plant_operations":["humanoid_motion"]})",
    create, destroy, on_request, nullptr, on_step, on_response, reset,
};

} // namespace

HAKO_ROBOT_PLUGIN_ENTRY(person_step, table)

} // namespace hakoniwa::robot_runtime::plugins::person_step
