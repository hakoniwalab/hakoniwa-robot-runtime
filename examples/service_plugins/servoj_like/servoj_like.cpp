#include "hakoniwa/robot_runtime/plugin/service_plugin.h"

#include "my_robot_srvs_msgs/pdu_ctype_ServoJRequestPacket.h"
#include "my_robot_srvs_msgs/pdu_ctype_ServoJResponsePacket.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <new>
#include <string_view>

namespace {

struct Joint {
    std::string_view name;
    double minimum;
    double maximum;
};

constexpr std::array joints {
    Joint {"shoulder_pan", -1.91986, 1.91986},
    Joint {"shoulder_lift", -1.74533, 1.74533},
    Joint {"elbow_flex", -1.69, 1.69},
    Joint {"wrist_flex", -1.65806, 1.65806},
    Joint {"wrist_roll", -2.74385, 2.84121},
    Joint {"gripper", -0.174533, 1.74533},
};

} // namespace

struct HakoRobotPlugin {
    std::array<std::size_t, joints.size()> actuator_index {};
    std::array<double, joints.size()> start {};
    Hako_ServoJRequest goal {};
    uint64_t start_ns {0};
    bool active {false};
    bool commanded {false};
};

namespace {

HakoRobotPlugin* create(const HakoRobotCreateV1* args, char* error, size_t error_size)
{
    if (args->actuator_count != joints.size()) {
        std::snprintf(error, error_size, "expected six SO-101 actuators");
        return nullptr;
    }
    HakoRobotPlugin service;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const auto first = args->actuators;
        const auto last = first + args->actuator_count;
        const auto found = std::find_if(first, last, [&](const auto& actuator) {
            return actuator.id == joints[i].name;
        });
        if (found == last) {
            std::snprintf(error, error_size, "missing SO-101 actuator %s", joints[i].name.data());
            return nullptr;
        }
        service.actuator_index[i] = static_cast<std::size_t>(found - first);
    }
    return new (std::nothrow) HakoRobotPlugin(service);
}

void destroy(HakoRobotPlugin* instance)
{
    delete instance;
}

int32_t on_request(HakoRobotPlugin* instance, uint32_t service, const HakoServiceRequestV1* request)
{
    const auto* packet = request->data;
    const auto size = request->size;
    constexpr auto offset = HAKO_PDU_META_DATA_SIZE() + offsetof(Hako_ServoJRequestPacket, body);
    if (service != 0 || size < offset + sizeof(Hako_ServoJRequest)) {
        return 0;
    }
    HakoPduMetaDataType metadata {};
    std::memcpy(&metadata, packet, sizeof(metadata));
    if (HAKO_PDU_METADATA_IS_INVALID(&metadata)
        || metadata.base_off != HAKO_PDU_META_DATA_SIZE()
        || metadata.heap_off < offset + sizeof(Hako_ServoJRequest)
        || metadata.heap_off > metadata.total_size || metadata.total_size > size) {
        return 0;
    }
    Hako_ServoJRequest goal {};
    std::memcpy(&goal, static_cast<const char*>(packet) + offset, sizeof(goal));
    if (!std::isfinite(goal.t) || goal.t < 1e-6 || goal.t > 60.0) {
        return 0;
    }
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const auto value = goal.joint[i];
        if (!std::isfinite(value) || value < joints[i].minimum || value > joints[i].maximum) {
            return 0;
        }
    }
    instance->goal = goal;
    return 1;
}

int32_t on_step(HakoRobotPlugin* instance, const HakoTickV1* tick, const HakoRobotStateV1* state,
    HakoServiceSlotV1* slots, uint32_t slot_count, HakoRobotOutputV1* output, char*, size_t)
{
    if (slot_count != 1) {
        return 1;
    }
    auto& service = *instance;
    auto& slot = slots[0];
    if (slot.state == HAKO_SERVICE_STATE_CANCELING) {
        service.active = false;
        slot.result = HAKO_SERVICE_RESULT_CANCELED;
    } else if (slot.state == HAKO_SERVICE_STATE_DOING) {
        for (std::size_t i = 0; i < joints.size(); ++i) {
            service.start[i] = state->joints.position[service.actuator_index[i]];
        }
        service.start_ns = tick->sim_time_ns;
        service.active = true;
        service.commanded = false;
        slot.result = HAKO_SERVICE_RESULT_OK;
    }
    if (service.commanded && !tick->selected) {
        service.active = false;
    }
    if (!service.active) {
        return 0;
    }
    const double elapsed = static_cast<double>(tick->sim_time_ns - service.start_ns) * 1e-9;
    const double progress = std::min(elapsed / service.goal.t, 1.0);
    auto& commands = output->commands;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const auto target = service.actuator_index[i];
        commands.has_command[target] = 1;
        commands.command_type[target] = HAKO_JOINT_COMMAND_POSITION;
        commands.value[target] = std::lerp(service.start[i], service.goal.joint[i], progress);
    }
    service.commanded = true;
    return 0;
}

int32_t on_response(HakoRobotPlugin*, uint32_t service, const HakoTickV1*, const HakoRobotStateV1*,
    const HakoServiceResponseV1* response)
{
    constexpr auto packet_size = HAKO_ALIGN_SIZE(sizeof(Hako_ServoJResponsePacket), HAKO_ALIGNMENT_SIZE);
    const auto base = hako_service_packet_base_offset(response->data, response->size);
    if (service != 0 || base != HAKO_PDU_META_DATA_SIZE() || response->size - base < packet_size) {
        return 0;
    }
    HakoPduMetaDataType metadata {};
    std::memcpy(&metadata, response->data, sizeof(metadata));
    if (HAKO_PDU_METADATA_IS_INVALID(&metadata)) {
        return 0;
    }
    const auto size = static_cast<uint32_t>(base + packet_size);
    metadata.heap_off = size;
    metadata.total_size = size;
    std::memcpy(response->data, &metadata, sizeof(metadata));
    auto* packet = static_cast<char*>(response->data) + base;
    constexpr auto header_size = sizeof(Hako_ServiceResponseHeader);
    const Hako_ServoJResponse body {0};
    std::memset(packet + header_size, 0, packet_size - header_size);
    std::memcpy(packet + offsetof(Hako_ServoJResponsePacket, body), &body, sizeof(body));
    return 1;
}

void reset(HakoRobotPlugin* instance)
{
    instance->active = false;
    instance->commanded = false;
}

const HakoServicePluginV1 table {
    {sizeof(HakoServicePluginV1), HAKO_SERVICE_PLUGIN_ABI_VERSION, HAKO_PLUGIN_KIND_SERVICE,
        HAKO_SERVICE_PLUGIN_INTERFACE_REVISION, "servoj-like"},
    create, destroy, on_request, nullptr, on_step, on_response, reset,
};

} // namespace

HAKO_ROBOT_PLUGIN_ENTRY(servoj_like, table)
