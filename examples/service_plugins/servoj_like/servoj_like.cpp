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

struct ServoJ {
    std::array<std::size_t, joints.size()> actuator_index {};
    std::array<double, joints.size()> start {};
    Hako_ServoJRequest goal {};
    uint64_t start_ns {0};
    bool active {false};
    bool commanded {false};
};

void* create(const HakoServiceCreateV1* args, char* error, size_t error_size)
{
    if (args->joint_count != joints.size()) {
        std::snprintf(error, error_size, "expected six SO-101 actuators");
        return nullptr;
    }
    ServoJ service;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const auto names = args->joint_names;
        const auto found = std::find(names, names + args->joint_count, joints[i].name);
        if (found == names + args->joint_count) {
            std::snprintf(error, error_size, "missing SO-101 actuator %s", joints[i].name.data());
            return nullptr;
        }
        service.actuator_index[i] = static_cast<std::size_t>(found - names);
    }
    return new (std::nothrow) ServoJ(service);
}

int32_t on_request(void* instance, const void* packet, size_t size)
{
    constexpr auto offset = HAKO_PDU_META_DATA_SIZE() + offsetof(Hako_ServoJRequestPacket, body);
    if (size < offset + sizeof(Hako_ServoJRequest)) {
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
    static_cast<ServoJ*>(instance)->goal = goal;
    return 1;
}

int32_t on_step(void* instance, const HakoTickV1* tick, const HakoJointStatesV1* states,
    uint32_t server_state, HakoJointCommandsV1*)
{
    auto& service = *static_cast<ServoJ*>(instance);
    if (server_state == HAKO_SERVICE_STATE_CANCELING) {
        service.active = false;
        return HAKO_SERVICE_STEP_CANCELED;
    }
    for (std::size_t i = 0; i < joints.size(); ++i) {
        service.start[i] = states->position[service.actuator_index[i]];
    }
    service.start_ns = tick->sim_time_ns;
    service.active = true;
    service.commanded = false;
    return HAKO_SERVICE_STEP_OK;
}

int32_t on_response(void*, const HakoTickV1*, const HakoJointStatesV1*, const HakoServiceReplyV1* reply)
{
    constexpr auto body_offset = offsetof(Hako_ServoJResponsePacket, body)
        - sizeof(Hako_ServiceResponseHeader);
    constexpr auto padded_size = HAKO_ALIGN_SIZE(sizeof(Hako_ServoJResponsePacket), HAKO_ALIGNMENT_SIZE)
        - sizeof(Hako_ServiceResponseHeader);
    if (reply->metadata_size != HAKO_PDU_META_DATA_SIZE()
        || reply->header_size != sizeof(Hako_ServiceResponseHeader)
        || reply->payload_size < padded_size) {
        return 0;
    }
    HakoPduMetaDataType metadata {};
    std::memcpy(&metadata, reply->metadata, sizeof(metadata));
    if (HAKO_PDU_METADATA_IS_INVALID(&metadata) || metadata.base_off != reply->metadata_size) {
        return 0;
    }
    const auto size = static_cast<uint32_t>(reply->metadata_size + reply->header_size + padded_size);
    metadata.heap_off = size;
    metadata.total_size = size;
    std::memcpy(reply->metadata, &metadata, sizeof(metadata));
    const Hako_ServoJResponse body {0};
    std::memset(reply->payload, 0, padded_size);
    std::memcpy(static_cast<char*>(reply->payload) + body_offset, &body, sizeof(body));
    return 1;
}

int32_t update(void* instance, const HakoTickV1* tick, const HakoJointStatesV1*, uint32_t,
    const HakoPduViewV1*, HakoJointCommandsV1* commands, char*, size_t)
{
    auto& service = *static_cast<ServoJ*>(instance);
    if (service.commanded && !tick->selected) {
        service.active = false;
    }
    if (!service.active) {
        return 0;
    }
    const double elapsed = static_cast<double>(tick->sim_time_ns - service.start_ns) * 1e-9;
    const double progress = std::min(elapsed / service.goal.t, 1.0);
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const auto target = service.actuator_index[i];
        commands->has_command[target] = 1;
        commands->command_type[target] = HAKO_JOINT_COMMAND_POSITION;
        commands->value[target] = std::lerp(service.start[i], service.goal.joint[i], progress);
    }
    service.commanded = true;
    return 0;
}

void reset(void* instance)
{
    auto& service = *static_cast<ServoJ*>(instance);
    service.active = service.commanded = false;
}

void destroy(void* instance)
{
    delete static_cast<ServoJ*>(instance);
}

const HakoServicePluginV1 table {
    {sizeof(HakoServicePluginV1), HAKO_SERVICE_PLUGIN_ABI_VERSION, HAKO_PLUGIN_KIND_SERVICE,
        HAKO_SERVICE_PLUGIN_INTERFACE_REVISION, "servoj-like"},
    create, on_request, on_step, on_response, nullptr, reset, destroy, update,
};

} // namespace

extern "C" HAKO_PLUGIN_EXPORT const HakoPluginHeaderV1* hako_robot_runtime_plugin_v1(void)
{
    return &table.header;
}
