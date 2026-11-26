/*
 * Robot Runtime asynchronous service plugin interface (C).
 *
 * The entry is hako_robot_runtime_plugin_v1. A service instance is shared by
 * the RPC hooks and the optional per-step controller. Calls are synchronous
 * on the simulation thread after create, until destroy. No Core callbacks
 * or transport APIs are needed by a plugin.
 */
#ifndef HAKONIWA_ROBOT_RUNTIME_SERVICE_PLUGIN_H
#define HAKONIWA_ROBOT_RUNTIME_SERVICE_PLUGIN_H

#include "hakoniwa/robot_runtime/plugin/plugin_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HAKO_PLUGIN_KIND_SERVICE 3u
#define HAKO_SERVICE_PLUGIN_ABI_VERSION 1u
#define HAKO_SERVICE_PLUGIN_INTERFACE_REVISION "service-plugin/1"

/* Values of server_state in on_step. */
#define HAKO_SERVICE_STATE_IDLE      0u
#define HAKO_SERVICE_STATE_DOING     1u
#define HAKO_SERVICE_STATE_CANCELING 2u

/* Return values of on_step. Lockstep services must finish in one step. */
#define HAKO_SERVICE_STEP_PENDING  0
#define HAKO_SERVICE_STEP_OK       1
#define HAKO_SERVICE_STEP_CANCELED 2
#define HAKO_SERVICE_STEP_ERROR    3

typedef struct HakoServiceCreateV1 {
    uint32_t struct_size;
    const char* service_name;
    const char* config_json;
    /* Manifest actuator IDs for pdu_robot, in manifest order. May be empty. */
    uint32_t joint_count;
    const char* const* joint_names;
    uint64_t physics_step_ns;
} HakoServiceCreateV1;

/*
 * The prepared reply packet, in wire order. Only metadata and payload are
 * writable. Keep the metadata's base offset and preserve the Core-owned
 * service header. Extend metadata to describe the payload as needed and
 * initialize the whole reply body: Core may reuse its buffer.
 */
typedef struct HakoServiceReplyV1 {
    void* metadata;
    size_t metadata_size;
    const void* header;
    size_t header_size;
    void* payload;
    size_t payload_size;
} HakoServiceReplyV1;

typedef struct HakoServicePluginV1 {
    HakoPluginHeaderV1 header;

    /* Returns the instance, or null with an optional NUL-terminated error. */
    void* (*create)(const HakoServiceCreateV1* args, char* error, size_t error_size);
    /* 1 accepts the request; 0 replies INVALID without running other RPC hooks. */
    int32_t (*on_request)(void* plugin, const void* packet, size_t packet_size);
    /* Pre-physics state. The host initializes each command array to zero. */
    int32_t (*on_step)(void* plugin, const HakoTickV1* tick, const HakoJointStatesV1* states,
                       uint32_t server_state, HakoJointCommandsV1* commands);
    /* Post-physics state. Returns 1 if the payload was written, 0 on failure. */
    int32_t (*on_response)(void* plugin, const HakoTickV1* tick, const HakoJointStatesV1* states,
                           const HakoServiceReplyV1* reply);
    /* Optional. Called only when an admitted request is canceled. */
    void (*on_cancel)(void* plugin);
    void (*reset)(void* plugin);
    void (*destroy)(void* plugin);

    /*
     * Optional. Runs every physics tick after any RPC on_step and before
     * arbitration, including idle ticks. Same signature as controller update.
     * In this ABI there are no sensor PDU channels (0, null). Returns 0 on
     * success. The manifest priority applies to both command hooks. on_step
     * commands take precedence on joints written by both hooks in one tick.
     * tick.selected describes the preceding arbitration and tick.reset is 1
     * on the first call to each tick-taking hook after create or reset.
     */
    int32_t (*update)(void* plugin, const HakoTickV1* tick, const HakoJointStatesV1* states,
                      uint32_t pdu_count, const HakoPduViewV1* pdus,
                      HakoJointCommandsV1* commands, char* error, size_t error_size);
} HakoServicePluginV1;

#ifdef __cplusplus
}
#endif

#endif /* HAKONIWA_ROBOT_RUNTIME_SERVICE_PLUGIN_H */
