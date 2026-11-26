#include "hakoniwa/robot_runtime/plugin/service_plugin.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct Fixture {
    /* The value records hook order. */
    int phase;
} Fixture;

static void* create(const HakoServiceCreateV1* args, char* error, size_t error_size)
{
    (void)error;
    (void)error_size;
    if (strcmp(args->service_name, "any/route") != 0
        || strcmp(args->config_json, "{\"mode\":\"test\"}") != 0
        || args->joint_count != 1 || strcmp(args->joint_names[0], "joint") != 0
        || args->physics_step_ns != 1000000) {
        return NULL;
    }
    return calloc(1, sizeof(Fixture));
}

static int32_t request(void* plugin, const void* packet, size_t size)
{
    Fixture* fixture = plugin;
    if (size != 1 || *(const uint8_t*)packet > 1 || fixture->phase != 0) {
        return 0;
    }
    fixture->phase = *(const uint8_t*)packet == 0 ? 1 : 4;
    return 1;
}

#ifdef SERVICE_MISSING_STEP
#define step NULL
#else
static int32_t step(void* plugin, const HakoTickV1* tick, const HakoJointStatesV1* states,
                    uint32_t server_state, HakoJointCommandsV1* commands)
{
    Fixture* fixture = plugin;
    if (server_state == HAKO_SERVICE_STATE_CANCELING && fixture->phase == 5) {
        return HAKO_SERVICE_STEP_CANCELED;
    }
    if (server_state == HAKO_SERVICE_STATE_DOING && fixture->phase == 4) {
        return HAKO_SERVICE_STEP_PENDING;
    }
    if (server_state != HAKO_SERVICE_STATE_DOING || fixture->phase != 1
        || tick->physics_step_ns != 1000000 || states->count != 1) {
        return HAKO_SERVICE_STEP_ERROR;
    }
    fixture->phase = 2;
    commands->has_command[0] = 1;
    commands->value[0] = 7.0;
    return HAKO_SERVICE_STEP_OK;
}
#endif

static int32_t response(void* plugin, const HakoTickV1* tick, const HakoJointStatesV1* states,
                        const HakoServiceReplyV1* reply)
{
    Fixture* fixture = plugin;
    if (reply->header_size != 2 || memcmp(reply->header, "OK", 2) != 0
        || reply->metadata_size != 1 || reply->payload_size != 1
        || states->count != 1 || tick->sim_time_ns == 0) {
        return 0;
    }
    if (fixture->phase == 2 && states->position[0] == 7.0) {
        *(uint8_t*)reply->payload = 7;
    } else if (fixture->phase == 5) {
        *(uint8_t*)reply->payload = 5;
    } else {
        return 0;
    }
    *(uint8_t*)reply->metadata = 1;
    fixture->phase = 0;
    return 1;
}

#ifdef SERVICE_NO_OPTIONAL
#define update NULL
#define cancel NULL
#else
#ifdef SERVICE_FAILING_UPDATE
static int32_t update(void* plugin, const HakoTickV1* tick, const HakoJointStatesV1* states,
                      uint32_t pdu_count, const HakoPduViewV1* pdus,
                      HakoJointCommandsV1* commands, char* error, size_t error_size)
{
    (void)plugin;
    (void)tick;
    (void)states;
    (void)pdu_count;
    (void)pdus;
    (void)commands;
    snprintf(error, error_size, "fixture update failed");
    return 1;
}
#else
static int32_t update(void* plugin, const HakoTickV1* tick, const HakoJointStatesV1* states,
                      uint32_t pdu_count, const HakoPduViewV1* pdus,
                      HakoJointCommandsV1* commands, char* error, size_t error_size)
{
    (void)plugin;
    (void)states;
    (void)error;
    (void)error_size;
    if (pdu_count != 0 || pdus != NULL || (tick->step_index == 0 && !tick->reset)) {
        return 1;
    }
    commands->has_command[0] = 1;
    commands->value[0] = tick->selected ? 4.0 : 3.0;
    return 0;
}
#endif

static void cancel(void* plugin)
{
    Fixture* fixture = plugin;
    fixture->phase = fixture->phase == 4 ? 5 : -1;
}
#endif

static void reset(void* plugin)
{
    ((Fixture*)plugin)->phase = 0;
}

static const HakoServicePluginV1 table = {
    {
#ifdef SERVICE_SMALL_TABLE
        sizeof(HakoPluginHeaderV1),
#else
        sizeof(HakoServicePluginV1),
#endif
#ifdef SERVICE_BAD_ABI
        HAKO_SERVICE_PLUGIN_ABI_VERSION + 1,
#else
        HAKO_SERVICE_PLUGIN_ABI_VERSION,
#endif
#ifdef SERVICE_BAD_KIND
        HAKO_PLUGIN_KIND_DATA_ADAPTER,
#else
        HAKO_PLUGIN_KIND_SERVICE,
#endif
        HAKO_SERVICE_PLUGIN_INTERFACE_REVISION, "service-fixture"
    },
    create, request, step, response, cancel, reset, free, update,
};

HAKO_PLUGIN_EXPORT const HakoPluginHeaderV1* hako_robot_runtime_plugin_v1(void)
{
    return &table.header;
}
