/*
 * Sample controller plugin (C, no ROS): position commands toward fixed
 * targets.
 *
 * Config file (the plugin's own format), one "key values..." per line:
 *   period_ns 10000000         control period (default 0: every step)
 *   target 0.5 -0.25 ...       one position target per joint (default: none,
 *                              hold the position seen after create/reset)
 *   fail_on_update 3           make the 3rd update fail (tests)
 *   fail_on_start 1            make on_start fail (tests)
 *   fail_on_stop 1             make on_stop fail (tests)
 *   fail_on_reset 1            make the 1st reset fail (tests)
 *
 * Behaviour:
 *   - tick.reset = 1: hold the current position (no jump after a reset).
 *   - otherwise: command the targets, or keep holding when none are given.
 *   - tick.selected falls from 1 to 0 (another control took over): drop the
 *     targets and hold the current position from then on, like a cancelled
 *     trajectory, until the next reset.
 *
 * Built in variants for the loader tests (compile definitions):
 *   HOLD_TARGET_BAD_KIND, HOLD_TARGET_BAD_ABI, HOLD_TARGET_SMALL_TABLE,
 *   HOLD_TARGET_NO_ENTRY.
 *
 * hold_target_plugin_test_stats() exposes what the last update saw, so tests
 * can check the host's calls; a real plugin has no such function.
 */
#include "hakoniwa/robot_runtime/plugin/controller_plugin.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HOLD_TARGET_MAX_JOINTS 64
#define HOLD_TARGET_MAX_PDUS 4
#define HOLD_TARGET_NAME_SIZE 64

typedef struct HoldTargetStats {
    uint64_t update_count;
    uint64_t reset_count;
    uint64_t last_step_index;
    uint64_t last_sim_time_ns;
    uint8_t last_selected;
    uint8_t last_reset;
    /* Lifecycle events. */
    uint64_t start_count;
    uint64_t stop_count;
    /* Sensor PDUs: create's channel list, the last update's views. */
    uint32_t pdu_channel_count;
    char pdu_names[HOLD_TARGET_MAX_PDUS][HOLD_TARGET_NAME_SIZE];
    char pdu_types[HOLD_TARGET_MAX_PDUS][HOLD_TARGET_NAME_SIZE];
    uint32_t last_pdu_count;
    uint32_t last_pdu_channel[HOLD_TARGET_MAX_PDUS];
    uint32_t last_pdu_size[HOLD_TARGET_MAX_PDUS];
    uint8_t last_pdu_first_byte[HOLD_TARGET_MAX_PDUS];
    uint64_t last_pdu_sample_time_ns[HOLD_TARGET_MAX_PDUS];
    uint64_t total_pdu_count;
} HoldTargetStats;

typedef struct HoldTarget {
    uint32_t joint_count;
    uint64_t period_ns;
    int has_target;
    double target[HOLD_TARGET_MAX_JOINTS];
    double hold[HOLD_TARGET_MAX_JOINTS];
    int holding;
    uint8_t was_selected;
    uint64_t fail_on_update;
    int fail_on_start;
    int fail_on_stop;
    uint64_t fail_on_reset;
    HakoPluginLogFunctionV1 log;
    void* log_user;
} HoldTarget;

static HoldTargetStats g_stats;

static void write_error(char* error, size_t error_size, const char* message)
{
    if (error != NULL && error_size > 0) {
        snprintf(error, error_size, "%s", message);
    }
}

static int read_config(HoldTarget* self, const char* path, char* error, size_t error_size)
{
    FILE* file;
    char line[1024];

    if (path == NULL || path[0] == '\0') {
        return 0;
    }
    file = fopen(path, "r");
    if (file == NULL) {
        write_error(error, error_size, "hold_target: cannot open config");
        return -1;
    }
    while (fgets(line, sizeof(line), file) != NULL) {
        char* cursor = line;
        char* key = strtok(cursor, " \t\r\n");
        if (key == NULL || key[0] == '#') {
            continue;
        }
        if (strcmp(key, "period_ns") == 0) {
            char* value = strtok(NULL, " \t\r\n");
            self->period_ns = value != NULL ? strtoull(value, NULL, 10) : 0;
        } else if (strcmp(key, "fail_on_update") == 0) {
            char* value = strtok(NULL, " \t\r\n");
            self->fail_on_update = value != NULL ? strtoull(value, NULL, 10) : 0;
        } else if (strcmp(key, "fail_on_start") == 0) {
            char* value = strtok(NULL, " \t\r\n");
            self->fail_on_start = value != NULL ? atoi(value) : 0;
        } else if (strcmp(key, "fail_on_reset") == 0) {
            char* value = strtok(NULL, " \t\r\n");
            self->fail_on_reset = value != NULL ? strtoull(value, NULL, 10) : 0;
        } else if (strcmp(key, "fail_on_stop") == 0) {
            char* value = strtok(NULL, " \t\r\n");
            self->fail_on_stop = value != NULL ? atoi(value) : 0;
        } else if (strcmp(key, "target") == 0) {
            uint32_t count = 0;
            char* value = strtok(NULL, " \t\r\n");
            while (value != NULL && count < HOLD_TARGET_MAX_JOINTS) {
                self->target[count] = strtod(value, NULL);
                count++;
                value = strtok(NULL, " \t\r\n");
            }
            if (count != self->joint_count) {
                fclose(file);
                write_error(error, error_size, "hold_target: target count differs from joint count");
                return -1;
            }
            self->has_target = 1;
        }
    }
    fclose(file);
    return 0;
}

static void* hold_target_create(const HakoControllerCreateV1* args, char* error, size_t error_size)
{
    HoldTarget* self;

    if (args == NULL || args->struct_size < sizeof(HakoControllerCreateV1)) {
        write_error(error, error_size, "hold_target: create arguments too old");
        return NULL;
    }
    if (args->joint_count == 0 || args->joint_count > HOLD_TARGET_MAX_JOINTS) {
        write_error(error, error_size, "hold_target: unsupported joint count");
        return NULL;
    }
    self = (HoldTarget*)calloc(1, sizeof(HoldTarget));
    if (self == NULL) {
        write_error(error, error_size, "hold_target: out of memory");
        return NULL;
    }
    self->joint_count = args->joint_count;
    self->log = args->log;
    self->log_user = args->log_user;
    if (read_config(self, args->config_path, error, error_size) != 0) {
        free(self);
        return NULL;
    }
    if (args->pdu_channel_count > HOLD_TARGET_MAX_PDUS) {
        free(self);
        write_error(error, error_size, "hold_target: too many PDU channels");
        return NULL;
    }
    memset(&g_stats, 0, sizeof(g_stats));
    g_stats.pdu_channel_count = args->pdu_channel_count;
    {
        uint32_t index;
        for (index = 0; index < args->pdu_channel_count; ++index) {
            snprintf(g_stats.pdu_names[index], HOLD_TARGET_NAME_SIZE, "%s", args->pdu_names[index]);
            snprintf(g_stats.pdu_types[index], HOLD_TARGET_NAME_SIZE, "%s", args->pdu_types[index]);
        }
    }
    if (self->log != NULL) {
        self->log(self->log_user, HAKO_PLUGIN_LOG_INFO, "hold_target created");
    }
    return self;
}

static uint64_t hold_target_update_period_ns(void* plugin)
{
    const HoldTarget* self = (const HoldTarget*)plugin;
    return self->period_ns;
}

static int32_t hold_target_on_start(void* plugin, char* error, size_t error_size)
{
    const HoldTarget* self = (const HoldTarget*)plugin;
    g_stats.start_count++;
    if (self->fail_on_start) {
        write_error(error, error_size, "hold_target: injected start failure");
        return -1;
    }
    return 0;
}

static int32_t hold_target_on_stop(void* plugin, char* error, size_t error_size)
{
    const HoldTarget* self = (const HoldTarget*)plugin;
    g_stats.stop_count++;
    if (self->fail_on_stop) {
        write_error(error, error_size, "hold_target: injected stop failure");
        return -1;
    }
    return 0;
}

static void capture_hold(HoldTarget* self, const HakoJointStatesV1* states)
{
    uint32_t index;
    for (index = 0; index < self->joint_count; ++index) {
        self->hold[index] = states->position[index];
    }
    self->holding = 1;
}

static int32_t hold_target_update(void* plugin, const HakoTickV1* tick, const HakoJointStatesV1* states,
                                  uint32_t pdu_count, const HakoPduViewV1* pdus,
                                  HakoJointCommandsV1* commands, char* error, size_t error_size)
{
    HoldTarget* self = (HoldTarget*)plugin;
    uint32_t index;
    const double* source;

    g_stats.update_count++;
    g_stats.last_pdu_count = pdu_count;
    g_stats.total_pdu_count += pdu_count;
    for (index = 0; index < pdu_count && index < HOLD_TARGET_MAX_PDUS; ++index) {
        g_stats.last_pdu_channel[index] = pdus[index].channel;
        g_stats.last_pdu_size[index] = pdus[index].size;
        g_stats.last_pdu_first_byte[index] =
            pdus[index].size > 0 ? ((const uint8_t*)pdus[index].data)[0] : 0;
        g_stats.last_pdu_sample_time_ns[index] = pdus[index].sample_time_ns;
    }
    g_stats.last_step_index = tick->step_index;
    g_stats.last_sim_time_ns = tick->sim_time_ns;
    g_stats.last_selected = tick->selected;
    g_stats.last_reset = tick->reset;

    if (self->fail_on_update != 0 && g_stats.update_count == self->fail_on_update) {
        write_error(error, error_size, "hold_target: injected failure");
        return -1;
    }
    if (states->count != self->joint_count || commands->count != self->joint_count) {
        write_error(error, error_size, "hold_target: joint count changed");
        return -1;
    }

    if (tick->reset) {
        /* Right after create/reset: hold where the robot is, then (next
         * update) move to the targets. */
        capture_hold(self, states);
        source = self->hold;
        self->holding = self->has_target ? 0 : 1;
    } else {
        if (self->was_selected && !tick->selected) {
            /* Taken over: drop the targets, hold the current position. */
            capture_hold(self, states);
        }
        source = (self->holding || !self->has_target) ? self->hold : self->target;
    }
    self->was_selected = tick->selected;

    for (index = 0; index < self->joint_count; ++index) {
        commands->has_command[index] = 1;
        commands->command_type[index] = (uint8_t)HAKO_JOINT_COMMAND_POSITION;
        commands->value[index] = source[index];
    }
    return 0;
}

static int32_t hold_target_reset(void* plugin, char* error, size_t error_size)
{
    HoldTarget* self = (HoldTarget*)plugin;
    self->holding = 0;
    self->was_selected = 0;
    g_stats.reset_count++;
    if (self->fail_on_reset != 0 && g_stats.reset_count == self->fail_on_reset) {
        write_error(error, error_size, "hold_target: injected reset failure");
        return -1;
    }
    return 0;
}

static void hold_target_destroy(void* plugin)
{
    free(plugin);
}

#if defined(HOLD_TARGET_BAD_KIND)
#define HOLD_TARGET_KIND HAKO_PLUGIN_KIND_DATA_ADAPTER
#else
#define HOLD_TARGET_KIND HAKO_PLUGIN_KIND_CONTROLLER
#endif

#if defined(HOLD_TARGET_BAD_ABI)
#define HOLD_TARGET_ABI 99u
#else
#define HOLD_TARGET_ABI HAKO_CONTROLLER_PLUGIN_ABI_VERSION
#endif

#if defined(HOLD_TARGET_SMALL_TABLE)
#define HOLD_TARGET_TABLE_SIZE ((uint32_t)sizeof(HakoPluginHeaderV1))
#else
#define HOLD_TARGET_TABLE_SIZE ((uint32_t)sizeof(HakoControllerPluginV1))
#endif

static const HakoControllerPluginV1 kHoldTargetPlugin = {
    {
        HOLD_TARGET_TABLE_SIZE,
        HOLD_TARGET_ABI,
        HOLD_TARGET_KIND,
        HAKO_CONTROLLER_PLUGIN_INTERFACE_REVISION,
        "hold_target",
    },
    hold_target_create,
    hold_target_update_period_ns,
    hold_target_on_start,
    hold_target_update,
    hold_target_on_stop,
    hold_target_reset,
    hold_target_destroy,
};

#if defined(HOLD_TARGET_NO_ENTRY)
HAKO_PLUGIN_EXPORT const HakoPluginHeaderV1* hold_target_wrong_entry_name(void)
#else
HAKO_PLUGIN_EXPORT const HakoPluginHeaderV1* hako_robot_runtime_plugin_v1(void)
#endif
{
    return &kHoldTargetPlugin.header;
}

/* Test hook (not part of the plugin interface). */
HAKO_PLUGIN_EXPORT void hold_target_plugin_test_stats(HoldTargetStats* output)
{
    *output = g_stats;
}
