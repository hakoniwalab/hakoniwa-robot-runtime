/*
 * Robot Runtime controller plugin interface (C).
 *
 * A controller plugin takes over the control flow of a set of joints: it
 * receives the joint states (and the new sensor PDUs) and returns joint
 * commands. Robot Runtime calls it as one controller of its existing step,
 * next to the built-in controllers; its commands compete in the priority
 * arbitration like theirs.
 *
 * The entry is the common one (plugin_common.h); the header's kind is
 * HAKO_PLUGIN_KIND_CONTROLLER and the table is HakoControllerPluginV1.
 *
 * Lifecycle (one controller instance; plugin_common.h gives the threads):
 *   create            once, while the simulation is stopped
 *   update_period_ns  right after create; the host checks that the period is
 *                     a whole multiple of the physics step (0: every step)
 *                     and refuses the plugin otherwise
 *   on_start          at the start event
 *   update            on the physics steps whose step_index is a multiple of
 *                     period / physics_step; on the steps in between the host
 *                     keeps applying the commands of the last update
 *   on_stop           at the stop event
 *   reset             at a reset; the next update has tick.reset = 1 and
 *                     should return commands that hold the current position
 *   destroy           once, at the end
 * The first update after create also has tick.reset = 1.
 *
 * update is called on every period step, also while another control has
 * taken over (tick.selected = 0), so the plugin keeps receiving the states.
 * A plugin that runs trajectories should drop them when selected falls to 0,
 * so that an old trajectory does not resume after the takeover ends.
 *
 * The joint order of every array is the order of joint_names in create.
 * Memory rules are those of plugin_common.h: commands->* arrays are owned by
 * the host and written by the plugin during update; the plugin owns its
 * instance (create allocates it, destroy frees it).
 */
#ifndef HAKONIWA_ROBOT_RUNTIME_CONTROLLER_PLUGIN_H
#define HAKONIWA_ROBOT_RUNTIME_CONTROLLER_PLUGIN_H

#include "hakoniwa/robot_runtime/plugin/plugin_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HAKO_CONTROLLER_PLUGIN_ABI_VERSION 1u
/* Changes whenever a struct or a function of this header changes. */
#define HAKO_CONTROLLER_PLUGIN_INTERFACE_REVISION "controller-plugin/1"

/* What the host passes to create one controller. */
typedef struct HakoControllerCreateV1 {
    /* sizeof(HakoControllerCreateV1) as the host built it. */
    uint32_t struct_size;
    /* The plugin's own configuration file (the host does not read it). */
    const char* config_path;
    /* The joints this controller commands: Robot Runtime actuator IDs. */
    uint32_t joint_count;
    const char* const* joint_names;
    /* Physics step [ns]. */
    uint64_t physics_step_ns;
    /* Sensor PDUs passed to update (index = HakoPduViewV1.channel). */
    uint32_t pdu_channel_count;
    const char* const* pdu_names;
    const char* const* pdu_types; /* for example "sensor_msgs/PointCloud2" */
    /* Log callback into the Robot Runtime log; may be null. */
    HakoPluginLogFunctionV1 log;
    void* log_user;
} HakoControllerCreateV1;

typedef struct HakoControllerPluginV1 {
    /* header.kind = HAKO_PLUGIN_KIND_CONTROLLER,
     * header.abi_version = HAKO_CONTROLLER_PLUGIN_ABI_VERSION,
     * header.struct_size = sizeof(HakoControllerPluginV1). */
    HakoPluginHeaderV1 header;

    /* Returns the instance, or null with a message in error. */
    void* (*create)(const HakoControllerCreateV1* args, char* error, size_t error_size);
    /* Control period [ns]: a whole multiple of physics_step_ns; 0 = every step. */
    uint64_t (*update_period_ns)(void* plugin);
    int32_t (*on_start)(void* plugin, char* error, size_t error_size);
    /* pdus holds only the PDUs that changed since the previous update, the
     * latest value of each channel. commands arrives with has_command all 0. */
    int32_t (*update)(void* plugin, const HakoTickV1* tick, const HakoJointStatesV1* states,
                      uint32_t pdu_count, const HakoPduViewV1* pdus,
                      HakoJointCommandsV1* commands, char* error, size_t error_size);
    int32_t (*on_stop)(void* plugin, char* error, size_t error_size);
    int32_t (*reset)(void* plugin, char* error, size_t error_size);
    void (*destroy)(void* plugin);
} HakoControllerPluginV1;

#ifdef __cplusplus
}
#endif

#endif /* HAKONIWA_ROBOT_RUNTIME_CONTROLLER_PLUGIN_H */
