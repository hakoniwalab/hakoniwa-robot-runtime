/*
 * Robot Runtime plugin interface: the part common to every plugin kind (C).
 *
 * Robot Runtime (hakoniwa-robot-runtime) loads shared libraries into its own
 * process. Every plugin exports exactly one function,
 * hako_robot_runtime_plugin_v1, which returns a pointer to a table in static
 * storage. Every table starts with HakoPluginHeaderV1; the host reads `kind`
 * first and only then treats the pointer as the table of that kind
 * (controller_plugin.h for HAKO_PLUGIN_KIND_CONTROLLER).
 *
 * Only C types cross this boundary: no C++ object, no standard-library type,
 * no exception, and no memory allocated on one side and freed on the other.
 * A host and a plugin built with different compilers or Debug/Release
 * settings therefore work together.
 *
 * Validation by the host (a failure refuses the load and names the path):
 *   - the entry symbol exists and returns a non-null header;
 *   - `kind` is a kind this host knows and the kind the manifest component
 *     asked for (a controller component accepts only controller plugins);
 *   - `abi_version` is the version of that kind this host was built with;
 *   - `struct_size` is at least the size of that kind's table as the host
 *     knows it (a plugin may append members; an older host ignores them).
 *
 * Lifecycle (common to every kind; the host calls these in this order):
 *   create     once, while the simulation is stopped, on the creating thread
 *   on_start   at the start event, on the simulation thread
 *   (kind)     the kind's own functions while running, on the simulation thread
 *   on_stop    at the stop event, on the simulation thread
 *   reset      at a reset, on the simulation thread (back to the state right
 *              after create)
 *   destroy    once, at the end, on the creating thread
 * A plugin may own threads (for example for communication), but it never
 * calls into Robot Runtime: the only callback it receives is the log function
 * passed to create. The host keeps the shared library open while any instance
 * created from it still exists.
 *
 * Errors: every function that can fail returns 0 on success. On failure it
 * returns non-zero and writes a NUL-terminated message into the caller's
 * `error` buffer (at most `error_size` bytes, truncated).
 *
 * Memory: whoever allocates frees. Pointers passed across the boundary are
 * valid only during the call that receives them.
 */
#ifndef HAKONIWA_ROBOT_RUNTIME_PLUGIN_COMMON_H
#define HAKONIWA_ROBOT_RUNTIME_PLUGIN_COMMON_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The one exported symbol of every plugin. */
#define HAKO_ROBOT_RUNTIME_PLUGIN_ENTRY_V1 "hako_robot_runtime_plugin_v1"

/* Plugin kinds (HakoPluginHeaderV1.kind). */
#define HAKO_PLUGIN_KIND_CONTROLLER   1u
#define HAKO_PLUGIN_KIND_DATA_ADAPTER 2u /* reserved: no host accepts it yet */

/* Log levels for the log callback passed to create. */
#define HAKO_PLUGIN_LOG_DEBUG 0
#define HAKO_PLUGIN_LOG_INFO  1
#define HAKO_PLUGIN_LOG_WARN  2
#define HAKO_PLUGIN_LOG_ERROR 3

/* Joint command types (HakoJointCommandsV1.command_type). */
#define HAKO_JOINT_COMMAND_POSITION 0u
#define HAKO_JOINT_COMMAND_VELOCITY 1u
#define HAKO_JOINT_COMMAND_EFFORT   2u

#if defined(_WIN32)
#define HAKO_PLUGIN_EXPORT __declspec(dllexport)
#else
#define HAKO_PLUGIN_EXPORT __attribute__((visibility("default")))
#endif

/* First member of every plugin table. */
typedef struct HakoPluginHeaderV1 {
    /* sizeof the whole table (not only this header) as the plugin built it. */
    uint32_t struct_size;
    /* ABI version of the table of this kind (for example
     * HAKO_CONTROLLER_PLUGIN_ABI_VERSION). */
    uint32_t abi_version;
    /* HAKO_PLUGIN_KIND_CONTROLLER or HAKO_PLUGIN_KIND_DATA_ADAPTER. */
    uint32_t kind;
    /* The header revision the plugin was built with, for logs. */
    const char* interface_revision;
    /* Plugin name, for logs. */
    const char* plugin_id;
} HakoPluginHeaderV1;

/* The one exported function: returns the header of a table in static storage. */
typedef const struct HakoPluginHeaderV1* (*HakoRobotRuntimePluginEntryV1)(void);

/* Log callback passed to create. `message` is valid only during the call.
 * It may be called from any thread of the plugin. */
typedef void (*HakoPluginLogFunctionV1)(void* user, int32_t level, const char* message);

/*
 * Joint states (read only, valid during the call). Arrays have `count`
 * elements in the order of the joint names passed to create.
 * position: rad (revolute) or m (prismatic); velocity: rad/s or m/s;
 * effort: N m or N.
 */
typedef struct HakoJointStatesV1 {
    uint32_t count;
    const double* position;
    const double* velocity;
    const double* effort;
} HakoJointStatesV1;

/*
 * Joint commands (arrays owned by the caller, written by the plugin). Arrays
 * have `count` elements in the order of the joint names passed to create.
 * has_command 0 means no command for that joint in this call.
 * command_type is one of HAKO_JOINT_COMMAND_*; value uses the units of
 * HakoJointStatesV1.
 */
typedef struct HakoJointCommandsV1 {
    uint32_t count;
    uint8_t* has_command;
    uint8_t* command_type;
    double* value;
} HakoJointCommandsV1;

/* Time information of one call. */
typedef struct HakoTickV1 {
    /* sizeof(HakoTickV1) as the host built it. */
    uint32_t struct_size;
    /* Simulation time of the state passed in this call [ns]. */
    uint64_t sim_time_ns;
    /* Physics step number: sim_time_ns / physics_step_ns. */
    uint64_t step_index;
    /* Physics step [ns]. */
    uint64_t physics_step_ns;
    /* Controller plugins: 1 when this component's commands were selected in
     * the previous physics step (0 while another control has taken over). */
    uint8_t selected;
    /* 1 on the first call after create or reset. */
    uint8_t reset;
} HakoTickV1;

/* One Hakoniwa PDU as bytes (valid during the call). */
typedef struct HakoPduViewV1 {
    /* Index into the PDU list passed to create. */
    uint32_t channel;
    uint32_t size;
    const void* data;
    /* Simulation time the value was measured at [ns]. */
    uint64_t sample_time_ns;
} HakoPduViewV1;

#ifdef __cplusplus
}
#endif

#endif /* HAKONIWA_ROBOT_RUNTIME_PLUGIN_COMMON_H */
