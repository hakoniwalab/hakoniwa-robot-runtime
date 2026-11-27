/*
 * Robot Runtime robot service plugin interface (C).
 *
 * One plugin definition serves one kind of robot. The framework creates one
 * instance per robot (pdu_robot) and passes it back to every hook. A robot
 * owns several RPC services; the service index is the position in the
 * robot's service list given to create. The same source builds either as a
 * shared library (the default: the exported entry hako_robot_runtime_plugin_v1)
 * or into Robot Runtime (with HAKO_ROBOT_PLUGIN_BUILTIN: a named entry that the
 * builtin list refers to). The framework validates the table the same way in
 * both cases.
 *
 * Hooks run synchronously on the simulation thread between create and
 * destroy. A plugin never calls into Robot Runtime, Hakoniwa Core or a
 * transport. Pointers passed in are valid only during the call. Strings and
 * arrays a plugin writes into an output stay owned by the plugin and must stay
 * valid until the hook returns; the framework copies them right after.
 */
#ifndef HAKONIWA_ROBOT_RUNTIME_SERVICE_PLUGIN_H
#define HAKONIWA_ROBOT_RUNTIME_SERVICE_PLUGIN_H

#include "hakoniwa/robot_runtime/plugin/plugin_common.h"

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HAKO_PLUGIN_KIND_SERVICE 3u
#define HAKO_SERVICE_PLUGIN_ABI_VERSION 1u
#define HAKO_SERVICE_PLUGIN_INTERFACE_REVISION "service-plugin/1-robot-declaration"

/* HakoServiceSlotV1.state, set by the framework. */
#define HAKO_SERVICE_STATE_IDLE      0u
#define HAKO_SERVICE_STATE_DOING     1u
#define HAKO_SERVICE_STATE_CANCELING 2u

/* HakoServiceSlotV1.result, set by the plugin for a non-Idle slot. */
#define HAKO_SERVICE_RESULT_PENDING  0u
#define HAKO_SERVICE_RESULT_OK       1u
#define HAKO_SERVICE_RESULT_CANCELED 2u
#define HAKO_SERVICE_RESULT_ERROR    3u

/* The plugin's own control data for one robot. Only the plugin knows it. */
typedef struct HakoRobotPlugin HakoRobotPlugin;

typedef struct HakoVector3V1 {
    double x;
    double y;
    double z;
} HakoVector3V1;

typedef struct HakoQuaternionV1 {
    double x;
    double y;
    double z;
    double w;
} HakoQuaternionV1;

typedef struct HakoEulerV1 {
    double roll;
    double pitch;
    double yaw;
} HakoEulerV1;

/* ---- create ---------------------------------------------------------- */

/* One actuator of the robot, in manifest order (the order of every joint array). */
typedef struct HakoRobotActuatorV1 {
    const char* id;
    uint8_t command_type; /* HAKO_JOINT_COMMAND_* */
    double minimum;       /* spec.limit of the actuator config */
    double maximum;
} HakoRobotActuatorV1;

/* One RPC service of the robot. Its index is its position in the list. */
typedef struct HakoRobotServiceV1 {
    const char* name; /* Core service name */
    const char* type; /* service.json type, for example my_robot_srvs_msgs/ServoJ */
    uint8_t lockstep;
} HakoRobotServiceV1;

typedef struct HakoRobotCreateV1 {
    uint32_t struct_size;
    const char* robot; /* the robot name (pdu_robot) */
    uint32_t actuator_count;
    const HakoRobotActuatorV1* actuators;
    uint32_t body_count; /* bodies whose state the robot reads, in this order */
    const char* const* bodies;
    uint32_t humanoid_count; /* humanoids whose state the robot reads, in this order */
    const char* const* humanoids;
    uint32_t pdu_count; /* PDU inputs, in this order (HakoPduViewV1.channel) */
    const char* const* pdus;
    uint32_t service_count;
    const HakoRobotServiceV1* services;
    const char* config_json; /* the robot's config from the manifest, "{}" if none */
    uint64_t physics_step_ns;
    HakoPluginLogFunctionV1 log;
    void* log_user;
} HakoRobotCreateV1;

/* ---- state ----------------------------------------------------------- */

typedef struct HakoBodyStateV1 {
    HakoVector3V1 position; /* m, world axes */
    HakoQuaternionV1 orientation;
    HakoVector3V1 linear_velocity;  /* m/s */
    HakoVector3V1 angular_velocity; /* rad/s */
} HakoBodyStateV1;

typedef struct HakoHumanoidStateV1 {
    HakoVector3V1 position;
    HakoQuaternionV1 orientation;
    HakoVector3V1 linear_velocity;
    HakoVector3V1 angular_velocity;
    uint32_t limb_count;
    const double* limb_positions; /* rad, in the humanoid type's fixed order */
    uint32_t last_motion_result;  /* 0 = the last motion was accepted */
} HakoHumanoidStateV1;

typedef struct HakoRobotStateV1 {
    uint32_t struct_size;
    HakoJointStatesV1 joints; /* create's actuators order */
    uint32_t body_count;      /* create's bodies order */
    const HakoBodyStateV1* bodies;
    uint32_t humanoid_count;  /* create's humanoids order */
    const HakoHumanoidStateV1* humanoids;
    uint32_t pdu_count;       /* the PDU inputs that have a value, any order */
    const HakoPduViewV1* pdus;
} HakoRobotStateV1;

/* ---- output (Plant operations defined by the common layer) ------------ */

typedef struct HakoSurfaceFrictionV1 {
    uint32_t surface_count;
    const char* const* surfaces;
    double sliding_friction;
} HakoSurfaceFrictionV1;

typedef struct HakoBodyMotionV1 {
    const char* body;
    HakoVector3V1 position;
    HakoEulerV1 orientation;
    HakoVector3V1 linear_velocity;
    HakoVector3V1 angular_velocity;
} HakoBodyMotionV1;

typedef struct HakoHumanoidMotionV1 {
    const char* humanoid;
    HakoVector3V1 linear_velocity;
    HakoVector3V1 angular_velocity;
    const char* animation;
    const char* ride;
} HakoHumanoidMotionV1;

/*
 * Arrays owned by the framework. The plugin writes at most `*_capacity`
 * elements and sets `*_count`. Joint commands go through arbitration; the
 * operations do not. The framework stamps operations with the step time.
 */
typedef struct HakoRobotOutputV1 {
    HakoJointCommandsV1 commands;
    uint32_t friction_capacity;
    uint32_t friction_count;
    HakoSurfaceFrictionV1* frictions;
    uint32_t body_motion_capacity;
    uint32_t body_motion_count;
    HakoBodyMotionV1* body_motions;
    uint32_t humanoid_motion_capacity;
    uint32_t humanoid_motion_count;
    HakoHumanoidMotionV1* humanoid_motions;
} HakoRobotOutputV1;

/* ---- packets (whole PDU, metadata first) ------------------------------ */

typedef struct HakoServiceRequestV1 {
    const void* data;
    size_t size;
} HakoServiceRequestV1;

/* Core's prepared reply buffer: keep the metadata base offset and the service header. */
typedef struct HakoServiceResponseV1 {
    void* data;
    size_t size;
} HakoServiceResponseV1;

/* Offset of the packet base (the C struct of the packet) after the metadata. 0 if too short. */
static inline size_t hako_service_packet_base_offset(const void* data, size_t size)
{
    uint32_t base_off = 0;
    if (data == NULL || size < 12u) {
        return 0u;
    }
    memcpy(&base_off, (const char*)data + 8u, sizeof(base_off));
    return base_off < size ? (size_t)base_off : 0u;
}

typedef struct HakoServiceSlotV1 {
    uint8_t state;  /* HAKO_SERVICE_STATE_* */
    uint8_t result; /* HAKO_SERVICE_RESULT_*, PENDING on entry */
} HakoServiceSlotV1;

/* ---- table ------------------------------------------------------------ */

typedef struct HakoServicePluginV1 {
    HakoPluginHeaderV1 header;

    /* NUL-terminated JSON in static storage: service_types and optional plant_operations. */
    const char* declaration_json;

    /* The robot's control data, or NULL with an optional NUL-terminated error. */
    HakoRobotPlugin* (*create)(const HakoRobotCreateV1* args, char* error, size_t error_size);
    void (*destroy)(HakoRobotPlugin* robot);

    /* 1 accepts the request; 0 replies INVALID. */
    int32_t (*on_request)(HakoRobotPlugin* robot, uint32_t service, const HakoServiceRequestV1* request);
    /* Optional (NULL). An admitted request of this service was canceled. */
    void (*on_cancel)(HakoRobotPlugin* robot, uint32_t service);

    /*
     * Every physics step, once per robot, also with no request. Pre-physics
     * state. Sets the result of each non-Idle slot and fills the output.
     * Returns 0 on success; otherwise a failure with a reason in `error`.
     */
    int32_t (*on_step)(HakoRobotPlugin* robot, const HakoTickV1* tick, const HakoRobotStateV1* state,
                       HakoServiceSlotV1* slots, uint32_t slot_count, HakoRobotOutputV1* output,
                       char* error, size_t error_size);

    /* Post-physics state. Writes the reply body (and its metadata). 1 = written, 0 = failure. */
    int32_t (*on_response)(HakoRobotPlugin* robot, uint32_t service, const HakoTickV1* tick,
                           const HakoRobotStateV1* state, const HakoServiceResponseV1* response);

    /* Back to the state right after create. */
    void (*reset)(HakoRobotPlugin* robot);
} HakoServicePluginV1;

#ifdef __cplusplus
#define HAKO_ROBOT_PLUGIN_EXTERN_C extern "C"
#else
#define HAKO_ROBOT_PLUGIN_EXTERN_C
#endif

/*
 * The one entry of a plugin source. `table` is a HakoServicePluginV1 in static
 * storage. Default: the exported shared library entry. With
 * HAKO_ROBOT_PLUGIN_BUILTIN: hako_robot_plugin_<name>(), listed by the builtin list.
 */
#if defined(HAKO_ROBOT_PLUGIN_BUILTIN)
#define HAKO_ROBOT_PLUGIN_ENTRY(name, table) \
    HAKO_ROBOT_PLUGIN_EXTERN_C const HakoPluginHeaderV1* hako_robot_plugin_##name(void) \
    { \
        return &(table).header; \
    }
#else
#define HAKO_ROBOT_PLUGIN_ENTRY(name, table) \
    HAKO_ROBOT_PLUGIN_EXTERN_C HAKO_PLUGIN_EXPORT const HakoPluginHeaderV1* hako_robot_runtime_plugin_v1(void) \
    { \
        return &(table).header; \
    }
#endif

#ifdef __cplusplus
}
#endif

#endif /* HAKONIWA_ROBOT_RUNTIME_SERVICE_PLUGIN_H */
