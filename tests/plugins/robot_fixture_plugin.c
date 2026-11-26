/*
 * Test fixture for the robot service plugin interface (stage 2c-1).
 * One robot with two services: 0 "move" (request: one signed byte, the joint
 * target in tenths of a radian) and 1 "stop". Built as a shared library and,
 * with HAKO_ROBOT_PLUGIN_BUILTIN, into the test executable.
 *
 * Variants: FIXTURE_BAD_KIND, FIXTURE_BAD_ABI, FIXTURE_SMALL_TABLE,
 * FIXTURE_MISSING_STEP, FIXTURE_NO_CANCEL, FIXTURE_FAILING_STEP.
 */
#include "hakoniwa/robot_runtime/plugin/service_plugin.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct Fixture {
    int moving;        /* 1 while the joint is commanded to target */
    int pending;       /* 1 while service 0 keeps its request open (target > 1.0) */
    double target;
    int last_service;  /* service of the last accepted request */
    double body_x;     /* body 0 position x seen in the last on_step */
} Fixture;

#if !defined(FIXTURE_MISSING_STEP) && !defined(FIXTURE_FAILING_STEP)
static const char* const tyre[] = {"tyre"};
#endif

static int create_args_ok(const HakoRobotCreateV1* a)
{
    return a->struct_size >= sizeof(HakoRobotCreateV1)
        && strcmp(a->robot, "R1") == 0
        && a->actuator_count == 1 && strcmp(a->actuators[0].id, "joint") == 0
        && a->actuators[0].command_type == HAKO_JOINT_COMMAND_POSITION
        && a->actuators[0].minimum == -1.5 && a->actuators[0].maximum == 1.5
        && a->body_count == 1 && strcmp(a->bodies[0], "base") == 0
        && a->humanoid_count == 1 && strcmp(a->humanoids[0], "H") == 0
        && a->service_count == 2
        && strcmp(a->services[0].name, "svc/move") == 0 && strcmp(a->services[0].type, "t/Move") == 0
        && a->services[0].lockstep == 0
        && strcmp(a->services[1].name, "svc/stop") == 0 && strcmp(a->services[1].type, "t/Stop") == 0
        && strcmp(a->config_json, "{\"mode\":\"test\"}") == 0
        && a->physics_step_ns == 1000000u;
}

static HakoRobotPlugin* create(const HakoRobotCreateV1* args, char* error, size_t error_size)
{
    if (!create_args_ok(args)) {
        snprintf(error, error_size, "fixture: unexpected create arguments");
        return NULL;
    }
    return (HakoRobotPlugin*)calloc(1, sizeof(Fixture));
}

static void destroy(HakoRobotPlugin* robot)
{
    free(robot);
}

static int32_t on_request(HakoRobotPlugin* robot, uint32_t service, const HakoServiceRequestV1* request)
{
    Fixture* f = (Fixture*)robot;
    const size_t base = hako_service_packet_base_offset(request->data, request->size);
    if (base == 0u || request->size < base + 1u) {
        return 0;
    }
    const signed char value = *((const signed char*)request->data + base);
    if (service == 0u) {
        if (value > 100) {
            return 0; /* INVALID */
        }
        f->target = value / 10.0;
        f->pending = f->target > 1.0;
    }
    f->last_service = (int)service;
    return 1;
}

#ifndef FIXTURE_NO_CANCEL
static void on_cancel(HakoRobotPlugin* robot, uint32_t service)
{
    Fixture* f = (Fixture*)robot;
    if (service == 0u) {
        f->pending = 0;
        f->moving = 0;
    }
}
#endif

#ifndef FIXTURE_MISSING_STEP
static int32_t on_step(HakoRobotPlugin* robot, const HakoTickV1* tick, const HakoRobotStateV1* state,
                       HakoServiceSlotV1* slots, uint32_t slot_count, HakoRobotOutputV1* output,
                       char* error, size_t error_size)
{
    Fixture* f = (Fixture*)robot;
    (void)tick;
#ifdef FIXTURE_FAILING_STEP
    (void)f;
    (void)state;
    (void)slots;
    (void)slot_count;
    (void)output;
    snprintf(error, error_size, "fixture step failed");
    return 1;
#else
    (void)error;
    (void)error_size;
    if (slot_count != 2u || state->body_count != 1u || state->humanoid_count != 1u
        || output->friction_capacity < 1u || output->humanoid_motion_capacity < 1u) {
        return 1;
    }
    f->body_x = state->bodies[0].position.x;
    if (tick->reset) {
        output->frictions[0].surface_count = 1u;
        output->frictions[0].surfaces = tyre;
        output->frictions[0].sliding_friction = 0.6;
        output->friction_count = 1u;
    }
    if (slots[0].state == HAKO_SERVICE_STATE_CANCELING) {
        slots[0].result = HAKO_SERVICE_RESULT_CANCELED;
    } else if (slots[0].state == HAKO_SERVICE_STATE_DOING && !f->pending) {
        f->moving = 1;
        slots[0].result = HAKO_SERVICE_RESULT_OK;
        output->humanoid_motions[0].humanoid = "H";
        output->humanoid_motions[0].linear_velocity.x = f->target;
        output->humanoid_motions[0].linear_velocity.y = 0.0;
        output->humanoid_motions[0].linear_velocity.z = 0.0;
        output->humanoid_motions[0].angular_velocity.x = 0.0;
        output->humanoid_motions[0].angular_velocity.y = 0.0;
        output->humanoid_motions[0].angular_velocity.z = 0.0;
        output->humanoid_motions[0].animation = "walk";
        output->humanoid_motions[0].ride = "";
        output->humanoid_motion_count = 1u;
    }
    if (slots[1].state == HAKO_SERVICE_STATE_DOING) {
        f->moving = 0;
        slots[1].result = HAKO_SERVICE_RESULT_OK;
    }
    if (f->moving) {
        output->commands.has_command[0] = 1u;
        output->commands.command_type[0] = HAKO_JOINT_COMMAND_POSITION;
        output->commands.value[0] = f->target;
    }
    return 0;
#endif
}
#endif

static int32_t on_response(HakoRobotPlugin* robot, uint32_t service, const HakoTickV1* tick,
                           const HakoRobotStateV1* state, const HakoServiceResponseV1* response)
{
    Fixture* f = (Fixture*)robot;
    (void)tick;
    (void)state;
    const size_t base = hako_service_packet_base_offset(response->data, response->size);
    if (base == 0u || response->size < base + 3u) {
        return 0;
    }
    unsigned char* body = (unsigned char*)response->data + base;
    body[0] = (unsigned char)(0x40u + service);
    body[1] = (unsigned char)(signed char)(f->target * 10.0);
    body[2] = (unsigned char)(f->body_x * 10.0);
    return 1;
}

static void reset(HakoRobotPlugin* robot)
{
    memset(robot, 0, sizeof(Fixture));
}

static const HakoServicePluginV1 table = {
    {
#ifdef FIXTURE_SMALL_TABLE
        sizeof(HakoPluginHeaderV1),
#else
        sizeof(HakoServicePluginV1),
#endif
#ifdef FIXTURE_BAD_ABI
        HAKO_SERVICE_PLUGIN_ABI_VERSION + 1u,
#else
        HAKO_SERVICE_PLUGIN_ABI_VERSION,
#endif
#ifdef FIXTURE_BAD_KIND
        HAKO_PLUGIN_KIND_CONTROLLER,
#else
        HAKO_PLUGIN_KIND_SERVICE,
#endif
        HAKO_SERVICE_PLUGIN_INTERFACE_REVISION, "robot-fixture"
    },
    create, destroy, on_request,
#ifdef FIXTURE_NO_CANCEL
    NULL,
#else
    on_cancel,
#endif
#ifdef FIXTURE_MISSING_STEP
    NULL,
#else
    on_step,
#endif
    on_response, reset,
};

HAKO_ROBOT_PLUGIN_ENTRY(robot_fixture, table)
