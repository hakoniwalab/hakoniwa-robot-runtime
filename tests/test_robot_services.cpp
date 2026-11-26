// Stage 2b: one robot owns several asynchronous services (notes 2026-11-27).
// Written before the implementation. The robot is driven only through the
// pipeline parts, never by calling its hooks directly.
#undef NDEBUG

#include "runtime/actuator_runtime.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runtime/controller/hold_controller.hpp"
#include "runtime/service/async_service_pipeline.hpp"
#include "runtime/service/robot_services.hpp"

#include <cassert>
#include <cstddef>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace hakoniwa::robot_runtime::runtime;

namespace {

constexpr int kServoId = 40;   // Core service id of service 0
constexpr int kStopId = 41;    // Core service id of service 1
constexpr auto kRobotSource = "robot-source";
constexpr auto kRobotController = "robot-controller";
constexpr auto kHold = "hold";
constexpr auto kJoint = "joint";
constexpr std::uint64_t kDelta = 1'000;
constexpr double kRobotVelocity = 1.5;
constexpr double kHoldPosition = 0.25;
const std::vector<std::byte> kRequest {std::byte {7}};

class FakePlant final : public IActuatorPlant {
public:
    std::vector<std::vector<ActuatorCommand>> applied;

    std::uint64_t delta_time_usec() const noexcept override { return kDelta; }

    RobotState read_state() const override
    {
        RobotState state;
        state.sample_time_usec = applied.size() * kDelta;
        state.actuators.push_back({kJoint, kHoldPosition, 0, 0, state.sample_time_usec});
        return state;
    }

    RobotState step(const std::vector<ActuatorCommand>& commands) override
    {
        applied.push_back(commands);
        return read_state();
    }

    RobotState reset(std::uint64_t) override
    {
        applied.clear();
        return read_state();
    }
};

/** Core with several services. Each service id has its own state and replies. */
class FakeServer final : public IAsyncServiceServer {
public:
    struct Service {
        AsyncServiceState state {AsyncServiceState::Idle};
        std::optional<std::vector<std::byte>> request;
        bool cancel {false};
        std::vector<AsyncServiceResultCode> replies;
        std::vector<std::vector<std::byte>> written;
    };
    std::map<int, Service> services {{kServoId, {}}, {kStopId, {}}};

    AsyncServiceEvent poll(int id) noexcept override
    {
        auto& service = services.at(id);
        if (std::exchange(service.cancel, false)) {
            if (service.state == AsyncServiceState::Doing) {
                service.state = AsyncServiceState::Canceling;
            }
            return AsyncServiceEvent::Cancel;
        }
        if (service.request) {
            service.state = AsyncServiceState::Doing;
            return AsyncServiceEvent::Request;
        }
        return AsyncServiceEvent::None;
    }

    std::optional<AsyncServiceState> status(int id) noexcept override
    {
        return services.at(id).state;
    }

    bool get_request(int id, std::vector<std::byte>& bytes) override
    {
        auto& service = services.at(id);
        assert(service.request);
        bytes = std::move(*service.request);
        service.request.reset();
        return true;
    }

    bool reply(int id, AsyncServiceResultCode code, const AsyncServiceResponseWriter& write) override
    {
        auto& service = services.at(id);
        std::vector<std::byte> packet(4, std::byte {0});
        if (write && !write(packet)) {
            return false;
        }
        service.replies.push_back(code);
        service.written.push_back(packet);
        service.state = AsyncServiceState::Idle;
        return true;
    }
};

/** Records every hook call. The test sets what on_step returns per service. */
class FakeRobot final : public IRobotServices {
public:
    struct StepCall {
        std::vector<AsyncServiceState> states;
        std::uint64_t time_usec;
    };

    bool accept {true};
    std::map<std::size_t, AsyncServiceStepOutcome> finish;  // result to set when a slot is not Idle
    bool command_when_idle {false};
    bool result_on_idle {false};  // also set finish[] on Idle slots (the controller must ignore it)
    std::string step_error;       // a robot-level failure reported by on_step
    std::vector<std::size_t> requests;
    std::vector<std::size_t> cancels;
    std::vector<std::size_t> responses;
    std::vector<StepCall> steps;
    int resets {0};

    bool on_request(std::size_t service, std::span<const std::byte> packet) override
    {
        assert(packet.size() == kRequest.size() && packet[0] == kRequest[0]);
        requests.push_back(service);
        return accept;
    }

    void on_cancel(std::size_t service) override { cancels.push_back(service); }

    RobotStepResult on_step(const RobotState& state, const RuntimeStepContext& context,
        std::span<RobotServiceSlot> slots) override
    {
        assert(state.sample_time_usec == context.simulation_time_usec);
        StepCall call {{}, context.simulation_time_usec};
        bool active = false;
        for (std::size_t i = 0; i < slots.size(); ++i) {
            assert(slots[i].result == AsyncServiceStepOutcome::Pending);
            call.states.push_back(slots[i].state);
            if (slots[i].state != AsyncServiceState::Idle) {
                active = true;
            }
            const auto found = finish.find(i);
            if (found != finish.end() && (slots[i].state != AsyncServiceState::Idle || result_on_idle)) {
                slots[i].result = found->second;
            }
        }
        steps.push_back(std::move(call));
        RobotStepResult result;
        result.error = step_error;
        if (active || command_when_idle) {
            // The robot never sets source_id; the controller does.
            result.commands.push_back({kJoint, ActuatorCommandType::Velocity, kRobotVelocity, {},
                context.simulation_time_usec, context.simulation_time_usec + kDelta});
        }
        return result;
    }

    bool on_response(std::size_t service, const RobotState&, const RuntimeStepContext&,
        std::span<std::byte> packet) override
    {
        responses.push_back(service);
        packet[0] = static_cast<std::byte>(0x10 + service);
        return true;
    }

    void reset() override { ++resets; }
};

struct Fixture {
    explicit Fixture(bool stop_is_lockstep = false)
    {
        servo = std::make_shared<AsyncServiceCommandSource>(
            "robot/servo", kServoId, server, robot, 0);
        stop = std::make_shared<AsyncServiceCommandSource>(
            "robot/stop", kStopId, server, robot, 1, stop_is_lockstep);
        source = std::make_shared<RobotServiceSource>(kRobotSource,
            std::vector<std::shared_ptr<AsyncServiceCommandSource>> {servo, stop});
        controller = std::make_shared<RobotServiceController>(kRobotController, source, robot);
        runtime.emplace(
            std::vector<std::shared_ptr<ICommandSource>> {source},
            std::vector<std::shared_ptr<IController>> {controller,
                std::make_shared<HoldController>(kHold, std::vector<HoldActuatorBinding> {{kJoint}})},
            std::make_shared<PriorityCommandArbiter>(std::vector<ControllerPriority> {
                {kRobotController, 10, {}}, {kHold, 0, {}},
            }),
            plant,
            std::vector<std::shared_ptr<IStatePublisher>> {
                std::make_shared<AsyncServiceResponsePublisher>("robot/servo:response", servo),
                std::make_shared<AsyncServiceResponsePublisher>("robot/stop:response", stop),
            });
    }

    std::shared_ptr<FakePlant> plant = std::make_shared<FakePlant>();
    std::shared_ptr<FakeServer> server = std::make_shared<FakeServer>();
    std::shared_ptr<FakeRobot> robot = std::make_shared<FakeRobot>();
    std::shared_ptr<AsyncServiceCommandSource> servo;
    std::shared_ptr<AsyncServiceCommandSource> stop;
    std::shared_ptr<RobotServiceSource> source;
    std::shared_ptr<RobotServiceController> controller;
    std::optional<ActuatorRuntime> runtime;

    FakeServer::Service& core(int id) { return server->services.at(id); }
};

using States = std::vector<AsyncServiceState>;
constexpr auto Idle = AsyncServiceState::Idle;
constexpr auto Doing = AsyncServiceState::Doing;
constexpr auto Canceling = AsyncServiceState::Canceling;

void assert_robot_command(const std::vector<ActuatorCommand>& commands)
{
    assert(commands.size() == 1);
    assert(commands.front().actuator_id == kJoint);
    assert(commands.front().value == kRobotVelocity);
    assert(commands.front().source_id == kRobotSource);
}

void assert_hold_command(const std::vector<ActuatorCommand>& commands)
{
    assert(commands.size() == 1);
    assert(commands.front().type == ActuatorCommandType::Position);
    assert(commands.front().value == kHoldPosition);
}

// on_step runs once per physics step for the robot, also with no request.
void on_step_runs_every_step_with_all_slots_idle()
{
    Fixture f;
    for (int i = 0; i < 3; ++i) {
        (void)f.runtime->step_hakoniwa(1);
    }
    assert(f.robot->steps.size() == 3);
    for (std::size_t i = 0; i < 3; ++i) {
        assert(f.robot->steps[i].states == States({Idle, Idle}));
        assert(f.robot->steps[i].time_usec == i * kDelta);
    }
    assert(f.robot->requests.empty() && f.robot->responses.empty());
    assert_hold_command(f.plant->applied.back());
}

// An idle robot with no commands leaves the controller waiting (hold wins).
void idle_robot_without_commands_is_waiting()
{
    Fixture f;
    const auto report = f.runtime->step_hakoniwa(1);
    assert(report.controller_statuses[0].state == ComponentState::WaitingForInput);
    assert_hold_command(f.plant->applied.back());
}

// An idle robot may still command (a motion that outlives its reply).
void idle_robot_commands_are_applied_with_the_robot_source_id()
{
    Fixture f;
    f.robot->command_when_idle = true;
    const auto report = f.runtime->step_hakoniwa(1);
    assert(report.controller_statuses[0].state == ComponentState::Ready);
    assert_robot_command(f.plant->applied.back());
}

// A request on service 1 reaches the robot as service 1 and is answered on its Core id.
void request_is_routed_by_service_index()
{
    Fixture f;
    f.robot->finish[1] = AsyncServiceStepOutcome::Ok;
    f.core(kStopId).request = kRequest;
    (void)f.runtime->step_hakoniwa(1);
    assert(f.robot->requests == std::vector<std::size_t> {1});
    assert(f.robot->steps.back().states == States({Idle, Doing}));
    assert(f.robot->responses == std::vector<std::size_t> {1});
    assert(f.core(kStopId).replies == std::vector {AsyncServiceResultCode::Ok});
    assert(f.core(kStopId).written.back()[0] == std::byte {0x11});
    assert(f.core(kServoId).replies.empty());
    assert_robot_command(f.plant->applied.back());
}

// Two services active at once: each slot shows its own state; results go back to each.
void two_active_services_keep_their_own_state_and_result()
{
    Fixture f;
    f.core(kServoId).request = kRequest;
    (void)f.runtime->step_hakoniwa(1);
    assert(f.robot->steps.back().states == States({Doing, Idle}));
    assert(f.core(kServoId).replies.empty());

    f.robot->finish[1] = AsyncServiceStepOutcome::Ok;
    f.core(kStopId).request = kRequest;
    (void)f.runtime->step_hakoniwa(1);
    assert(f.robot->steps.back().states == States({Doing, Doing}));
    assert(f.core(kStopId).replies == std::vector {AsyncServiceResultCode::Ok});
    assert(f.core(kServoId).replies.empty());

    f.robot->finish[0] = AsyncServiceStepOutcome::Ok;
    (void)f.runtime->step_hakoniwa(1);
    assert(f.robot->steps.back().states == States({Doing, Idle}));
    assert(f.core(kServoId).replies == std::vector {AsyncServiceResultCode::Ok});
    assert(f.robot->responses == std::vector<std::size_t>({1, 0}));
}

// A cancel on service 0 calls on_cancel(0); the slot turns Canceling; CANCELED goes to its id.
void cancel_is_routed_by_service_index()
{
    Fixture f;
    f.core(kServoId).request = kRequest;
    (void)f.runtime->step_hakoniwa(1);
    f.core(kServoId).cancel = true;
    (void)f.runtime->step_hakoniwa(1);
    assert(f.robot->cancels == std::vector<std::size_t> {0});
    assert(f.robot->steps.back().states == States({Canceling, Idle}));

    f.robot->finish[0] = AsyncServiceStepOutcome::Canceled;
    (void)f.runtime->step_hakoniwa(1);
    assert(f.core(kServoId).replies == std::vector {AsyncServiceResultCode::Canceled});
    assert(f.core(kStopId).replies.empty());
}

// A rejected request is answered INVALID without on_response; its slot never turns Doing.
void rejected_request_skips_the_slot_and_the_response()
{
    Fixture f;
    f.robot->accept = false;
    f.core(kServoId).request = kRequest;
    (void)f.runtime->step_hakoniwa(1);
    assert(f.robot->requests == std::vector<std::size_t> {0});
    assert(f.robot->steps.back().states == States({Idle, Idle}));
    assert(f.robot->responses.empty());
    assert(f.core(kServoId).replies == std::vector {AsyncServiceResultCode::Invalid});
}

// A result set on an idle slot is ignored (nothing to finish).
void result_on_an_idle_slot_is_ignored()
{
    Fixture f;
    f.robot->finish[0] = AsyncServiceStepOutcome::Ok;
    f.robot->result_on_idle = true;
    (void)f.runtime->step_hakoniwa(2);
    assert(f.core(kServoId).replies.empty() && f.core(kStopId).replies.empty());
    assert(f.robot->responses.empty());
}

// A robot failure without commands is a controller error that keeps its reason.
void robot_error_without_commands_is_an_error()
{
    Fixture f;
    f.robot->step_error = "periodic hook failed";
    const auto report = f.runtime->step_hakoniwa(1);
    assert(report.controller_statuses[0].state == ComponentState::Error);
    assert(report.controller_statuses[0].detail == "periodic hook failed");
    assert_hold_command(f.plant->applied.back());
}

// A robot failure with commands degrades the controller; the commands still apply.
void robot_error_with_commands_is_degraded()
{
    Fixture f;
    f.robot->step_error = "periodic hook failed";
    f.robot->command_when_idle = true;
    const auto report = f.runtime->step_hakoniwa(1);
    assert(report.controller_statuses[0].state == ComponentState::Degraded);
    assert(report.controller_statuses[0].detail == "periodic hook failed");
    assert_robot_command(f.plant->applied.back());
}

// A service error outranks a robot failure, as the RPC error did before.
void service_error_outranks_robot_error()
{
    Fixture f;
    f.core(kStopId).request = kRequest;
    (void)f.runtime->step_hakoniwa(1);
    f.core(kStopId).state = Idle;
    f.robot->step_error = "periodic hook failed";
    const auto report = f.runtime->step_hakoniwa(1);
    assert(report.controller_statuses[0].state == ComponentState::Error);
    assert(report.controller_statuses[0].detail != "periodic hook failed");
}

// Any lockstep service makes a controller error stop the run.
void lockstep_service_makes_errors_stop_the_run()
{
    Fixture plain;
    assert(!plain.controller->error_stops_run());
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP) && HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP
    Fixture lockstep(true);
    assert(lockstep.controller->error_stops_run());
#endif
}

// Reset calls the robot once and drops every pending completion.
void reset_resets_the_robot_once_and_every_service()
{
    Fixture f;
    f.core(kServoId).request = kRequest;
    f.core(kStopId).request = kRequest;
    (void)f.runtime->step_hakoniwa(1);
    assert(f.robot->steps.back().states == States({Doing, Doing}));
    f.core(kServoId).state = Idle;
    f.core(kStopId).state = Idle;
    (void)f.runtime->reset();
    assert(f.robot->resets == 1);
    (void)f.runtime->step_hakoniwa(1);
    assert(f.robot->steps.back().states == States({Idle, Idle}));
    assert(f.core(kServoId).replies.empty() && f.core(kStopId).replies.empty());
}

// A Core state that disagrees with the expected one is a controller error.
void server_state_mismatch_is_a_controller_error()
{
    Fixture f;
    f.core(kStopId).request = kRequest;
    (void)f.runtime->step_hakoniwa(1);
    f.core(kStopId).state = Idle;  // Core lost the request behind the robot's back
    const auto report = f.runtime->step_hakoniwa(1);
    assert(report.controller_statuses[0].state == ComponentState::Error);
}

} // namespace

int main()
{
    on_step_runs_every_step_with_all_slots_idle();
    idle_robot_without_commands_is_waiting();
    idle_robot_commands_are_applied_with_the_robot_source_id();
    request_is_routed_by_service_index();
    two_active_services_keep_their_own_state_and_result();
    cancel_is_routed_by_service_index();
    rejected_request_skips_the_slot_and_the_response();
    result_on_an_idle_slot_is_ignored();
    robot_error_without_commands_is_an_error();
    robot_error_with_commands_is_degraded();
    service_error_outranks_robot_error();
    lockstep_service_makes_errors_stop_the_run();
    reset_resets_the_robot_once_and_every_service();
    server_state_mismatch_is_a_controller_error();
    std::cout << "robot services: all passed" << std::endl;
    return 0;
}
