#undef NDEBUG  // Keep the contract checks in Release builds.

#include "runtime/service/async_service_pipeline.hpp"
#include "support/single_service_robot.hpp"
#include "runner/lockstep_gate.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP) && HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP
#include "runtime/actuator_runtime.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"

#include <chrono>
#include <initializer_list>
#include <utility>
#endif

using namespace hakoniwa::robot_runtime::runtime;
using namespace hakoniwa::robot_runtime::runner;
namespace test_support = hakoniwa::robot_runtime::test_support;

namespace {

enum class Call { Request, Cancel, Step, Physics, Response, Reply };
using Trace = std::vector<Call>;

class FakeServer final : public IAsyncServiceServer {
public:
    FakeServer(int id, Trace& trace) : id_(id), trace_(trace) {}

    int polls {0};
    int reads {0};
    int busy_replies {0};
    int canceled_replies {0};
    bool cancel_pending {false};
    std::vector<std::array<std::byte, 2>> replies;

    void queue(std::byte request)
    {
        assert(!pending_);
        pending_ = request;
    }

    AsyncServiceEvent poll(int service_id) noexcept override
    {
        assert(service_id == id_);
        ++polls;
        if (cancel_pending) {
            assert(state_ == AsyncServiceState::Doing);
            cancel_pending = false;
            state_ = AsyncServiceState::Canceling;
            return AsyncServiceEvent::Cancel;
        }
        if (!pending_) {
            return AsyncServiceEvent::None;
        }
        if (state_ == AsyncServiceState::Doing) {
            // Model Core before #122: its BUSY reply leaves the server IDLE.
            ++busy_replies;
            pending_.reset();
            state_ = AsyncServiceState::Idle;
            return AsyncServiceEvent::None;
        }
        state_ = AsyncServiceState::Doing;
        return AsyncServiceEvent::Request;
    }

    std::optional<AsyncServiceState> status(int service_id) noexcept override
    {
        assert(service_id == id_);
        return state_;
    }

    bool get_request(int service_id, std::vector<std::byte>& request) override
    {
        assert(service_id == id_ && state_ == AsyncServiceState::Doing && pending_);
        ++reads;
        request = {*pending_};
        pending_.reset();
        return true;
    }

    bool reply(
        int service_id, AsyncServiceResultCode code,
        const AsyncServiceResponseWriter& write) override
    {
        assert(service_id == id_);
        if (code == AsyncServiceResultCode::Canceled) {
            assert(state_ == AsyncServiceState::Canceling);
            if (write) {
                std::array<std::byte, 2> packet {};
                if (!write(packet)) return false;
            }
            trace_.push_back(Call::Reply);
            ++canceled_replies;
            state_ = AsyncServiceState::Idle;
            return true;
        }
        assert(code == AsyncServiceResultCode::Ok && write);
        if (state_ != AsyncServiceState::Doing) {
            return false;
        }
        std::array<std::byte, 2> packet {};
        if (!write(packet)) {
            return false;
        }
        trace_.push_back(Call::Reply);
        replies.push_back(packet);
        state_ = AsyncServiceState::Idle;
        return true;
    }

private:
    int id_;
    Trace& trace_;
    AsyncServiceState state_ {AsyncServiceState::Idle};
    std::optional<std::byte> pending_;
};

constexpr std::uint64_t kDelta = 1'000;

class FakeService final : public test_support::OneService {
public:
    FakeService(std::string joint, Trace& trace, bool finish_request = true)
        : joint_(std::move(joint)), trace_(trace), finish_request_(finish_request) {}

    bool on_request(std::span<const std::byte> request) override
    {
        trace_.push_back(Call::Request);
        assert(request.size() == 1);
        value_ = request.front();
        return true;
    }

    void on_cancel() override { trace_.push_back(Call::Cancel); }

    test_support::OneServiceStepResult on_step(
        const RobotState& state, AsyncServiceState server_state,
        const RuntimeStepContext& context) override
    {
        trace_.push_back(Call::Step);
        assert(state.sample_time_usec == context.simulation_time_usec);
        if (server_state == AsyncServiceState::Canceling) {
            assert(!finish_request_);
            return {AsyncServiceStepOutcome::Canceled, {}};
        }
        assert(server_state == AsyncServiceState::Doing);
        if (!finish_request_) {
            return {AsyncServiceStepOutcome::Pending, {}};
        }
        return {
            AsyncServiceStepOutcome::Ok,
            {{joint_, ActuatorCommandType::Velocity, double(std::to_integer<int>(value_)),
                {}, context.simulation_time_usec, context.simulation_time_usec + kDelta}},
        };
    }

    bool on_response(
        const RobotState& state, const RuntimeStepContext& context,
        std::span<std::byte> packet) override
    {
        trace_.push_back(Call::Response);
        assert(packet.size() == 2 && state.sample_time_usec == context.simulation_time_usec);
        packet[0] = value_;
        packet[1] = std::byte(state.sample_time_usec / kDelta);
        return true;
    }

    void reset() override {}

private:
    std::string joint_;
    Trace& trace_;
    std::byte value_ {};
    bool finish_request_;
};

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP) && HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP

using namespace std::chrono_literals;

class FakePlant final : public IActuatorPlant {
public:
    explicit FakePlant(Trace& trace) : trace_(trace) {}

    std::vector<std::vector<ActuatorCommand>> applied;
    std::uint64_t delta_time_usec() const noexcept override { return kDelta; }
    RobotState read_state() const override
    {
        RobotState state;
        state.sample_time_usec = time_usec_;
        return state;
    }
    RobotState step(const std::vector<ActuatorCommand>& commands) override
    {
        trace_.push_back(Call::Physics);
        time_usec_ += kDelta;
        applied.push_back(commands);
        return read_state();
    }
    RobotState reset(std::uint64_t time_usec) override
    {
        time_usec_ = time_usec;
        applied.clear();
        return read_state();
    }

private:
    Trace& trace_;
    std::uint64_t time_usec_ {0};
};

class FakePduInput final : public ILockstepInput {
public:
    bool lockstep_ready() noexcept override
    {
        ++polls;
        return arrived;
    }

    bool arrived {false};
    int polls {0};
};

struct Fixture {
    Trace trace;
    LockstepGate::Clock::time_point now {};
    std::shared_ptr<FakePlant> plant = std::make_shared<FakePlant>(trace);
    std::vector<std::shared_ptr<FakeServer>> servers;
    std::shared_ptr<FakePduInput> pdu;
    std::unique_ptr<LockstepGate> gate;
    std::unique_ptr<ActuatorRuntime> runtime;

    explicit Fixture(std::initializer_list<bool> lockstep = {true}, bool with_pdu = false,
                     bool finish_request = true)
    {
        std::vector<LockstepGate::Input> gate_inputs;
        std::vector<std::shared_ptr<ICommandSource>> sources;
        std::vector<std::shared_ptr<IController>> controllers;
        std::vector<std::shared_ptr<IStatePublisher>> publishers;
        std::vector<ControllerPriority> priorities;

        int id = 0;
        for (const bool is_lockstep : lockstep) {
            const auto suffix = std::to_string(++id);
            auto server = std::make_shared<FakeServer>(id, trace);
            auto robot = std::make_shared<test_support::SingleServiceRobot>(
                std::make_shared<FakeService>("joint-" + suffix, trace, finish_request));
            auto service_source = std::make_shared<AsyncServiceCommandSource>(
                "service-" + suffix + "/0", id, server, robot, 0, is_lockstep);
            auto source = std::make_shared<RobotServiceSource>("service-" + suffix, std::vector {service_source});
            auto controller = std::make_shared<RobotServiceController>(
                "controller-" + suffix, source, robot);
            servers.push_back(server);
            if (is_lockstep) {
                gate_inputs.push_back({std::string(source->id()), service_source});
            }
            sources.push_back(source);
            controllers.push_back(controller);
            publishers.push_back(std::make_shared<AsyncServiceResponsePublisher>(
                "publisher-" + suffix, service_source));
            priorities.push_back({std::string(controller->id()), 1, "services"});
        }

        if (with_pdu) {
            pdu = std::make_shared<FakePduInput>();
            gate_inputs.push_back({"Car-1/tire_friction", pdu});
        }
        gate = std::make_unique<LockstepGate>(
            std::move(gate_inputs), LockstepGate::Config {}, [this] { return now; });
        runtime = std::make_unique<ActuatorRuntime>(
            std::move(sources), std::move(controllers),
            std::make_shared<PriorityCommandArbiter>(std::move(priorities)), plant,
            std::move(publishers));
        gate->prepare();
    }

    bool step_if_ready()
    {
        if (!gate->ready_to_step()) {
            return false;
        }
        (void)runtime->step_hakoniwa(1);
        if (pdu) {
            pdu->arrived = false;
        }
        return true;
    }

    void assert_time(std::size_t steps) const
    {
        assert(plant->applied.size() == steps);
        assert(plant->read_state().sample_time_usec == steps * kDelta);
    }
};

void waits_for_pdu_and_service_in_either_order()
{
    for (const bool pdu_first : {true, false}) {
        Fixture f({true}, true);
        auto& server = *f.servers[0];
        if (pdu_first) {
            f.pdu->arrived = true;
        } else {
            server.queue(std::byte {1});
        }
        assert(!f.step_if_ready());
        f.assert_time(0);
        assert(server.replies.empty());
        if (pdu_first) {
            server.queue(std::byte {1});
        } else {
            f.pdu->arrived = true;
        }
        assert(f.step_if_ready());
        f.assert_time(1);
        assert(server.reads == 1 && server.replies.size() == 1);
    }
}

void mixed_timeout_names_only_the_missing_input()
{
    for (const bool pdu_missing : {true, false}) {
        Fixture f({true}, true);
        if (pdu_missing) {
            f.servers[0]->queue(std::byte {1});
        } else {
            f.pdu->arrived = true;
        }
        assert(!f.step_if_ready() && !f.gate->failed());
        f.now += 60s;
        assert(!f.step_if_ready() && f.gate->failed());
        const auto reason = f.gate->failure_reason();
        assert((reason.find("Car-1/tire_friction") != std::string::npos) == pdu_missing);
        assert((reason.find("service-1") != std::string::npos) == !pdu_missing);
        f.assert_time(0);
    }
}

void mixed_pause_holds_both_inputs_and_restarts_the_deadline()
{
    Fixture f({true}, true);
    auto& server = *f.servers[0];
    f.now += 59s;
    assert(!f.step_if_ready());
    f.gate->set_waiting_paused(true);
    server.queue(std::byte {1});
    f.now += 2h;
    assert(!f.step_if_ready() && !f.gate->failed());
    assert(f.pdu->polls == 1 && server.polls == 1);
    f.gate->set_waiting_paused(false);
    assert(!f.step_if_ready());
    f.now += 59s;
    assert(!f.step_if_ready() && !f.gate->failed());
    f.gate->set_waiting_paused(true);
    f.pdu->arrived = true;
    assert(!f.step_if_ready());
    f.assert_time(0);
    f.gate->set_waiting_paused(false);
    assert(f.step_if_ready());
    f.assert_time(1);
}

void pdu_only_keeps_initial_sync_and_has_no_deadline()
{
    const auto pdu = std::make_shared<FakePduInput>();
    LockstepGate gate({{"Car-1/tire_friction", pdu}}, {}, [] {
        assert(false && "PDU-only lockstep must not consult the clock");
        return LockstepGate::Clock::time_point {};
    });
    gate.prepare();
    assert(gate.sleep_before_first_step());
    assert(!gate.ready_to_step() && !gate.failed());
    pdu->arrived = true;
    assert(gate.ready_to_step());
}

void waits_without_a_request()
{
    Fixture f;
    assert(!f.gate->empty());
    assert(!f.step_if_ready());
    f.assert_time(0);
    assert(f.trace.empty());
    assert(f.servers[0]->polls == 1);
    assert(f.servers[0]->reads == 0 && f.servers[0]->replies.empty());
    assert(!f.gate->failed());
}

void consumes_the_staged_request_in_the_same_step()
{
    Fixture f;
    auto& server = *f.servers[0];
    assert(!f.step_if_ready());
    server.queue(std::byte {3});
    const int polls = server.polls;
    assert(f.step_if_ready());
    f.assert_time(1);
    assert(server.polls == polls + 1 && server.reads == 1);
    assert(f.trace == Trace({
        Call::Request, Call::Step, Call::Physics, Call::Response, Call::Reply}));
    const std::vector<std::array<std::byte, 2>> expected {{std::byte {3}, std::byte {1}}};
    assert(server.replies == expected);
    assert(f.plant->applied[0].size() == 1 && f.plant->applied[0][0].value == 3.0);
}

void alternates_requests_and_replies_for_ten_steps()
{
    Fixture f;
    auto& server = *f.servers[0];
    for (int step = 1; step <= 10; ++step) {
        server.queue(std::byte(100 + step));
        assert(f.step_if_ready());
        const auto count = static_cast<std::size_t>(step);
        f.assert_time(count);
        assert(server.polls == step && server.reads == step);
        assert(server.replies.size() == count);
        assert(server.replies.back()[0] == std::byte(100 + step));
        assert(server.replies.back()[1] == std::byte(step));
    }
    assert(!f.step_if_ready());
    f.assert_time(10);
}

void waits_for_every_lockstep_service()
{
    Fixture f {true, true};
    f.servers[0]->queue(std::byte {1});
    assert(!f.step_if_ready());
    assert(!f.step_if_ready());
    f.assert_time(0);
    assert(f.servers[0]->polls == 2 && f.servers[0]->reads == 1);
    assert(f.servers[1]->polls == 2 && f.servers[1]->reads == 0);
    assert(f.trace.empty());

    f.servers[1]->queue(std::byte {2});
    assert(f.step_if_ready());
    f.assert_time(1);
    assert(f.servers[0]->polls == 3 && f.servers[1]->polls == 3);
    assert(f.servers[0]->replies.size() == 1 && f.servers[1]->replies.size() == 1);
}

void applies_separate_initial_and_subsequent_timeouts()
{
    {
        Fixture f {true, true};
        f.servers[0]->queue(std::byte {1});
        assert(!f.step_if_ready());
        f.now += 60s - 1ms;
        assert(!f.step_if_ready() && !f.gate->failed());
        f.now += 1ms;
        assert(!f.step_if_ready() && f.gate->failed());
        assert(f.gate->failure_reason().find("service-2") != std::string::npos);
        assert(f.gate->failure_reason().find("service-1") == std::string::npos);
        f.assert_time(0);
    }

    Fixture f;
    f.servers[0]->queue(std::byte {1});
    assert(f.step_if_ready());
    assert(!f.step_if_ready());
    f.now += 10s - 1ms;
    assert(!f.step_if_ready() && !f.gate->failed());
    f.now += 1ms;
    assert(!f.step_if_ready() && f.gate->failed());
    assert(f.gate->failure_reason().find("service-1") != std::string::npos);
    f.assert_time(1);
}

void pause_blocks_progress_and_restarts_timeout()
{
    Fixture f {true, true};
    f.now += 59s;
    assert(!f.step_if_ready() && !f.gate->failed());
    f.gate->set_waiting_paused(true);
    f.servers[0]->queue(std::byte {1});
    f.now += 2h;
    assert(!f.step_if_ready() && !f.gate->failed());
    assert(f.servers[0]->polls == 1 && f.servers[1]->polls == 1);
    f.assert_time(0);

    f.gate->set_waiting_paused(false);
    assert(!f.step_if_ready() && !f.gate->failed());
    f.now += 59s;
    assert(!f.step_if_ready() && !f.gate->failed());
    f.servers[1]->queue(std::byte {2});
    f.gate->set_waiting_paused(true);
    assert(!f.step_if_ready());
    f.assert_time(0);
    f.gate->set_waiting_paused(false);
    assert(f.step_if_ready());
    f.assert_time(1);
}

void gate_does_not_wait_for_or_poll_an_async_service()
{
    Fixture f {true, false};
    f.servers[1]->queue(std::byte {9});
    assert(!f.step_if_ready());
    f.assert_time(0);
    assert(f.servers[1]->polls == 0);
    f.servers[0]->queue(std::byte {3});
    assert(f.step_if_ready());
    assert(f.servers[0]->replies.size() == 1 && f.servers[1]->replies.size() == 1);
    assert(f.servers[1]->polls == 1);

    f.servers[0]->queue(std::byte {4});
    assert(f.step_if_ready()); // There is no pending request from the async service.
    f.assert_time(2);
    assert(f.servers[0]->replies.size() == 2 && f.servers[1]->replies.size() == 1);
    assert(f.servers[1]->polls == 2);
}

void cancel_staged_request()
{
    Fixture f({true}, true);
    auto& server = *f.servers[0];
    server.queue(std::byte {7});
    assert(!f.step_if_ready() && server.reads == 1);
    f.gate->set_waiting_paused(true);
    server.cancel_pending = true;
    assert(!f.step_if_ready() && !f.gate->failed());
    assert(server.canceled_replies == 1);
    assert(f.trace == Trace({Call::Reply}));
    f.assert_time(0);
}

void accepts_next_request_after_cancel()
{
    Fixture f({true}, true);
    auto& server = *f.servers[0];
    server.queue(std::byte {7});
    assert(!f.step_if_ready() && server.reads == 1);
    server.cancel_pending = true;
    assert(!f.step_if_ready());
    server.queue(std::byte {8});
    f.pdu->arrived = true;
    assert(f.step_if_ready());
    f.assert_time(1);
    assert(server.canceled_replies == 1 && server.reads == 2);
    assert(server.replies.size() == 1 && server.replies[0][0] == std::byte {8});
    assert(f.trace == Trace({
        Call::Reply, Call::Request, Call::Step, Call::Physics, Call::Response, Call::Reply}));
}

void pdu_alone_cannot_admit_a_canceled_request()
{
    Fixture f({true}, true);
    auto& server = *f.servers[0];
    server.queue(std::byte {7});
    assert(!f.step_if_ready() && server.reads == 1);
    server.cancel_pending = true;
    f.pdu->arrived = true;
    assert(!f.step_if_ready() && !f.gate->failed());
    assert(!f.step_if_ready() && !f.gate->failed());
    assert(f.pdu->arrived && server.canceled_replies == 1);
    assert(f.trace == Trace({Call::Reply}));
    f.assert_time(0);
}

void non_lockstep_cancel_runs_hooks_in_step()
{
    Fixture f({false}, false, false);
    auto& server = *f.servers[0];
    server.queue(std::byte {7});
    assert(f.step_if_ready());
    assert(f.trace == Trace({Call::Request, Call::Step, Call::Physics}));
    f.trace.clear();

    server.cancel_pending = true;
    assert(f.step_if_ready());
    assert(server.canceled_replies == 1);
    assert(f.trace == Trace({Call::Cancel, Call::Step, Call::Physics, Call::Response, Call::Reply}));
    f.assert_time(2);
}

void old_core_busy_state_mismatch_stops_the_step()
{
    Fixture f;
    auto& server = *f.servers[0];
    server.queue(std::byte {1});
    assert(f.gate->ready_to_step());
    assert(server.reads == 1 && server.replies.empty());

    // Emulate Core having observed the overlap while the first input was staged.
    // The source must report the error to the gate before admitting a step.
    server.queue(std::byte {2});
    assert(server.poll(1) == AsyncServiceEvent::None);
    assert(server.busy_replies == 1);
    assert(!f.gate->ready_to_step());
    assert(f.gate->failed());
    assert(f.gate->failure_reason().find("service-1") != std::string::npos);
    assert(f.gate->failure_reason().find("service state mismatch") != std::string::npos);
    assert(f.trace.empty());
    assert(server.polls == 3 && server.reads == 1);
    assert(server.replies.empty());
    f.assert_time(0);
}

#else

void rejects_lockstep_construction_when_disabled()
{
    Trace trace;
    auto server = std::make_shared<FakeServer>(1, trace);
    auto service = std::make_shared<FakeService>("joint", trace);
    auto robot = std::make_shared<test_support::SingleServiceRobot>(service);
    AsyncServiceCommandSource ordinary("async-service", 1, server, robot, 0);
    assert(ordinary.id() == "async-service");
    bool rejected = false;
    try {
        AsyncServiceCommandSource lockstep("lockstep-service", 1, server, robot, 0, true);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);
}

#endif

} // namespace

int main(int argc, [[maybe_unused]] char** argv)
{
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP) && HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP
    const auto cases = std::array {
        cancel_staged_request, accepts_next_request_after_cancel,
        pdu_alone_cannot_admit_a_canceled_request, non_lockstep_cancel_runs_hooks_in_step,
    };
    if (argc == 2) {
        const std::string_view name = argv[1];
        if (name.size() != 2 || name[0] != 'C' || name[1] < '1' || name[1] > '4') {
            return 2;
        }
        cases[name[1] - '1']();
        std::cout << name << " passed\n";
        return 0;
    }
    for (const auto test : cases) {
        test();
    }
    waits_without_a_request();
    consumes_the_staged_request_in_the_same_step();
    alternates_requests_and_replies_for_ten_steps();
    waits_for_every_lockstep_service();
    applies_separate_initial_and_subsequent_timeouts();
    pause_blocks_progress_and_restarts_timeout();
    gate_does_not_wait_for_or_poll_an_async_service();
    old_core_busy_state_mismatch_stops_the_step();
    waits_for_pdu_and_service_in_either_order();
    mixed_timeout_names_only_the_missing_input();
    mixed_pause_holds_both_inputs_and_restarts_the_deadline();
    pdu_only_keeps_initial_sync_and_has_no_deadline();
    std::cout << "async service lockstep unit tests H1-H8 and common gate G1-G4 passed\n";
#else
    if (argc != 1) return 2;
    rejects_lockstep_construction_when_disabled();
    std::cout << "async service lockstep-disabled unit test H9 passed\n";
#endif
}
