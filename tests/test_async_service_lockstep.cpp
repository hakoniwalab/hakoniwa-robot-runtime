#undef NDEBUG  // Keep the contract checks in Release builds.

#include "runtime/service/async_service_pipeline.hpp"

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
#include "runtime/service/async_service_lockstep_gate.hpp"

#include <chrono>
#include <initializer_list>
#include <utility>
#endif

using namespace hakoniwa::robot_runtime::runtime;

namespace {

enum class Call { Request, Step, Physics, Response, Reply };
using Trace = std::vector<Call>;

class FakeServer final : public IAsyncServiceServer {
public:
    FakeServer(int id, Trace& trace) : id_(id), trace_(trace) {}

    int polls {0};
    int reads {0};
    int busy_replies {0};
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
        assert(service_id == id_ && code == AsyncServiceResultCode::Ok && write);
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

class FakeService final : public IAsyncService {
public:
    FakeService(std::string joint, Trace& trace) : joint_(std::move(joint)), trace_(trace) {}

    bool on_request(std::span<const std::byte> request) override
    {
        trace_.push_back(Call::Request);
        assert(request.size() == 1);
        value_ = request.front();
        return true;
    }

    AsyncServiceStepResult on_step(
        const RobotState& state, AsyncServiceState server_state,
        const RuntimeStepContext& context) override
    {
        trace_.push_back(Call::Step);
        assert(state.sample_time_usec == context.simulation_time_usec);
        assert(server_state == AsyncServiceState::Doing);
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

struct Fixture {
    Trace trace;
    AsyncServiceLockstepGate::Clock::time_point now {};
    std::shared_ptr<FakePlant> plant = std::make_shared<FakePlant>(trace);
    std::vector<std::shared_ptr<FakeServer>> servers;
    std::unique_ptr<AsyncServiceLockstepGate> gate;
    std::unique_ptr<ActuatorRuntime> runtime;

    explicit Fixture(std::initializer_list<bool> lockstep = {true})
    {
        std::vector<std::shared_ptr<AsyncServiceCommandSource>> gate_sources;
        std::vector<std::shared_ptr<ICommandSource>> sources;
        std::vector<std::shared_ptr<IController>> controllers;
        std::vector<std::shared_ptr<IStatePublisher>> publishers;
        std::vector<ControllerPriority> priorities;

        int id = 0;
        for (const bool is_lockstep : lockstep) {
            const auto suffix = std::to_string(++id);
            auto server = std::make_shared<FakeServer>(id, trace);
            auto source = std::make_shared<AsyncServiceCommandSource>(
                "service-" + suffix, id, server,
                std::make_shared<FakeService>("joint-" + suffix, trace), is_lockstep);
            auto controller = std::make_shared<AsyncServiceController>(
                "controller-" + suffix, source);
            servers.push_back(server);
            gate_sources.push_back(source);
            sources.push_back(source);
            controllers.push_back(controller);
            publishers.push_back(std::make_shared<AsyncServiceResponsePublisher>(
                "publisher-" + suffix, source));
            priorities.push_back({std::string(controller->id()), 1, "services"});
        }

        gate = std::make_unique<AsyncServiceLockstepGate>(
            std::move(gate_sources), AsyncServiceLockstepConfig {}, [this] { return now; });
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
        return true;
    }

    void assert_time(std::size_t steps) const
    {
        assert(plant->applied.size() == steps);
        assert(plant->read_state().sample_time_usec == steps * kDelta);
    }
};

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
    assert(f.servers[0]->polls == 1 && f.servers[0]->reads == 1);
    assert(f.servers[1]->polls == 2 && f.servers[1]->reads == 0);
    assert(f.trace.empty());

    f.servers[1]->queue(std::byte {2});
    assert(f.step_if_ready());
    f.assert_time(1);
    assert(f.servers[0]->polls == 1 && f.servers[1]->polls == 3);
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

void old_core_busy_state_mismatch_stops_the_step()
{
    Fixture f;
    auto& server = *f.servers[0];
    server.queue(std::byte {1});
    assert(f.gate->ready_to_step());
    assert(server.reads == 1 && server.replies.empty());

    // Emulate Core having observed the overlap while the first input was staged.
    // The gate must still avoid a second poll when the staged input is consumed.
    server.queue(std::byte {2});
    assert(server.poll(1) == AsyncServiceEvent::None);
    assert(server.busy_replies == 1);
    bool stopped = false;
    try {
        (void)f.step_if_ready();
    } catch (const std::runtime_error& error) {
        stopped = std::string(error.what()).find("service state mismatch") != std::string::npos;
    }
    assert(stopped);
    assert(f.trace.empty());
    assert(server.polls == 2 && server.reads == 1);
    assert(server.replies.empty());
    f.assert_time(0);
}

#else

void rejects_lockstep_construction_when_disabled()
{
    Trace trace;
    auto server = std::make_shared<FakeServer>(1, trace);
    auto service = std::make_shared<FakeService>("joint", trace);
    AsyncServiceCommandSource ordinary("async-service", 1, server, service);
    assert(ordinary.id() == "async-service");
    bool rejected = false;
    try {
        AsyncServiceCommandSource lockstep("lockstep-service", 1, server, service, true);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);
}

#endif

} // namespace

int main()
{
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP) && HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP
    waits_without_a_request();
    consumes_the_staged_request_in_the_same_step();
    alternates_requests_and_replies_for_ten_steps();
    waits_for_every_lockstep_service();
    applies_separate_initial_and_subsequent_timeouts();
    pause_blocks_progress_and_restarts_timeout();
    gate_does_not_wait_for_or_poll_an_async_service();
    old_core_busy_state_mismatch_stops_the_step();
    std::cout << "async service lockstep unit tests H1-H8 passed\n";
#else
    rejects_lockstep_construction_when_disabled();
    std::cout << "async service lockstep-disabled unit test H9 passed\n";
#endif
}
