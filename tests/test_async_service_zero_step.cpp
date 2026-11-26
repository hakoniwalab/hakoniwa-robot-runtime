// Stage B (first half): complete a request in its first physics substep.
#include "runtime/actuator_runtime.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runtime/controller/hold_controller.hpp"
#include "runtime/service/async_service_pipeline.hpp"
#include "support/single_service_robot.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iostream>
#include <optional>
#include <utility>

using namespace hakoniwa::robot_runtime::runtime;
namespace test_support = hakoniwa::robot_runtime::test_support;

namespace {

constexpr int kServiceId = 17;
constexpr auto kSource = "service-source";
constexpr auto kController = "service-controller";
constexpr auto kHold = "hold";
constexpr auto kJoint = "joint";
constexpr std::uint64_t kDelta = 1'000;
constexpr double kServiceVelocity = -2.5;
constexpr double kHoldPosition = 0.75;
const std::vector<std::byte> kRequest {std::byte {0}, std::byte {0xFF}};

enum class Call { Poll, Request, Cancel, NextSource, Step, Physics, Response, Reply };
using Trace = std::vector<Call>;

std::vector<std::byte> packet_for(AsyncServiceResultCode code)
{
    return {static_cast<std::byte>(code), std::byte {0xC1},
        std::byte {0x53}, std::byte {0x54}, std::byte {0x55}};
}

class FakePlant final : public IActuatorPlant {
public:
    explicit FakePlant(Trace& trace) : trace_(trace) {}

    std::vector<std::vector<ActuatorCommand>> applied;
    bool advance_position {false};

    PlantCapabilities capabilities() const noexcept override { return {}; }
    void apply_operations(const PlantOperations&) override {}

    std::uint64_t delta_time_usec() const noexcept override { return kDelta; }

    RobotState read_state() const override
    {
        RobotState state;
        state.sample_time_usec = applied.size() * kDelta;
        const auto position = kHoldPosition + (advance_position ? applied.size() : 0);
        state.actuators.push_back({kJoint, position, 0, 0, state.sample_time_usec});
        return state;
    }

    RobotState step(const std::vector<ActuatorCommand>& commands) override
    {
        trace_.push_back(Call::Physics);
        applied.push_back(commands);
        return read_state();
    }

    RobotState reset(std::uint64_t time_usec) override
    {
        assert(time_usec == 0);
        applied.clear();
        return read_state();
    }

private:
    Trace& trace_;
};

class FakeServer final : public IAsyncServiceServer {
public:
    FakeServer(Trace& trace, const FakePlant& plant) : trace_(trace), plant_(plant) {}

    struct Reply {
        AsyncServiceResultCode code;
        std::vector<std::byte> bytes;
        std::size_t physics_steps;
    };

    int polls {0};
    int reads {0};
    int reply_attempts {0};
    int writes {0};
    bool cancel {false};
    AsyncServiceState server_state {AsyncServiceState::Idle};
    std::optional<std::vector<std::byte>> request;
    std::vector<Reply> replies;

    AsyncServiceEvent poll(int service_id) noexcept override
    {
        assert(service_id == kServiceId);
        ++polls;
        trace_.push_back(Call::Poll);
        if (std::exchange(cancel, false)) {
            if (server_state == AsyncServiceState::Doing) {
                server_state = AsyncServiceState::Canceling;
            }
            return AsyncServiceEvent::Cancel;
        }
        if (request) {
            server_state = AsyncServiceState::Doing;
            return AsyncServiceEvent::Request;
        }
        return AsyncServiceEvent::None;
    }

    std::optional<AsyncServiceState> status(int service_id) noexcept override
    {
        assert(service_id == kServiceId);
        return server_state;
    }

    bool get_request(int service_id, std::vector<std::byte>& bytes) override
    {
        assert(service_id == kServiceId && request);
        ++reads;
        bytes = std::move(*request);
        request.reset();
        return true;
    }

    bool reply(
        int service_id, AsyncServiceResultCode code,
        const AsyncServiceResponseWriter& write) override
    {
        assert(service_id == kServiceId);
        ++reply_attempts;
        packet_[0] = static_cast<std::byte>(code);
        if (write) {
            ++writes;
            if (!write(packet_)) {
                return false;
            }
        }
        trace_.push_back(Call::Reply);
        replies.push_back({code, packet_, plant_.applied.size()});
        server_state = AsyncServiceState::Idle;
        return true;
    }

private:
    Trace& trace_;
    const FakePlant& plant_;
    std::vector<std::byte> packet_ = packet_for(AsyncServiceResultCode::Ok);
};

std::vector<std::byte> response_for(
    std::span<const std::byte> request, std::byte position = std::byte {0})
{
    auto response = packet_for(AsyncServiceResultCode::Ok);
    std::copy(request.begin(), request.end(), response.begin() + 2);
    response.back() = position;
    return response;
}

class ObserveServicePoll final : public ICommandSource {
public:
    explicit ObserveServicePoll(Trace& trace) : trace_(trace) {}
    std::string_view id() const noexcept override { return "source-order-probe"; }
    CommandSourcePollResult poll(const RuntimeStepContext&) override
    {
        trace_.push_back(Call::NextSource);
        return {nullptr, {std::string(id()), ComponentState::WaitingForInput, {}}};
    }
    void reset() noexcept override {}
private:
    Trace& trace_;
};

class FakeService : public test_support::OneService {
public:
    explicit FakeService(Trace& trace) : trace_(trace) {}

    bool accept {true};
    bool write_ok {true};
    int steps_until_done {1};
    AsyncServiceStepOutcome final_outcome {AsyncServiceStepOutcome::Ok};
    int cancel_notifications {0};
    int resets {0};
    std::vector<std::vector<std::byte>> requests;
    std::vector<std::uint64_t> step_times;
    std::vector<AsyncServiceState> step_states;
    std::vector<double> response_positions;
    std::vector<std::uint64_t> response_times;

    bool on_request(std::span<const std::byte> request) override
    {
        trace_.push_back(Call::Request);
        requests.emplace_back(request.begin(), request.end());
        steps_ = 0;
        cancel_steps_ = 0;
        return accept;
    }

    test_support::OneServiceStepResult on_step(
        const RobotState& state, AsyncServiceState server_state,
        const RuntimeStepContext& context) override
    {
        trace_.push_back(Call::Step);
        assert(state.sample_time_usec == context.simulation_time_usec);
        assert(state.actuators.size() == 1);
        step_times.push_back(context.simulation_time_usec);
        step_states.push_back(server_state);
        auto outcome = AsyncServiceStepOutcome::Pending;
        if (server_state == AsyncServiceState::Canceling) {
            if (++cancel_steps_ == 2) {
                outcome = AsyncServiceStepOutcome::Canceled;
            }
        } else if (++steps_ == steps_until_done) {
            outcome = final_outcome;
        }
        return {
            outcome,
            {{kJoint, ActuatorCommandType::Velocity, kServiceVelocity, {},
                context.simulation_time_usec, context.simulation_time_usec + kDelta}},
        };
    }

    bool on_response(
        const RobotState& state, const RuntimeStepContext& context,
        std::span<std::byte> packet) override
    {
        trace_.push_back(Call::Response);
        assert(state.sample_time_usec == context.simulation_time_usec);
        assert(packet.size() == 5 && packet[1] == std::byte {0xC1});
        response_positions.push_back(state.actuators.at(0).position_rad);
        response_times.push_back(context.simulation_time_usec);
        const auto& request = requests.back();
        packet[2] = request[0];
        packet[3] = request.size() == 2 ? request[1] : std::byte {0x54};
        packet.back() = static_cast<std::byte>(static_cast<int>(response_positions.back()));
        return write_ok;
    }

    void reset() override
    {
        ++resets;
        steps_ = 0;
        cancel_steps_ = 0;
    }

protected:
    Trace& trace_;
private:
    int steps_ {0};
    int cancel_steps_ {0};
};

class NotifiedService final : public FakeService {
public:
    using FakeService::FakeService;
    void on_cancel() override
    {
        trace_.push_back(Call::Cancel);
        ++cancel_notifications;
    }
};

struct Fixture {
    explicit Fixture(bool notify_cancel = true)
        : service(notify_cancel ? std::make_shared<NotifiedService>(trace)
                                : std::make_shared<FakeService>(trace))
    {
    }

    Trace trace;
    std::shared_ptr<FakePlant> plant = std::make_shared<FakePlant>(trace);
    std::shared_ptr<FakeServer> server = std::make_shared<FakeServer>(trace, *plant);
    std::shared_ptr<FakeService> service;
    std::shared_ptr<test_support::SingleServiceRobot> robot =
        std::make_shared<test_support::SingleServiceRobot>(service);
    std::shared_ptr<AsyncServiceCommandSource> service_source =
        std::make_shared<AsyncServiceCommandSource>(std::string(kSource) + "/0", kServiceId, server, robot, 0);
    std::shared_ptr<RobotServiceSource> source =
        std::make_shared<RobotServiceSource>(kSource, std::vector {service_source});
    std::shared_ptr<RobotServiceController> controller =
        std::make_shared<RobotServiceController>(kController, source, robot);
    ActuatorRuntime runtime {
        {source, std::make_shared<ObserveServicePoll>(trace)},
        {controller, std::make_shared<HoldController>(
            kHold, std::vector<HoldActuatorBinding> {{kJoint}})},
        std::make_shared<PriorityCommandArbiter>(std::vector<ControllerPriority> {
            {kController, 10, {}}, {kHold, 0, {}},
        }),
        plant,
        {std::make_shared<AsyncServiceResponsePublisher>(
            "service-response", service_source)},
    };
};

void assert_service_command(const std::vector<ActuatorCommand>& commands)
{
    assert(commands.size() == 1);
    assert(commands.front().actuator_id == kJoint);
    assert(commands.front().type == ActuatorCommandType::Velocity);
    assert(commands.front().value == kServiceVelocity);
    assert(commands.front().source_id == kSource);
}

void assert_hold_command(const std::vector<ActuatorCommand>& commands)
{
    assert(commands.size() == 1);
    assert(commands.front().actuator_id == kJoint);
    assert(commands.front().type == ActuatorCommandType::Position);
    assert(commands.front().value == kHoldPosition);
    assert(commands.front().source_id.empty());
}

void no_request_allows_ten_steps()
{
    Fixture f;
    for (int step = 0; step < 10; ++step) {
        (void)f.runtime.step_hakoniwa(1);
    }
    assert(f.plant->applied.size() == 10);
    assert(f.plant->read_state().sample_time_usec == 10 * kDelta);
    assert(f.server->polls == 10 && f.server->reads == 0);
    assert(f.service->requests.empty() && f.service->step_times.empty());
    assert(f.server->replies.empty());
}

void request_completes_in_its_first_step()
{
    Fixture f;
    (void)f.runtime.step_hakoniwa(1);
    f.trace.clear();
    f.server->request = kRequest;
    (void)f.runtime.step_hakoniwa(1);

    assert(f.trace == Trace({
        Call::Poll, Call::Request, Call::NextSource, Call::Step,
        Call::Physics, Call::Response, Call::Reply}));
    assert(f.service->requests == std::vector<std::vector<std::byte>> {kRequest});
    assert(f.service->step_times == std::vector<std::uint64_t> {kDelta});
    assert(f.server->reads == 1 && f.server->replies.size() == 1);
    assert(f.server->replies[0].physics_steps == 2);
    assert(f.server->replies[0].code == AsyncServiceResultCode::Ok);
    assert(f.server->replies[0].bytes == response_for(kRequest));
    (void)f.runtime.step_hakoniwa(1);
    assert(f.server->replies.size() == 1);
}

void completion_step_applies_service_command()
{
    Fixture f;
    f.server->request = kRequest;
    const auto report = f.runtime.step_hakoniwa(1);

    assert(report.arbitration.selected_control_id == kController);
    assert(f.plant->applied.size() == 1);
    assert_service_command(f.plant->applied[0]);
}

void five_consecutive_requests_each_complete_on_time()
{
    Fixture f;
    for (std::size_t step = 0; step < 5; ++step) {
        const std::vector<std::byte> request {static_cast<std::byte>(step)};
        f.server->request = request;
        (void)f.runtime.step_hakoniwa(1);
        assert(f.server->replies.size() == step + 1);
        assert(f.server->replies.back().physics_steps == step + 1);
        assert(f.server->replies.back().code == AsyncServiceResultCode::Ok);
        assert(f.server->replies.back().bytes == response_for(request));
        assert(f.service->step_times.back() == step * kDelta);
    }
    assert(f.server->polls == 5 && f.server->reads == 5);
    assert(f.service->requests.size() == 5 && f.service->step_times.size() == 5);
}

void four_substeps_use_service_only_in_first_substep()
{
    Fixture f;
    f.server->request = kRequest;
    const auto report = f.runtime.step_hakoniwa(4);

    assert(f.server->polls == 1 && f.server->reads == 1);
    assert(f.service->requests.size() == 1 && f.service->step_times.size() == 1);
    assert(f.plant->applied.size() == 4);
    assert_service_command(f.plant->applied[0]);
    for (std::size_t step = 1; step < 4; ++step) {
        assert_hold_command(f.plant->applied[step]);
    }
    assert(report.arbitration.selected_control_id == kHold);
    assert(f.server->replies.size() == 1);
    assert(f.server->replies[0].physics_steps == 4);
    assert(f.server->replies[0].code == AsyncServiceResultCode::Ok);
    assert(f.server->replies[0].bytes == response_for(kRequest));
}

void rejected_request_replies_invalid_after_physics()
{
    Fixture f;
    f.service->accept = false;
    f.server->request = kRequest;
    (void)f.runtime.step_hakoniwa(1);

    assert(f.trace == Trace({
        Call::Poll, Call::Request, Call::NextSource, Call::Physics, Call::Reply}));
    assert(f.service->step_times.empty());
    assert(f.server->replies.size() == 1);
    assert(f.server->replies[0].physics_steps == 1);
    assert(f.server->replies[0].code == AsyncServiceResultCode::Invalid);
    assert(f.server->replies[0].bytes == packet_for(AsyncServiceResultCode::Invalid));
    assert_hold_command(f.plant->applied[0]);
    (void)f.runtime.step_hakoniwa(1);
    assert(f.server->replies.size() == 1);
}

void idle_service_leaves_hold_selected()
{
    Fixture f;
    const auto report = f.runtime.step_hakoniwa(1);

    assert(report.controller_statuses[0].state == ComponentState::WaitingForInput);
    assert(report.arbitration.selected_control_id == kHold);
    assert_hold_command(f.plant->applied[0]);
}

void response_writes_post_physics_position()
{
    Fixture f;
    f.plant->advance_position = true;
    f.server->request = kRequest;
    (void)f.runtime.step_hakoniwa(1);

    assert(f.service->step_times == std::vector<std::uint64_t> {0});
    assert(f.service->response_times == std::vector<std::uint64_t> {kDelta});
    assert(f.service->response_positions == std::vector<double> {kHoldPosition + 1});
    assert(f.server->replies.size() == 1);
    assert(f.server->replies[0].bytes == response_for(kRequest, std::byte {1}));
}

void response_runs_once_after_all_four_substeps()
{
    Fixture f;
    f.plant->advance_position = true;
    f.server->request = kRequest;
    (void)f.runtime.step_hakoniwa(4);

    assert(f.service->response_times == std::vector<std::uint64_t> {4 * kDelta});
    assert(f.service->response_positions == std::vector<double> {kHoldPosition + 4});
    assert(f.server->replies.size() == 1);
    assert(f.server->replies[0].bytes == response_for(kRequest, std::byte {4}));
}

void rejection_never_invokes_response_hook()
{
    Fixture f;
    f.server->request = kRequest;
    (void)f.runtime.step_hakoniwa(1);
    assert(f.server->replies.size() == 1);
    f.service->accept = false;
    f.server->request = kRequest;
    (void)f.runtime.step_hakoniwa(1);

    assert(f.service->response_times == std::vector<std::uint64_t> {kDelta});
    assert(f.server->writes == 1);
    assert(f.server->replies.size() == 2);
    assert(f.server->replies[1].code == AsyncServiceResultCode::Invalid);
    auto previous_packet = f.server->replies[0].bytes;
    previous_packet[0] = static_cast<std::byte>(AsyncServiceResultCode::Invalid);
    assert(f.server->replies[1].bytes == previous_packet);
}

void failed_response_writer_reports_error_without_sending()
{
    Fixture f;
    f.service->write_ok = false;
    f.server->request = kRequest;
    const auto report = f.runtime.step_hakoniwa(1);

    assert(report.publisher_statuses.size() == 1);
    assert(report.publisher_statuses[0].state == ComponentState::Error);
    assert(f.server->reply_attempts == 1 && f.server->writes == 1);
    assert(f.server->replies.empty());
    (void)f.runtime.step_hakoniwa(1);
    assert(f.server->reply_attempts == 1);
}

void reused_packet_gets_a_fresh_body_on_each_response()
{
    Fixture f;
    f.plant->advance_position = true;
    f.server->request = kRequest;
    (void)f.runtime.step_hakoniwa(1);
    const std::vector second_request {std::byte {0x32}};
    f.server->request = second_request;
    (void)f.runtime.step_hakoniwa(1);

    assert(f.server->writes == 2 && f.server->replies.size() == 2);
    assert(f.server->replies[0].bytes == response_for(kRequest, std::byte {1}));
    assert(f.server->replies[1].bytes == response_for(second_request, std::byte {2}));
}

void long_request_keeps_running_until_the_third_step()
{
    Fixture f;
    f.service->steps_until_done = 3;
    f.server->request = kRequest;
    for (std::size_t step = 1; step <= 3; ++step) {
        const auto report = f.runtime.step_hakoniwa(1);
        assert(report.arbitration.selected_control_id == kController);
        assert_service_command(f.plant->applied.back());
        assert(f.service->step_times.size() == step);
        assert(f.server->replies.size() == (step == 3 ? 1 : 0));
    }
    assert(f.service->step_times == std::vector<std::uint64_t>({0, kDelta, 2 * kDelta}));
    assert(f.service->step_states == std::vector<AsyncServiceState>(3, AsyncServiceState::Doing));
    assert(f.service->response_times == std::vector<std::uint64_t> {3 * kDelta});
    assert(f.server->replies[0].code == AsyncServiceResultCode::Ok);
}

void hold_takes_over_after_long_completion()
{
    Fixture f;
    f.service->steps_until_done = 3;
    f.server->request = kRequest;
    for (int step = 0; step < 3; ++step) {
        (void)f.runtime.step_hakoniwa(1);
    }
    const auto report = f.runtime.step_hakoniwa(1);
    assert(f.service->step_times.size() == 3);
    assert(f.server->replies.size() == 1);
    assert(report.arbitration.selected_control_id == kHold);
    assert_hold_command(f.plant->applied.back());
}

void completion_in_second_group_of_four_waits_for_publish()
{
    Fixture f;
    f.service->steps_until_done = 6;
    f.server->request = kRequest;
    (void)f.runtime.step_hakoniwa(4);
    assert(f.server->replies.empty() && f.service->step_times.size() == 4);
    assert(f.server->polls == 1);

    (void)f.runtime.step_hakoniwa(4);
    assert(f.server->polls == 2 && f.service->step_times.size() == 6);
    assert(f.service->response_times == std::vector<std::uint64_t> {8 * kDelta});
    assert(f.server->replies.size() == 1);
    assert(f.server->replies[0].physics_steps == 8);
    assert(f.server->replies[0].code == AsyncServiceResultCode::Ok);
    assert(f.plant->applied.size() == 8);
    for (std::size_t step = 0; step < 6; ++step) {
        assert_service_command(f.plant->applied[step]);
    }
    assert_hold_command(f.plant->applied[6]);
    assert_hold_command(f.plant->applied[7]);
}

void cancellation_takes_two_steps(bool notify_service)
{
    Fixture f(notify_service);
    f.service->steps_until_done = 10;
    f.server->request = kRequest;
    (void)f.runtime.step_hakoniwa(1);
    f.server->cancel = true;
    f.trace.clear();
    (void)f.runtime.step_hakoniwa(1);
    const Trace expected = notify_service
        ? Trace {Call::Poll, Call::Cancel, Call::NextSource, Call::Step, Call::Physics}
        : Trace {Call::Poll, Call::NextSource, Call::Step, Call::Physics};
    assert(f.trace == expected);
    assert(f.service->cancel_notifications == (notify_service ? 1 : 0));
    assert(f.server->replies.empty());
    assert_service_command(f.plant->applied.back());

    (void)f.runtime.step_hakoniwa(1);
    assert(f.service->step_states == std::vector<AsyncServiceState>({
        AsyncServiceState::Doing, AsyncServiceState::Canceling, AsyncServiceState::Canceling}));
    assert(f.service->cancel_notifications == (notify_service ? 1 : 0));
    assert(f.service->response_times == std::vector<std::uint64_t> {3 * kDelta});
    assert(f.server->replies.size() == 1);
    assert(f.server->replies[0].code == AsyncServiceResultCode::Canceled);
    assert_service_command(f.plant->applied.back());
}

void service_error_writes_reply_then_leaves_hold_in_control()
{
    Fixture f;
    f.service->steps_until_done = 2;
    f.service->final_outcome = AsyncServiceStepOutcome::Error;
    f.server->request = kRequest;
    (void)f.runtime.step_hakoniwa(1);
    assert(f.server->replies.empty());
    (void)f.runtime.step_hakoniwa(1);
    assert_service_command(f.plant->applied.back());
    assert(f.service->response_times == std::vector<std::uint64_t> {2 * kDelta});
    assert(f.server->replies.size() == 1);
    assert(f.server->replies[0].code == AsyncServiceResultCode::Error);
    (void)f.runtime.step_hakoniwa(1);
    assert_hold_command(f.plant->applied.back());
    assert(f.service->step_times.size() == 2 && f.server->replies.size() == 1);
}

void cancel_without_active_request_is_ignored()
{
    Fixture f;
    f.server->cancel = true;
    const auto report = f.runtime.step_hakoniwa(1);
    assert(report.source_statuses[0].can_produce_commands()
        || report.source_statuses[0].state == ComponentState::WaitingForInput);
    assert(report.controller_statuses[0].state == ComponentState::WaitingForInput);
    assert(f.service->cancel_notifications == 0 && f.service->step_times.empty());
    assert(f.server->reply_attempts == 0);
    assert_hold_command(f.plant->applied.back());
}

void reset_drops_active_or_staged_reply()
{
    for (const int completion_step : {1, 3}) {
        Fixture f;
        f.service->steps_until_done = completion_step;
        f.server->request = kRequest;
        (void)f.runtime.step(false);
        assert(f.server->reply_attempts == 0);
        assert(f.service->step_times.size() == 1);
        // Core resets its own server on the next run; Runtime resets the service.
        f.server->server_state = AsyncServiceState::Idle;
        (void)f.runtime.reset();
        (void)f.runtime.step_hakoniwa(1);
        assert(f.service->resets == 1);
        assert(f.service->step_times.size() == 1);
        assert(f.server->reply_attempts == 0);
        assert_hold_command(f.plant->applied.back());
        f.server->request = kRequest;
        f.service->steps_until_done = 1;
        (void)f.runtime.step_hakoniwa(1);
        assert(f.service->requests.size() == 2 && f.server->replies.size() == 1);
        assert(f.server->replies[0].code == AsyncServiceResultCode::Ok);
    }
}

void server_state_mismatch_is_a_controller_error()
{
    Fixture f;
    f.service->steps_until_done = 3;
    f.server->request = kRequest;
    (void)f.runtime.step_hakoniwa(1);
    f.server->server_state = AsyncServiceState::Idle;
    const auto report = f.runtime.step_hakoniwa(1);
    assert(report.controller_statuses[0].state == ComponentState::Error);
    assert(f.service->step_times.size() == 1);
    assert(f.server->reply_attempts == 0);
    assert_hold_command(f.plant->applied.back());
}

} // namespace

int main()
{
    no_request_allows_ten_steps();
    request_completes_in_its_first_step();
    completion_step_applies_service_command();
    five_consecutive_requests_each_complete_on_time();
    four_substeps_use_service_only_in_first_substep();
    rejected_request_replies_invalid_after_physics();
    idle_service_leaves_hold_selected();
    response_writes_post_physics_position();
    response_runs_once_after_all_four_substeps();
    rejection_never_invokes_response_hook();
    failed_response_writer_reports_error_without_sending();
    reused_packet_gets_a_fresh_body_on_each_response();
    long_request_keeps_running_until_the_third_step();
    hold_takes_over_after_long_completion();
    completion_in_second_group_of_four_waits_for_publish();
    cancellation_takes_two_steps(true);
    cancellation_takes_two_steps(false);
    service_error_writes_reply_then_leaves_hold_in_control();
    cancel_without_active_request_is_ignored();
    reset_drops_active_or_staged_reply();
    server_state_mismatch_is_a_controller_error();
    std::cout << "async service unit tests passed (21 cases)\n";
}
