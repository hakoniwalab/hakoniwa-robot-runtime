#undef NDEBUG  // Keep the contract checks in Release builds.

#include "runner/hakoniwa/hakoniwa_asset_driver.hpp"
#include "runner/hakoniwa/hakoniwa_runner.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runtime/service/async_service_pipeline.hpp"

#include <array>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace {

using namespace ::hakoniwa::robot_runtime;
using namespace ::hakoniwa::robot_runtime::runner;
using namespace ::hakoniwa::robot_runtime::runtime;
using namespace std::chrono_literals;

constexpr std::uint64_t kStepUsec = 1'000;
constexpr int kServiceId = 17;
constexpr char kServiceName[] = "runner-lockstep-service";
constexpr std::byte kRequest {0xA7};

struct Times {
    std::uint64_t hakoniwa_usec;
    std::uint64_t physics_usec;

    bool operator==(const Times&) const = default;
};

class FakeAssetApi final : public runner::detail::IHakoniwaAssetApi {
public:
    explicit FakeAssetApi(const adapters::mujoco::MujocoActuatorPlant& plant)
        : plant_(plant)
    {
    }

    int register_asset(
        const runner::detail::HakoniwaAssetDriverConfig& config,
        hako_asset_callbacks_t* callbacks) noexcept override
    {
        assert(config.delta_time_usec == kStepUsec);
        callbacks_ = callbacks;
        return 0;
    }

    int start_no_wait(int (*is_force_stop)()) noexcept override
    {
        const int init_result = callbacks_->on_initialize(nullptr);
        if (init_result != 0) {
            return init_result;
        }
        const int result = callbacks_->on_manual_timing_control(nullptr);
        force_stopped = is_force_stop() != 0;
        return result;
    }

    int sleep_no_wait(hako_time_t usec, int (*)()) noexcept override
    {
        assert(usec == kStepUsec);
        sleeps.push_back(times());
        hakoniwa_usec_ += static_cast<std::uint64_t>(usec);
        // Simulate a normal external stop after the test's last sleep.
        return sleeps.size() == stop_after_sleeps ? EINTR : 0;
    }

    CoreActivity core_activity() noexcept override { return CoreActivity::running; }

    Times times() const
    {
        return {hakoniwa_usec_, plant_.read_state().sample_time_usec};
    }

    std::vector<Times> sleeps;
    std::size_t stop_after_sleeps {1};
    bool force_stopped {false};

private:
    const adapters::mujoco::MujocoActuatorPlant& plant_;
    hako_asset_callbacks_t* callbacks_ {nullptr};
    std::uint64_t hakoniwa_usec_ {0};
};

class FakeServer final : public IAsyncServiceServer {
public:
    FakeServer(const FakeAssetApi& asset, std::size_t request_on_poll,
        std::chrono::milliseconds clock_tick)
        : asset_(asset)
        , request_on_poll_(request_on_poll)
        , clock_tick_(clock_tick)
    {
    }

    AsyncServiceEvent poll(int service_id) noexcept override
    {
        assert(service_id == kServiceId);
        polls.push_back(asset_.times());
        now += clock_tick_;
        if (polls.size() == request_on_poll_) {
            state_ = AsyncServiceState::Doing;
            return AsyncServiceEvent::Request;
        }
        return AsyncServiceEvent::None;
    }

    std::optional<AsyncServiceState> status(int service_id) noexcept override
    {
        assert(service_id == kServiceId);
        return state_;
    }

    bool get_request(int service_id, std::vector<std::byte>& bytes) override
    {
        assert(service_id == kServiceId && state_ == AsyncServiceState::Doing);
        ++reads;
        bytes = {kRequest};
        return true;
    }

    bool reply(
        int service_id, AsyncServiceResultCode code,
        const AsyncServiceResponseWriter& write) override
    {
        assert(service_id == kServiceId && code == AsyncServiceResultCode::Ok);
        assert(state_ == AsyncServiceState::Doing && write);
        std::array<std::byte, 1> packet {};
        if (!write(packet)) {
            return false;
        }
        assert(packet.front() == kRequest);
        replies.push_back(asset_.times());
        state_ = AsyncServiceState::Idle;
        return true;
    }

    std::chrono::steady_clock::time_point now {};
    std::vector<Times> polls;
    std::vector<Times> replies;
    int reads {0};

private:
    const FakeAssetApi& asset_;
    std::size_t request_on_poll_;
    std::chrono::milliseconds clock_tick_;
    AsyncServiceState state_ {AsyncServiceState::Idle};
};

class Service final : public IAsyncService {
public:
    bool on_request(std::span<const std::byte> request) override
    {
        return request.size() == 1 && request.front() == kRequest;
    }

    AsyncServiceStepResult on_step(
        const RobotState&, AsyncServiceState, const RuntimeStepContext&) override
    {
        return {AsyncServiceStepOutcome::Ok, {}};
    }

    bool on_response(
        const RobotState&, const RuntimeStepContext&, std::span<std::byte> packet) override
    {
        packet.front() = kRequest;
        return true;
    }

    void reset() override {}
};

std::shared_ptr<adapters::mujoco::MujocoActuatorPlant> make_plant()
{
    namespace fs = std::filesystem;
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto path = fs::temp_directory_path() /
        ("hakoniwa-runner-service-lockstep-" + std::to_string(stamp) + ".xml");
    {
        std::ofstream model(path);
        model << R"(<mujoco model="runner-lockstep">
  <option timestep="0.001"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>)";
        assert(model);
    }
    RuntimeDefinition definition;
    definition.model_path = path.string();
    auto plant = std::make_shared<adapters::mujoco::MujocoActuatorPlant>(definition);
    fs::remove(path);
    return plant;
}

class FakePduInput final : public ILockstepInput {
public:
    bool lockstep_ready() noexcept override { return ++polls >= 3; }
    int polls {0};
};

struct Fixture {
    Fixture(std::size_t request_on_poll, std::chrono::milliseconds clock_tick, bool with_pdu = false)
        : server(std::make_shared<FakeServer>(*api, request_on_poll, clock_tick))
    {
        if (with_pdu) {
            pdu = std::make_shared<FakePduInput>();
            api->stop_after_sleeps = 2;
        }
    }

    HakoniwaRunnerResources resources()
    {
        HakoniwaRunnerResources resources;
        resources.plant = plant;
        resources.endpoint = std::make_unique<::hakoniwa::pdu::Endpoint>(
            "runner-lockstep", HAKO_PDU_ENDPOINT_DIRECTION_INOUT);
        const auto source = std::make_shared<AsyncServiceCommandSource>(
            kServiceName, kServiceId, server, std::make_shared<Service>(), true);
        const auto controller = std::make_shared<AsyncServiceController>("controller", source);
        const auto publisher = std::make_shared<AsyncServiceResponsePublisher>("publisher", source);
        resources.runtime = std::make_unique<ActuatorRuntime>(
            std::vector<std::shared_ptr<ICommandSource>> {source},
            std::vector<std::shared_ptr<IController>> {controller},
            std::make_shared<PriorityCommandArbiter>(std::vector<ControllerPriority> {}),
            plant, std::vector<std::shared_ptr<IStatePublisher>> {publisher});
        resources.lockstep_inputs.push_back({kServiceName, source});
        if (pdu) {
            resources.lockstep_inputs.push_back({"Car-1/tire_friction", pdu});
        }
        return resources;
    }

    std::shared_ptr<adapters::mujoco::MujocoActuatorPlant> plant = make_plant();
    std::shared_ptr<FakeAssetApi> api = std::make_shared<FakeAssetApi>(*plant);
    std::shared_ptr<FakeServer> server;
    std::shared_ptr<FakePduInput> pdu;
    HakoniwaRunner runner {
        [this] { return resources(); },
        {.asset_name = "runner-lockstep", .pdu_definition_path = "unused.json",
            .owns_conductor = false, .lockstep_now = [this] { return server->now; }},
        api,
    };
};

void test_runner_waits_without_advancing()
{
    Fixture fixture(4, 1ms);
    assert(fixture.runner.start() == 0);
    assert(fixture.runner.wait() == 0);

    // With service-only lockstep the first sleep follows the first physics
    // step. While the first batch is missing, both times stay at zero.
    assert(fixture.api->sleeps == (std::vector<Times> {{0, kStepUsec}}));
    assert(fixture.server->polls == std::vector<Times>(4, {0, 0}));
    assert(fixture.server->reads == 1);
    assert(fixture.server->replies == (std::vector<Times> {{0, kStepUsec}}));
    assert(fixture.api->times() == (Times {kStepUsec, kStepUsec}));
    assert(!fixture.api->force_stopped);
}

void test_runner_stops_on_gate_timeout()
{
    // A request on poll 10 gives the fake an exit even if the gate forgets
    // to time out. The injected clock reaches the 60-second timeout first.
    Fixture fixture(10, 20s);
    std::ostringstream errors;
    auto* previous = std::cerr.rdbuf(errors.rdbuf());
    const int start_result = fixture.runner.start();
    const int wait_result = start_result == 0 ? fixture.runner.wait() : start_result;
    std::cerr.rdbuf(previous);

    assert(start_result == 0);
    assert(wait_result != 0);
    assert(fixture.api->force_stopped);
    assert(errors.str().find(kServiceName) != std::string::npos);
    assert(fixture.api->sleeps.empty());
    assert(!fixture.server->polls.empty());
    for (const auto times : fixture.server->polls) {
        assert(times == (Times {0, 0}));
    }
    assert(fixture.server->reads == 0);
    assert(fixture.server->replies.empty());
    assert(fixture.api->times() == (Times {0, 0}));
}

void test_runner_syncs_before_first_step_when_a_pdu_is_registered()
{
    Fixture fixture(1, 1ms, true);
    assert(fixture.runner.start() == 0);
    assert(fixture.runner.wait() == 0);
    assert(fixture.api->sleeps == (std::vector<Times> {{0, 0}, {kStepUsec, kStepUsec}}));
    assert(fixture.pdu->polls == 3);
    // Poll for cancellation each time the staged request waits for the PDU.
    assert(fixture.server->polls == std::vector<Times>(3, {kStepUsec, 0}));
    assert(fixture.server->reads == 1);
    assert(fixture.server->replies == (std::vector<Times> {{kStepUsec, kStepUsec}}));
}

} // namespace

int main()
{
    test_runner_waits_without_advancing();
    test_runner_stops_on_gate_timeout();
    test_runner_syncs_before_first_step_when_a_pdu_is_registered();
    return 0;
}
