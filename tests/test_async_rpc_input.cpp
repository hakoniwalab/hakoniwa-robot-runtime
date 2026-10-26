#undef NDEBUG
#include "hakoniwa/robot_runtime/runner/async_rpc_input.hpp"
#include "runner/hakoniwa/hakoniwa_asset_driver.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace hakoniwa::robot_runtime::runner;
using namespace hakoniwa::robot_runtime::runner::detail;

namespace {

std::vector<std::byte> bytes(std::initializer_list<unsigned int> values)
{
    std::vector<std::byte> result;
    for (const auto value : values) {
        result.push_back(static_cast<std::byte>(value));
    }
    return result;
}

class FakeChannel final : public IAsyncRpcChannel {
public:
    explicit FakeChannel(std::string value) : name_(std::move(value)) {}

    const std::string& name() const noexcept override { return name_; }

    AsyncRpcPollResult poll(AsyncRpcRequest& request, std::string&) noexcept override
    {
        if (requests.empty()) return AsyncRpcPollResult::none;
        request = std::move(requests.front());
        requests.pop_front();
        return AsyncRpcPollResult::request;
    }

    bool reply(const AsyncRpcResponse& response, std::string&) noexcept override
    {
        responses.push_back(response);
        return true;
    }

    void push(std::uint64_t step, std::vector<std::byte> payload)
    {
        requests.push_back(AsyncRpcRequest{name_, step, std::move(payload)});
    }

    std::string name_;
    std::deque<AsyncRpcRequest> requests;
    std::vector<AsyncRpcResponse> responses;
};

class FakeApi final : public IHakoniwaAssetApi {
public:
    hako_asset_callbacks_t* callbacks {nullptr};
    int sleeps {0};
    CoreActivity activity {CoreActivity::running};

    int register_asset(const HakoniwaAssetDriverConfig&, hako_asset_callbacks_t* value) noexcept override
    {
        callbacks = value;
        return 0;
    }
    int start_no_wait(int (*)(void)) noexcept override
    {
        if (callbacks->on_initialize(nullptr) != 0) return -1;
        return callbacks->on_manual_timing_control(nullptr);
    }
    int sleep_no_wait(hako_time_t, int (*)(void)) noexcept override
    {
        ++sleeps;
        return 0;
    }
    CoreActivity core_activity() noexcept override { return activity; }
};

class BatchCallbacks final : public IHakoniwaAssetDriverCallbacks {
public:
    explicit BatchCallbacks(std::shared_ptr<AsyncRpcInput> value)
        : input(std::move(value)) {}

    int on_initialize() noexcept override { input->lockstep_prepare(); return 0; }
    int on_step() noexcept override
    {
        if (!input->apply_ready_requests() || !input->complete_step()) {
            error = input->failure_reason();
            stopped = true;
            return -1;
        }
        ++steps;
        if (steps >= max_steps) stopped = true;
        return 0;
    }
    int on_reset() noexcept override { return 0; }
    bool should_stop() const noexcept override { return stopped; }
    int stop_result() const noexcept override { return error.empty() ? 0 : -1; }
    void wait_for_input() noexcept override
    {
        ++input_waits;
        if (on_input_wait) on_input_wait();
    }
    bool ready_to_step() noexcept override
    {
        input->set_waiting_paused(paused);
        const bool ready = input->lockstep_ready();
        if (input->failed()) {
            error = input->failure_reason();
            stopped = true;
        } else if (!ready && stop_after_first_wait) {
            stopped = true;
        }
        return ready;
    }

    std::shared_ptr<AsyncRpcInput> input;
    bool stopped {false};
    bool stop_after_first_wait {false};
    bool paused {false};
    int steps {0};
    int max_steps {1};
    int input_waits {0};
    std::function<void()> on_input_wait;
    std::string error;
};

int run_driver(BatchCallbacks& callbacks, const std::shared_ptr<FakeApi>& api)
{
    HakoniwaAssetDriver driver(
        {"asset", "pdu.json", 1000, HAKO_ASSET_MODEL_PLANT}, callbacks, api);
    assert(driver.register_asset() == 0);
    return driver.start();
}

std::shared_ptr<AsyncRpcInput> make_input(
    const std::shared_ptr<FakeChannel>& a,
    const std::shared_ptr<FakeChannel>& b,
    AsyncRpcInputConfig config,
    AsyncRpcInput::Now now = AsyncRpcInput::Clock::now,
    int* apply_count = nullptr)
{
    return std::make_shared<AsyncRpcInput>(
        std::vector<std::shared_ptr<IAsyncRpcChannel>>{a, b},
        std::move(config),
        [apply_count](std::span<const AsyncRpcRequest> requests, std::string&) {
            assert(requests.size() == 2);
            if (apply_count != nullptr) ++*apply_count;
            return true;
        },
        AsyncRpcBuildResponse{},
        std::move(now));
}

AsyncRpcInputConfig limits(
    std::chrono::milliseconds timeout,
    std::string log_path = {},
    std::chrono::milliseconds initial_timeout = std::chrono::milliseconds {-1})
{
    if (initial_timeout < std::chrono::milliseconds::zero()) {
        initial_timeout = timeout;
    }
    return AsyncRpcInputConfig {timeout, initial_timeout, std::move(log_path)};
}

struct ManualClock {
    AsyncRpcInput::Clock::time_point value {};
    AsyncRpcInput::Clock::time_point now() const { return value; }
    void advance(std::chrono::milliseconds amount) { value += amount; }
};

class PausedWithoutRpcCallbacks final : public IHakoniwaAssetDriverCallbacks {
public:
    int on_initialize() noexcept override { return 0; }
    int on_step() noexcept override
    {
        ++step_calls;
        return 0; // legacy Runner pause: no physics, but Driver still advances time
    }
    int on_reset() noexcept override { return 0; }
    bool should_stop() const noexcept override { return step_calls >= max_steps; }
    bool ready_to_step() noexcept override { return true; }

    int step_calls {0};
    int max_steps {3};
};

} // namespace

int main()
{
    {
        using CoreActivity = IHakoniwaAssetApi::CoreActivity;
        assert(core_activity_from_simulation_state(HAKO_ASSET_SIM_STATE_RUNNING)
            == CoreActivity::running);
        for (const int state : {
                 HAKO_ASSET_SIM_STATE_STOPPED,
                 HAKO_ASSET_SIM_STATE_RUNNABLE,
                 HAKO_ASSET_SIM_STATE_STOPPING,
                 HAKO_ASSET_SIM_STATE_RESETTING,
                 HAKO_ASSET_SIM_STATE_ERROR,
                 HAKO_ASSET_SIM_STATE_TERMINATED,
             }) {
            assert(core_activity_from_simulation_state(state) == CoreActivity::inactive);
        }
        assert(core_activity_from_simulation_state(-1) == CoreActivity::unknown);
        assert(core_activity_from_simulation_state(
                   static_cast<int>(HAKO_ASSET_SIM_STATE_TERMINATED) + 1)
            == CoreActivity::unknown);
    }
    {
        const AsyncRpcInputConfig defaults;
        assert(defaults.timeout == std::chrono::seconds(10));
        assert(defaults.initial_timeout == std::chrono::seconds(60));
    }
    {
        auto a = std::make_shared<FakeChannel>("A");
        auto b = std::make_shared<FakeChannel>("B");
        a->push(0, bytes({1}));
        auto input = make_input(a, b, limits(std::chrono::seconds(10)));
        BatchCallbacks callbacks(input);
        callbacks.stop_after_first_wait = true;
        auto api = std::make_shared<FakeApi>();
        assert(run_driver(callbacks, api) == 0);
        assert(callbacks.steps == 0);
        assert(api->sleeps == 0); // no request batch: neither physics nor time advances
        assert(callbacks.input_waits == 1); // ON: real-time backoff seam, no Hakoniwa time
    }
    {
        const auto log_path = std::filesystem::temp_directory_path()
            / "hakoniwa-async-rpc-input-test.jsonl";
        std::filesystem::remove(log_path);
        auto a = std::make_shared<FakeChannel>("A");
        auto b = std::make_shared<FakeChannel>("B");
        a->push(0, bytes({1, 2, 3}));
        b->push(0, bytes({4, 5}));
        int apply_count = 0;
        auto input = make_input(
            a, b, limits(std::chrono::seconds(10), log_path.string()),
            AsyncRpcInput::Clock::now, &apply_count);
        BatchCallbacks callbacks(input);
        auto api = std::make_shared<FakeApi>();
        assert(run_driver(callbacks, api) == 0);
        assert(callbacks.steps == 1);
        assert(apply_count == 1);
        assert(api->sleeps == 1);
        assert(a->responses.size() == 1 && a->responses[0].step == 0);
        assert(b->responses.size() == 1 && b->responses[0].step == 0);
        std::ifstream log(log_path);
        const std::string contents(
            (std::istreambuf_iterator<char>(log)), std::istreambuf_iterator<char>());
        assert(std::count(contents.begin(), contents.end(), '\n') == 1);
        assert(contents.find("\"step\":0") != std::string::npos);
        assert(contents.find("\"A\":\"fnv1a64:") != std::string::npos);
        assert(contents.find("\"B\":0") != std::string::npos);
        std::filesystem::remove(log_path);
    }
    {
        auto a = std::make_shared<FakeChannel>("A");
        auto b = std::make_shared<FakeChannel>("B");
        a->push(0, bytes({1}));
        b->push(0, bytes({2}));
        a->push(1, bytes({3}));
        auto clock = std::make_shared<AsyncRpcInput::Clock::time_point>();
        auto now = [clock]() mutable {
            const auto result = *clock;
            *clock += std::chrono::milliseconds(5);
            return result;
        };
        auto input = make_input(a, b, limits(std::chrono::milliseconds(10)), now);
        BatchCallbacks callbacks(input);
        callbacks.max_steps = 2;
        auto api = std::make_shared<FakeApi>();
        assert(run_driver(callbacks, api) != 0);
        assert(callbacks.steps == 1);
        assert(api->sleeps == 1);
        assert(callbacks.error.find("B") != std::string::npos);
        assert(callbacks.error.find("k=1") != std::string::npos);
    }
    {
        auto a = std::make_shared<FakeChannel>("A");
        auto b = std::make_shared<FakeChannel>("B");
        a->push(2, bytes({1}));
        auto input = make_input(a, b, limits(std::chrono::seconds(10)));
        BatchCallbacks callbacks(input);
        auto api = std::make_shared<FakeApi>();
        assert(run_driver(callbacks, api) != 0);
        assert(callbacks.error.find("expected k=0") != std::string::npos);
        assert(callbacks.error.find("received k=2") != std::string::npos);
        assert(api->sleeps == 0);
    }
    {
        auto a = std::make_shared<FakeChannel>("A");
        auto b = std::make_shared<FakeChannel>("B");
        a->push(0, bytes({1}));
        a->push(0, bytes({2}));
        auto input = make_input(a, b, limits(std::chrono::seconds(10)));
        BatchCallbacks callbacks(input);
        auto api = std::make_shared<FakeApi>();
        assert(run_driver(callbacks, api) != 0);
        assert(callbacks.error.find("duplicate") != std::string::npos);
        assert(callbacks.error.find("k=0") != std::string::npos);
        assert(api->sleeps == 0);
    }
    {
        auto a = std::make_shared<FakeChannel>("A");
        auto b = std::make_shared<FakeChannel>("B");
        a->push(0, bytes({1}));
        b->push(0, bytes({2}));
        a->push(0, bytes({3})); // expected k=1 after the first batch
        auto input = make_input(a, b, limits(std::chrono::seconds(10)));
        BatchCallbacks callbacks(input);
        callbacks.max_steps = 2;
        auto api = std::make_shared<FakeApi>();
        assert(run_driver(callbacks, api) != 0);
        assert(callbacks.steps == 1);
        assert(callbacks.error.find("expected k=1") != std::string::npos);
        assert(callbacks.error.find("received k=0") != std::string::npos);
        assert(callbacks.error.find("went backwards") != std::string::npos);
        assert(api->sleeps == 1);
    }
    {
        // The first request batch gets the longer default startup allowance.
        auto a = std::make_shared<FakeChannel>("A");
        auto b = std::make_shared<FakeChannel>("B");
        auto clock = std::make_shared<ManualClock>();
        auto input = make_input(
            a, b, AsyncRpcInputConfig{}, [clock]() { return clock->now(); });
        clock->advance(std::chrono::seconds(59));
        assert(!input->lockstep_ready());
        assert(!input->failed());
        clock->advance(std::chrono::seconds(1));
        assert(!input->lockstep_ready());
        assert(input->failed());
        assert(input->failure_reason().find("B") != std::string::npos);
        assert(input->failure_reason().find("k=0") != std::string::npos);
    }
    {
        // After k=0, the ordinary default is ten seconds.
        auto a = std::make_shared<FakeChannel>("A");
        auto b = std::make_shared<FakeChannel>("B");
        a->push(0, bytes({1}));
        b->push(0, bytes({2}));
        auto clock = std::make_shared<ManualClock>();
        auto input = make_input(
            a, b, AsyncRpcInputConfig{}, [clock]() { return clock->now(); });
        assert(input->lockstep_ready());
        assert(input->apply_ready_requests());
        assert(input->complete_step());
        clock->advance(std::chrono::seconds(9));
        assert(!input->lockstep_ready());
        assert(!input->failed());
        clock->advance(std::chrono::seconds(1));
        assert(!input->lockstep_ready());
        assert(input->failed());
        assert(input->failure_reason().find("k=1") != std::string::npos);
    }
    {
        // A pause suspends accounting; resume grants a fresh full timeout.
        auto a = std::make_shared<FakeChannel>("A");
        auto b = std::make_shared<FakeChannel>("B");
        a->push(0, bytes({1}));
        b->push(0, bytes({2}));
        a->push(1, bytes({3}));
        auto clock = std::make_shared<ManualClock>();
        auto input = make_input(
            a, b, AsyncRpcInputConfig{}, [clock]() { return clock->now(); });
        assert(input->lockstep_ready());
        assert(input->apply_ready_requests());
        assert(input->complete_step());
        assert(!input->lockstep_ready()); // A arrived; B is missing for k=1
        input->set_waiting_paused(true);
        clock->advance(std::chrono::seconds(30));
        assert(!input->lockstep_ready());
        assert(!input->failed());
        input->set_waiting_paused(false);
        assert(!input->lockstep_ready()); // observes resume and restarts the clock
        clock->advance(std::chrono::seconds(9));
        assert(!input->lockstep_ready());
        assert(!input->failed());
        clock->advance(std::chrono::seconds(1));
        assert(!input->lockstep_ready());
        assert(input->failed());
        assert(input->failure_reason().find("B") != std::string::npos);
        assert(input->failure_reason().find("k=1") != std::string::npos);
    }
    {
        // With RPC input, a viewer pause blocks both on_step and Hakoniwa time.
        auto a = std::make_shared<FakeChannel>("A");
        auto b = std::make_shared<FakeChannel>("B");
        a->push(0, bytes({1}));
        b->push(0, bytes({2}));
        auto input = make_input(a, b, AsyncRpcInputConfig{});
        BatchCallbacks callbacks(input);
        callbacks.paused = true;
        auto api = std::make_shared<FakeApi>();
        callbacks.on_input_wait = [&]() {
            assert(callbacks.steps == 0);
            assert(api->sleeps == 0);
            callbacks.paused = false;
        };
        assert(run_driver(callbacks, api) == 0);
        assert(callbacks.input_waits == 1);
        assert(callbacks.steps == 1);
        assert(api->sleeps == 1);
        assert(a->responses.size() == 1 && a->responses[0].step == 0);
        assert(b->responses.size() == 1 && b->responses[0].step == 0);
    }
    {
        // A Core stop during an incomplete batch is a quiet normal stop.  The
        // one-hour clock jump must never be observed as an RPC timeout.
        auto a = std::make_shared<FakeChannel>("A");
        auto b = std::make_shared<FakeChannel>("B");
        a->push(0, bytes({1}));
        auto clock = std::make_shared<ManualClock>();
        auto input = make_input(
            a, b, AsyncRpcInputConfig{}, [clock]() { return clock->now(); });
        BatchCallbacks callbacks(input);
        auto api = std::make_shared<FakeApi>();
        callbacks.on_input_wait = [&]() {
            clock->advance(std::chrono::hours(1));
            api->activity = IHakoniwaAssetApi::CoreActivity::inactive;
        };
        assert(run_driver(callbacks, api) == 0);
        assert(callbacks.steps == 0);
        assert(api->sleeps == 0);
        assert(callbacks.input_waits == 1);
        assert(!input->failed());
        assert(input->failure_reason().empty());
    }
    {
        // An unreadable/unknown Core state is not mistaken for a stop.
        auto a = std::make_shared<FakeChannel>("A");
        auto b = std::make_shared<FakeChannel>("B");
        a->push(0, bytes({1}));
        b->push(0, bytes({2}));
        auto input = make_input(a, b, AsyncRpcInputConfig{});
        BatchCallbacks callbacks(input);
        auto api = std::make_shared<FakeApi>();
        api->activity = IHakoniwaAssetApi::CoreActivity::unknown;
        assert(run_driver(callbacks, api) == 0);
        assert(callbacks.steps == 1);
        assert(api->sleeps == 1);
        assert(!input->failed());
    }
    {
        // Feature ON must not globally change the legacy PDU-only pause loop.
        PausedWithoutRpcCallbacks callbacks;
        auto api = std::make_shared<FakeApi>();
        HakoniwaAssetDriver driver(
            {"asset", "pdu.json", 1000, HAKO_ASSET_MODEL_PLANT}, callbacks, api);
        assert(driver.register_asset() == 0);
        assert(driver.start() == 0);
        assert(callbacks.step_calls == 3);
        assert(api->sleeps == 3);
    }
    {
        // Zero means no timeout, including the first batch.
        auto a = std::make_shared<FakeChannel>("A");
        auto b = std::make_shared<FakeChannel>("B");
        auto clock = std::make_shared<ManualClock>();
        auto input = make_input(
            a, b, limits(std::chrono::milliseconds(0)),
            [clock]() { return clock->now(); });
        clock->advance(std::chrono::hours(1));
        assert(!input->lockstep_ready());
        assert(!input->failed());
    }
    {
        auto a = std::make_shared<FakeChannel>("A");
        auto b = std::make_shared<FakeChannel>("B");
        bool rejected = false;
        try {
            (void)make_input(a, b, AsyncRpcInputConfig {
                std::chrono::milliseconds(-1), std::chrono::seconds(60), {}});
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        assert(rejected);
        rejected = false;
        try {
            (void)make_input(a, b, AsyncRpcInputConfig {
                std::chrono::seconds(10), std::chrono::milliseconds(-1), {}});
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        assert(rejected);
    }
    return 0;
}
