// The Driver's lockstep loop: one time wait before the first step (Hakoniwa's PDU
// sync mode holds time there until every asset has initialized), and no step and
// no time advance while an input the next step waits for has not arrived.
#undef NDEBUG  // the checks below are asserts; keep them in Release builds
#include "runner/hakoniwa/hakoniwa_asset_driver.hpp"

#include <cassert>
#include <memory>
#include <string>
#include <vector>

using namespace hakoniwa::robot_runtime::runner::detail;

namespace {

std::vector<std::string> trace;

class FakeApi final : public IHakoniwaAssetApi {
public:
    hako_asset_callbacks_t* callbacks {nullptr};
    int register_asset(const HakoniwaAssetDriverConfig&, hako_asset_callbacks_t* table) noexcept override
    {
        callbacks = table;
        return 0;
    }
    int start_no_wait(int (*)(void)) noexcept override
    {
        callbacks->on_initialize(nullptr);
        return callbacks->on_manual_timing_control(nullptr);
    }
    int sleep_no_wait(hako_time_t usec, int (*)(void)) noexcept override
    {
        trace.push_back("sleep " + std::to_string(usec));
        return 0;
    }
};

class Callbacks final : public IHakoniwaAssetDriverCallbacks {
public:
    bool lockstep {false};
    std::vector<bool> ready;  // ready_to_step answers, in order (then true)
    std::size_t asked {0};
    int steps {0};
    int max_steps {3};

    int on_initialize() noexcept override { trace.push_back("init"); return 0; }
    int on_step() noexcept override { trace.push_back("step"); ++steps; return 0; }
    int on_reset() noexcept override { return 0; }
    [[nodiscard]] bool should_stop() const noexcept override { return steps >= max_steps; }
    [[nodiscard]] bool ready_to_step() noexcept override
    {
        const bool answer = asked < ready.size() ? ready[asked] : true;
        ++asked;
        if (!answer) {
            trace.push_back("wait");
        }
        return answer;
    }
    [[nodiscard]] bool sleep_before_first_step() const noexcept override { return lockstep; }
};

std::vector<std::string> run(Callbacks& callbacks)
{
    trace.clear();
    auto api = std::make_shared<FakeApi>();
    HakoniwaAssetDriver driver({"asset", "pdu.json", 2000, HAKO_ASSET_MODEL_PLANT}, callbacks, api);
    assert(driver.register_asset() == 0);
    assert(driver.start() == 0);
    return trace;
}

} // namespace

int main()
{
    {
        // Without lockstep: step, then wait for time.
        Callbacks callbacks;
        const auto got = run(callbacks);
        const std::vector<std::string> expected {
            "init", "step", "sleep 2000", "step", "sleep 2000", "step", "sleep 2000"};
        assert(got == expected);
    }
    {
        // Lockstep: wait for time first; no step and no time advance while not ready.
        Callbacks callbacks;
        callbacks.lockstep = true;
        callbacks.ready = {true, false, false, true};
        const auto got = run(callbacks);
        const std::vector<std::string> expected {
            "init", "sleep 2000", "step", "sleep 2000", "wait", "wait", "step", "sleep 2000",
            "step", "sleep 2000"};
        assert(got == expected);
    }
    return 0;
}
