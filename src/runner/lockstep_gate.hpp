#pragma once

#include "hakoniwa/robot_runtime/runner/lockstep_input.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runner {

class LockstepGate final {
public:
    using Clock = std::chrono::steady_clock;
    using Now = std::function<Clock::time_point()>;

    struct Input {
        std::string name;
        std::shared_ptr<ILockstepInput> source;
    };

    struct Config {
        std::chrono::milliseconds timeout {10000};
        std::chrono::milliseconds initial_timeout {60000};
    };

    explicit LockstepGate(std::vector<Input> inputs, Config config, Now now = Clock::now);

    void prepare() noexcept;
    void reset() noexcept;
    [[nodiscard]] bool ready_to_step() noexcept;
    void set_waiting_paused(bool paused) noexcept;
    [[nodiscard]] bool empty() const noexcept { return inputs_.empty(); }
    [[nodiscard]] bool uses_deadline() const noexcept { return uses_deadline_; }
    [[nodiscard]] bool sleep_before_first_step() const noexcept { return initial_sync_; }
    [[nodiscard]] bool failed() const noexcept { return !failure_.empty(); }
    [[nodiscard]] const std::string& failure_reason() const noexcept { return failure_; }

private:
    void start_run() noexcept;

    std::vector<Input> inputs_;
    std::vector<bool> ready_;
    Config config_;
    Now now_;
    std::optional<Clock::time_point> wait_started_;
    bool uses_deadline_ {false};
    bool initial_sync_ {false};
    bool first_step_ {true};
    bool paused_ {false};
    std::string failure_;
};

} // namespace hakoniwa::robot_runtime::runner
