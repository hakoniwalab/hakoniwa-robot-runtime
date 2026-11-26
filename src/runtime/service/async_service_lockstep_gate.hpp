#pragma once

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

class AsyncServiceCommandSource;

struct AsyncServiceLockstepConfig {
    std::chrono::milliseconds timeout {10000};
    std::chrono::milliseconds initial_timeout {60000};
};

class AsyncServiceLockstepGate final {
public:
    using Clock = std::chrono::steady_clock;
    using Now = std::function<Clock::time_point()>;

    AsyncServiceLockstepGate(
        std::vector<std::shared_ptr<AsyncServiceCommandSource>> sources,
        AsyncServiceLockstepConfig config = {}, Now now = Clock::now);

    /** Starts a new Core run, discarding requests staged for the previous run. */
    void prepare() noexcept;
    [[nodiscard]] bool ready_to_step() noexcept;
    void set_waiting_paused(bool paused) noexcept;
    [[nodiscard]] bool empty() const noexcept;
    [[nodiscard]] bool failed() const noexcept;
    [[nodiscard]] const std::string& failure_reason() const noexcept;

private:
    bool poll_source(AsyncServiceCommandSource& source);

    std::vector<std::shared_ptr<AsyncServiceCommandSource>> sources_;
    AsyncServiceLockstepConfig config_;
    Now now_;
    std::optional<Clock::time_point> wait_started_;
    bool first_batch_ {true};
    bool paused_ {false};
    std::string failure_;
};

} // namespace hakoniwa::robot_runtime::runtime

#endif
