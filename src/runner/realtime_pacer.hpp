#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>

namespace hakoniwa::robot_runtime::runner {

/** Wall-clock pacing utility owned by the Runtime Runner layer. */
class RealtimePacer final {
public:
    explicit RealtimePacer(std::uint64_t sync_cycle_msec)
        : sync_cycle_sec_(static_cast<double>(sync_cycle_msec) / 1000.0)
    {
    }

    [[nodiscard]] bool enabled() const noexcept
    {
        return sync_cycle_sec_ > 0.0;
    }

    void reset() noexcept
    {
        initialized_ = false;
    }

    void pace(double simulation_time_sec)
    {
        if (!enabled()) {
            return;
        }

        const auto now = Clock::now();
        if (!initialized_ || simulation_time_sec < last_simulation_time_sec_) {
            simulation_anchor_sec_ = simulation_time_sec;
            wall_anchor_ = now;
            next_sync_simulation_sec_ = simulation_time_sec + sync_cycle_sec_;
            last_simulation_time_sec_ = simulation_time_sec;
            initialized_ = true;
            return;
        }

        last_simulation_time_sec_ = simulation_time_sec;
        if (simulation_time_sec + 1.0e-12 < next_sync_simulation_sec_) {
            return;
        }

        const auto target = wall_anchor_ + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(
                simulation_time_sec - simulation_anchor_sec_));
        if (now < target) {
            std::this_thread::sleep_until(target);
        }

        const auto completed_cycles = std::floor(
            (simulation_time_sec - simulation_anchor_sec_) / sync_cycle_sec_);
        next_sync_simulation_sec_ = simulation_anchor_sec_
            + (completed_cycles + 1.0) * sync_cycle_sec_;
    }

private:
    using Clock = std::chrono::steady_clock;

    double sync_cycle_sec_ {0.0};
    double simulation_anchor_sec_ {0.0};
    double next_sync_simulation_sec_ {0.0};
    double last_simulation_time_sec_ {0.0};
    Clock::time_point wall_anchor_ {};
    bool initialized_ {false};
};

} // namespace hakoniwa::robot_runtime::runner
