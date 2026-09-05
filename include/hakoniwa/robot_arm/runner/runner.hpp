#pragma once

#include <atomic>
#include <mutex>

struct mjData_;
struct mjModel_;
using mjData = mjData_;
using mjModel = mjModel_;

namespace hakoniwa::robot_runtime::runner {

/**
 * Application-facing execution contract for one configured Robot Arm Runtime.
 *
 * The concrete execution policy (Hakoniwa Asset today, Standalone in the
 * future) is selected by composition code. Application code only starts,
 * observes, controls, and waits for the returned Runner.
 *
 * The MuJoCo view accessors are kept in this current application contract so
 * the existing Viewer remains independent from concrete Runner classes. If a
 * non-MuJoCo presentation backend is introduced, that view capability can be
 * split from the execution contract without changing ActuatorRuntime.
 */
class IRunner {
public:
    virtual ~IRunner() = default;

    [[nodiscard]] virtual int start() = 0;
    [[nodiscard]] virtual int wait() = 0;

    virtual void request_stop() noexcept = 0;
    virtual void toggle_pause() noexcept = 0;
    virtual void request_reset() noexcept = 0;

    [[nodiscard]] virtual mjModel* model() noexcept = 0;
    [[nodiscard]] virtual mjData* data() noexcept = 0;
    [[nodiscard]] virtual std::mutex& model_mutex() noexcept = 0;
    [[nodiscard]] virtual std::atomic_bool& running() noexcept = 0;
};

} // namespace hakoniwa::robot_runtime::runner
