#pragma once

#include "hakoniwa/robot_runtime/runner/async_rpc_input.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runner {

/** Core's server state while a request is in progress (IDLE has no request). */
enum class AsyncServiceState {
    doing,
    canceling,
};

/**
 * One service behind one channel (docs/design-async-rpc.md, section 5.3).
 * All hooks run on the simulation thread between steps and must not wait.
 */
class IAsyncService {
public:
    virtual ~IAsyncService() = default;
    /**
     * A request arrived while the service was idle. Return false to refuse it
     * (the input then answers INVALID after physics; error is logged only).
     */
    virtual bool on_request(const AsyncRpcRequest& request, std::string& error) noexcept = 0;
    /** Notice only: the client canceled the request in progress. */
    virtual void on_cancel() noexcept {}
    /**
     * Every step before physics while a request is in progress, starting with
     * the step in which it was accepted. Return true when finished and fill
     * response (result_code ok, canceled or error); it is sent after physics.
     */
    virtual bool on_step(AsyncServiceState state, AsyncRpcResponse& response) noexcept = 0;
    /** The request in progress is dropped (reset); nothing is answered. */
    virtual void reset() noexcept {}
};

struct AsyncServiceBinding {
    std::shared_ptr<IAsyncRpcChannel> channel;
    std::shared_ptr<IAsyncService> service;
};

/**
 * Non-lockstep service input (stage B). Unlike AsyncRpcInput it never holds
 * the step: with no request the simulation advances as usual.
 *
 * Per step:
 *   before_physics()  poll each channel once; a request while idle goes to
 *                     on_request (accepted: in progress, DOING); a cancel while
 *                     in progress becomes CANCELING and calls on_cancel; then
 *                     on_step for every service in progress
 *   after_physics()   send the replies decided in before_physics (finished
 *                     services, refused requests); those services become idle
 *
 * Core never delivers a request while a service is in progress (it answers
 * BUSY itself) and drops a cancel while idle. If a channel still does so, a
 * request while in progress fails the input; a cancel while idle is ignored.
 * A channel poll or reply error fails the input. After failed() no hook runs.
 */
class AsyncServiceInput final {
public:
    explicit AsyncServiceInput(std::vector<AsyncServiceBinding> bindings);
    ~AsyncServiceInput();

    AsyncServiceInput(const AsyncServiceInput&) = delete;
    AsyncServiceInput& operator=(const AsyncServiceInput&) = delete;

    [[nodiscard]] bool prepare_channels() noexcept;
    [[nodiscard]] bool before_physics() noexcept;
    [[nodiscard]] bool after_physics() noexcept;
    /** Drop every request in progress without replies and call each reset(). */
    void reset() noexcept;

    [[nodiscard]] bool failed() const noexcept;
    [[nodiscard]] const std::string& failure_reason() const noexcept;
    /** State of binding index; nullopt while idle or out of range. */
    [[nodiscard]] std::optional<AsyncServiceState> state(std::size_t index) const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::runner
