#pragma once

#include "hakoniwa/robot_runtime/runner/lockstep_input.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runner {

struct AsyncRpcRequest {
    std::string channel;
    std::uint64_t step {0};
    std::vector<std::byte> payload;
};

struct AsyncRpcResponse {
    std::uint64_t step {0};
    std::vector<std::byte> payload;
};

enum class AsyncRpcPollResult {
    none,
    request,
    error,
};

/**
 * One robot's service channel.  The Core-backed implementation owns one
 * hako_asset_service server; tests and future transports can replace it.
 * Packet decoding belongs here, so #16 can map its typed Ackermann srv to the
 * common step + opaque-payload contract without changing the batch logic.
 */
class IAsyncRpcChannel {
public:
    virtual ~IAsyncRpcChannel() = default;
    /** Prepare transport resources after Core service initialization. */
    virtual bool prepare(std::string& error) noexcept
    {
        (void)error;
        return true;
    }
    [[nodiscard]] virtual const std::string& name() const noexcept = 0;
    virtual AsyncRpcPollResult poll(AsyncRpcRequest& request, std::string& error) noexcept = 0;
    virtual bool reply(const AsyncRpcResponse& response, std::string& error) noexcept = 0;
};

/** Hook where a typed input (#16 car, #17 person) applies a complete batch. */
using AsyncRpcApplyBatch =
    std::function<bool(std::span<const AsyncRpcRequest>, std::string& error)>;

/** Hook where a typed input builds one response after the physics step. */
using AsyncRpcBuildResponse =
    std::function<bool(const AsyncRpcRequest&, AsyncRpcResponse&, std::string& error)>;

struct AsyncRpcInputConfig {
    /** Per-step wait after k=0. Zero disables the limit. */
    std::chrono::milliseconds timeout {10000};
    /** First-step wait for peer asset startup. Zero disables the limit. */
    std::chrono::milliseconds initial_timeout {60000};
    std::string step_log_path;
};

/**
 * Collects exactly one request for expected step k from every robot channel.
 * It is an ILockstepInput, so Hakoniwa time does not advance until the batch is
 * complete.  apply_ready_requests() is called before physics and
 * complete_step() replies/logs after physics.
 */
class AsyncRpcInput final : public ILockstepInput {
public:
    using Clock = std::chrono::steady_clock;
    using Now = std::function<Clock::time_point()>;

    AsyncRpcInput(
        std::vector<std::shared_ptr<IAsyncRpcChannel>> channels,
        AsyncRpcInputConfig config,
        AsyncRpcApplyBatch apply_batch = {},
        AsyncRpcBuildResponse build_response = {},
        Now now = Clock::now);
    ~AsyncRpcInput() override;

    [[nodiscard]] bool lockstep_ready() noexcept override;
    void lockstep_prepare() noexcept override;

    [[nodiscard]] bool prepare_channels() noexcept;
    [[nodiscard]] bool apply_ready_requests() noexcept;
    [[nodiscard]] bool complete_step() noexcept;
    /** Pause timeout accounting. Resuming starts a fresh full timeout. */
    void set_waiting_paused(bool paused) noexcept;
    [[nodiscard]] bool failed() const noexcept;
    [[nodiscard]] const std::string& failure_reason() const noexcept;
    [[nodiscard]] std::uint64_t expected_step() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::runner
