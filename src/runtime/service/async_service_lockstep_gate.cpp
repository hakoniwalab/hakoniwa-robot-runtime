#include "runtime/service/async_service_lockstep_gate.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE

#include "runtime/service/async_service_pipeline.hpp"

#include <sstream>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

AsyncServiceLockstepGate::AsyncServiceLockstepGate(
    std::vector<std::shared_ptr<AsyncServiceCommandSource>> sources,
    AsyncServiceLockstepConfig config, Now now)
    : config_(config), now_(std::move(now))
{
    if (!now_ || config_.timeout < std::chrono::milliseconds::zero()
        || config_.initial_timeout < std::chrono::milliseconds::zero()) {
        throw std::invalid_argument("async service lockstep needs a clock and nonnegative timeouts");
    }
    for (auto& source : sources) {
        if (!source) {
            throw std::invalid_argument("async service lockstep source must not be null");
        }
        if (source->lockstep_) {
            sources_.push_back(std::move(source));
        }
    }
}

void AsyncServiceLockstepGate::prepare() noexcept
{
    failure_.clear();
    wait_started_.reset();
    first_batch_ = true;
    for (const auto& source : sources_) {
        source->staged_request_.reset();
    }
    if (!paused_ && !empty()) {
        try {
            wait_started_ = now_();
        } catch (const std::exception& error) {
            failure_ = std::string("async service lockstep clock failed: ") + error.what();
        } catch (...) {
            failure_ = "async service lockstep clock failed";
        }
    }
}

bool AsyncServiceLockstepGate::poll_source(AsyncServiceCommandSource& source)
{
    const char* error;
    switch (source.server_->poll(source.service_id_)) {
    case AsyncServiceEvent::None:
        return true;
    case AsyncServiceEvent::Request: {
        std::vector<std::byte> request;
        if (source.server_->get_request(source.service_id_, request)) {
            source.staged_request_ = std::move(request);
            return true;
        }
        error = "failed to read request";
        break;
    }
    case AsyncServiceEvent::Cancel:
        error = "request was canceled";
        break;
    default:
        error = "failed to poll server";
        break;
    }
    failure_ = "async service lockstep " + source.id_ + ": " + error;
    return false;
}

bool AsyncServiceLockstepGate::ready_to_step() noexcept
{
    if (failed() || paused_) {
        return false;
    }

    try {
        bool all_staged = true;
        for (const auto& source : sources_) {
            if (!source->staged_request_ && !poll_source(*source)) {
                return false;
            }
            all_staged = all_staged && source->staged_request_.has_value();
        }
        if (all_staged) {
            wait_started_.reset();
            first_batch_ = false;
            return true;
        }

        const auto timeout = first_batch_ ? config_.initial_timeout : config_.timeout;
        const auto now = now_();
        if (!wait_started_) {
            wait_started_ = now;
        }
        if (timeout != std::chrono::milliseconds::zero() && now - *wait_started_ >= timeout) {
            std::ostringstream missing;
            missing << "async service lockstep timeout: missing";
            for (const auto& source : sources_) {
                if (!source->staged_request_) {
                    missing << ' ' << source->id();
                }
            }
            failure_ = missing.str();
        }
    } catch (const std::exception& error) {
        failure_ = std::string("async service lockstep failed: ") + error.what();
    } catch (...) {
        failure_ = "async service lockstep failed with an unknown exception";
    }
    return false;
}

void AsyncServiceLockstepGate::set_waiting_paused(bool paused) noexcept
{
    if (paused_ != paused) {
        paused_ = paused;
        wait_started_.reset();
    }
}

bool AsyncServiceLockstepGate::empty() const noexcept { return sources_.empty(); }
bool AsyncServiceLockstepGate::failed() const noexcept { return !failure_.empty(); }
const std::string& AsyncServiceLockstepGate::failure_reason() const noexcept { return failure_; }

} // namespace hakoniwa::robot_runtime::runtime

#endif
