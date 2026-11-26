#include "runner/lockstep_gate.hpp"

#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runner {

LockstepGate::LockstepGate(std::vector<Input> inputs, Config config, Now now)
    : inputs_(std::move(inputs)), ready_(inputs_.size()), config_(config), now_(std::move(now))
{
    if (!now_ || config_.timeout < std::chrono::milliseconds::zero()
        || config_.initial_timeout < std::chrono::milliseconds::zero()) {
        throw std::invalid_argument("lockstep gate needs a clock and nonnegative timeouts");
    }
    for (const auto& input : inputs_) {
        if (input.name.empty() || !input.source) {
            throw std::invalid_argument("lockstep input needs a name and a source");
        }
        uses_deadline_ |= input.source->lockstep_uses_deadline();
        initial_sync_ |= input.source->lockstep_needs_initial_sync();
    }
}

void LockstepGate::prepare() noexcept
{
    for (const auto& input : inputs_) {
        input.source->lockstep_prepare();
    }
    start_run();
}

void LockstepGate::reset() noexcept
{
    for (const auto& input : inputs_) {
        input.source->lockstep_reset();
    }
    start_run();
}

void LockstepGate::start_run() noexcept
{
    failure_.clear();
    wait_started_.reset();
    first_step_ = true;
    if (uses_deadline_ && !paused_) {
        try {
            wait_started_ = now_();
        } catch (const std::exception& error) {
            failure_ = std::string("lockstep clock failed: ") + error.what();
        } catch (...) {
            failure_ = "lockstep clock failed";
        }
    }
}

bool LockstepGate::ready_to_step() noexcept
{
    if (failed() || paused_) {
        return false;
    }

    try {
        bool all_ready = true;
        for (std::size_t i = 0; i < inputs_.size(); ++i) {
            const auto& [name, source] = inputs_[i];
            ready_[i] = source->lockstep_ready();
            const auto error = source->lockstep_failure_reason();
            if (!error.empty()) {
                failure_ = "lockstep input " + name + ": ";
                failure_ += error;
                return false;
            }
            if (!ready_[i]) {
                if (!uses_deadline_) {
                    return false;  // Preserve the untimed PDU polling order.
                }
                all_ready = false;
            }
        }
        if (all_ready) {
            wait_started_.reset();
            first_step_ = false;
            return true;
        }

        const auto timeout = first_step_ ? config_.initial_timeout : config_.timeout;
        const auto now = now_();
        if (!wait_started_) {
            wait_started_ = now;
        }
        if (timeout != std::chrono::milliseconds::zero() && now - *wait_started_ >= timeout) {
            failure_ = "lockstep timeout: missing";
            for (std::size_t i = 0; i < inputs_.size(); ++i) {
                if (!ready_[i]) {
                    failure_ += ' ';
                    failure_ += inputs_[i].name;
                }
            }
        }
    } catch (const std::exception& error) {
        failure_ = std::string("lockstep gate failed: ") + error.what();
    } catch (...) {
        failure_ = "lockstep gate failed with an unknown exception";
    }
    return false;
}

void LockstepGate::set_waiting_paused(bool paused) noexcept
{
    if (uses_deadline_ && paused_ != paused) {
        paused_ = paused;
        wait_started_.reset();
    }
}

} // namespace hakoniwa::robot_runtime::runner
