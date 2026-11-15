#include "hakoniwa/robot_runtime/runner/async_service_input.hpp"

#include <iostream>
#include <map>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runner {

struct AsyncServiceInput::Impl {
    struct Slot {
        AsyncServiceBinding binding;
        std::optional<AsyncServiceState> state;
        std::optional<AsyncRpcResponse> pending_response;
    };

    std::vector<Slot> slots;
    std::string failure;

    void fail(std::string message) noexcept
    {
        if (!failure.empty()) {
            return;
        }
        try {
            failure = std::move(message);
        } catch (...) {
            try {
                failure = "async service input failed";
            } catch (...) {
                // There is no useful recovery from an allocation failure here,
                // but a noexcept entry point must still not let it escape.
            }
        }
    }

    void fail_for_channel(
        const std::string& channel,
        const std::string& operation,
        const std::string& detail) noexcept
    {
        try {
            std::string message = "async service channel " + channel + ' ' + operation;
            if (!detail.empty()) {
                message += ": " + detail;
            }
            fail(std::move(message));
        } catch (...) {
            fail("async service input failed while reporting a channel error");
        }
    }
};

AsyncServiceInput::AsyncServiceInput(std::vector<AsyncServiceBinding> bindings)
    : impl_(std::make_unique<Impl>())
{
    if (bindings.empty()) {
        throw std::invalid_argument("async service input needs at least one binding");
    }

    std::map<std::string, bool> names;
    impl_->slots.reserve(bindings.size());
    for (auto& binding : bindings) {
        if (binding.channel == nullptr || binding.service == nullptr
            || binding.channel->name().empty()) {
            throw std::invalid_argument(
                "async service input channel/name/service must not be empty");
        }
        if (!names.emplace(binding.channel->name(), true).second) {
            throw std::invalid_argument(
                "duplicate async service input channel: " + binding.channel->name());
        }
        impl_->slots.push_back(Impl::Slot {std::move(binding), std::nullopt, std::nullopt});
    }
}

AsyncServiceInput::~AsyncServiceInput() = default;

bool AsyncServiceInput::prepare_channels() noexcept
{
    if (!impl_->failure.empty()) {
        return false;
    }

    try {
        for (const auto& slot : impl_->slots) {
            std::string error;
            if (!slot.binding.channel->prepare(error)) {
                impl_->fail_for_channel(
                    slot.binding.channel->name(), "preparation failed", error);
                return false;
            }
        }
        return true;
    } catch (const std::exception& error) {
        impl_->fail(std::string("async service channel preparation failed: ") + error.what());
        return false;
    } catch (...) {
        impl_->fail("async service channel preparation failed: unknown exception");
        return false;
    }
}

bool AsyncServiceInput::before_physics() noexcept
{
    if (!impl_->failure.empty()) {
        return false;
    }

    try {
        for (auto& slot : impl_->slots) {
            AsyncRpcRequest request;
            std::string error;
            const auto result = slot.binding.channel->poll(request, error);

            if (result == AsyncRpcPollResult::error) {
                impl_->fail_for_channel(slot.binding.channel->name(), "poll failed", error);
                return false;
            }
            if (result == AsyncRpcPollResult::none) {
                continue;
            }
            if (result == AsyncRpcPollResult::cancel) {
                if (slot.state == AsyncServiceState::doing) {
                    slot.state = AsyncServiceState::canceling;
                    slot.binding.service->on_cancel();
                }
                continue;
            }

            if (slot.state.has_value()) {
                impl_->fail_for_channel(
                    slot.binding.channel->name(),
                    "received a request while another request was in progress",
                    "the channel violated the Core service contract");
                return false;
            }

            request.channel = slot.binding.channel->name();
            if (slot.binding.service->on_request(request, error)) {
                slot.state = AsyncServiceState::doing;
            } else {
                slot.pending_response = AsyncRpcResponse {
                    request.step, {}, AsyncRpcResultCode::invalid};
                try {
                    std::cerr << "[WARN] async service channel "
                              << slot.binding.channel->name()
                              << " refused request: " << error << '\n';
                } catch (...) {
                    // Refusal is not an input failure; logging must not change that.
                }
            }
        }

        for (auto& slot : impl_->slots) {
            if (!slot.state.has_value() || slot.pending_response.has_value()) {
                continue;
            }
            AsyncRpcResponse response;
            if (slot.binding.service->on_step(*slot.state, response)) {
                slot.pending_response = std::move(response);
            }
        }
        return true;
    } catch (const std::exception& error) {
        impl_->fail(std::string("async service input before physics failed: ") + error.what());
        return false;
    } catch (...) {
        impl_->fail("async service input before physics failed: unknown exception");
        return false;
    }
}

bool AsyncServiceInput::after_physics() noexcept
{
    if (!impl_->failure.empty()) {
        return false;
    }

    try {
        for (auto& slot : impl_->slots) {
            if (!slot.pending_response.has_value()) {
                continue;
            }
            std::string error;
            if (!slot.binding.channel->reply(*slot.pending_response, error)) {
                impl_->fail_for_channel(
                    slot.binding.channel->name(), "reply failed", error);
                return false;
            }
            slot.pending_response.reset();
            slot.state.reset();
        }
        return true;
    } catch (const std::exception& error) {
        impl_->fail(std::string("async service input after physics failed: ") + error.what());
        return false;
    } catch (...) {
        impl_->fail("async service input after physics failed: unknown exception");
        return false;
    }
}

void AsyncServiceInput::reset() noexcept
{
    if (!impl_->failure.empty()) {
        return;
    }

    try {
        for (auto& slot : impl_->slots) {
            slot.binding.service->reset();
            slot.state.reset();
            slot.pending_response.reset();
        }
    } catch (const std::exception& error) {
        impl_->fail(std::string("async service input reset failed: ") + error.what());
    } catch (...) {
        impl_->fail("async service input reset failed: unknown exception");
    }
}

bool AsyncServiceInput::failed() const noexcept
{
    return !impl_->failure.empty();
}

const std::string& AsyncServiceInput::failure_reason() const noexcept
{
    return impl_->failure;
}

std::optional<AsyncServiceState> AsyncServiceInput::state(std::size_t index) const noexcept
{
    if (index >= impl_->slots.size()) {
        return std::nullopt;
    }
    return impl_->slots[index].state;
}

} // namespace hakoniwa::robot_runtime::runner
