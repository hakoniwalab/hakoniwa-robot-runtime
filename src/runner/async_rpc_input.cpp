#include "hakoniwa/robot_runtime/runner/async_rpc_input.hpp"

#include <algorithm>
#include <atomic>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runner {
namespace {

std::string json_escape(const std::string& value)
{
    std::ostringstream out;
    for (const unsigned char ch : value) {
        switch (ch) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (ch < 0x20) {
                out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                    << static_cast<unsigned int>(ch) << std::dec;
            } else {
                out << static_cast<char>(ch);
            }
        }
    }
    return out.str();
}

std::string fnv1a64(std::span<const std::byte> bytes)
{
    std::uint64_t hash = 14695981039346656037ULL;
    for (const auto byte : bytes) {
        hash ^= static_cast<std::uint8_t>(byte);
        hash *= 1099511628211ULL;
    }
    std::ostringstream out;
    out << "fnv1a64:" << std::hex << std::setw(16) << std::setfill('0') << hash;
    return out.str();
}

} // namespace

struct AsyncRpcInput::Impl {
    std::vector<std::shared_ptr<IAsyncRpcChannel>> channels;
    AsyncRpcInputConfig config;
    AsyncRpcApplyBatch apply_batch;
    AsyncRpcBuildResponse build_response;
    Now now;
    Clock::time_point wait_started;
    std::uint64_t expected {0};
    std::map<std::string, AsyncRpcRequest> pending;
    std::string failure;
    bool applied {false};
    std::atomic_bool pause_requested {false};
    bool pause_observed {false};
    std::ofstream log;

    void fail(std::string message) noexcept
    {
        if (failure.empty()) {
            failure = std::move(message);
        }
    }

    [[nodiscard]] std::vector<AsyncRpcRequest> ordered_requests() const
    {
        std::vector<AsyncRpcRequest> result;
        result.reserve(channels.size());
        for (const auto& channel : channels) {
            result.push_back(pending.at(channel->name()));
        }
        return result;
    }

    void write_log(const std::vector<AsyncRpcRequest>& requests)
    {
        if (!log.is_open()) {
            return;
        }
        log << "{\"step\":" << expected << ",\"requests\":{";
        bool first = true;
        for (const auto& request : requests) {
            if (!first) log << ',';
            first = false;
            log << '"' << json_escape(request.channel) << "\":\""
                << fnv1a64(request.payload) << '"';
        }
        log << "},\"responses\":{";
        first = true;
        for (const auto& request : requests) {
            if (!first) log << ',';
            first = false;
            log << '"' << json_escape(request.channel) << "\":" << expected;
        }
        log << "}}\n";
        log.flush();
        if (!log) {
            throw std::runtime_error("failed to write async RPC step log");
        }
    }
};

AsyncRpcInput::AsyncRpcInput(
    std::vector<std::shared_ptr<IAsyncRpcChannel>> channels,
    AsyncRpcInputConfig config,
    AsyncRpcApplyBatch apply_batch,
    AsyncRpcBuildResponse build_response,
    Now now)
    : impl_(std::make_unique<Impl>())
{
    if (channels.empty()) {
        throw std::invalid_argument("async RPC input needs at least one channel");
    }
    if (config.timeout < std::chrono::milliseconds::zero()
        || config.initial_timeout < std::chrono::milliseconds::zero()) {
        throw std::invalid_argument("async RPC input timeouts must not be negative");
    }
    if (!now) {
        throw std::invalid_argument("async RPC input clock must be provided");
    }
    std::map<std::string, bool> names;
    for (const auto& channel : channels) {
        if (channel == nullptr || channel->name().empty()) {
            throw std::invalid_argument("async RPC input channel/name must not be empty");
        }
        if (!names.emplace(channel->name(), true).second) {
            throw std::invalid_argument("duplicate async RPC input channel: " + channel->name());
        }
    }
    impl_->channels = std::move(channels);
    impl_->config = std::move(config);
    impl_->apply_batch = apply_batch ? std::move(apply_batch)
                                     : [](std::span<const AsyncRpcRequest>, std::string&) { return true; };
    impl_->build_response = build_response
        ? std::move(build_response)
        : [](const AsyncRpcRequest& request, AsyncRpcResponse& response, std::string&) {
              response.step = request.step;
              return true;
          };
    impl_->now = std::move(now);
    if (!impl_->config.step_log_path.empty()) {
        impl_->log.open(impl_->config.step_log_path, std::ios::out | std::ios::trunc);
        if (!impl_->log) {
            throw std::runtime_error(
                "failed to open async RPC step log: " + impl_->config.step_log_path);
        }
    }
    lockstep_prepare();
}

AsyncRpcInput::~AsyncRpcInput() = default;

bool AsyncRpcInput::prepare_channels() noexcept
{
    if (!impl_->failure.empty()) {
        return false;
    }
    for (const auto& channel : impl_->channels) {
        std::string error;
        if (!channel->prepare(error)) {
            impl_->fail("async RPC channel " + channel->name()
                        + " preparation failed: " + error);
            return false;
        }
    }
    return true;
}

void AsyncRpcInput::lockstep_prepare() noexcept
{
    impl_->expected = 0;
    impl_->pending.clear();
    impl_->failure.clear();
    impl_->applied = false;
    impl_->pause_observed = impl_->pause_requested.load();
    impl_->wait_started = impl_->now();
}

bool AsyncRpcInput::lockstep_ready() noexcept
{
    if (!impl_->failure.empty()) {
        return false;
    }
    if (impl_->pause_requested.load()) {
        impl_->pause_observed = true;
        return false;
    }
    if (impl_->pause_observed) {
        impl_->pause_observed = false;
        impl_->wait_started = impl_->now();
    }
    for (const auto& channel : impl_->channels) {
        AsyncRpcRequest request;
        std::string error;
        const auto result = channel->poll(request, error);
        if (result == AsyncRpcPollResult::error) {
            impl_->fail("async RPC channel " + channel->name() + " poll failed: " + error);
            return false;
        }
        if (result == AsyncRpcPollResult::none) {
            continue;
        }
        request.channel = channel->name();
        if (request.step != impl_->expected) {
            std::ostringstream message;
            message << "async RPC channel " << channel->name() << " expected k="
                    << impl_->expected << " but received k=" << request.step
                    << (request.step < impl_->expected ? " (step went backwards)" : " (step skipped)");
            impl_->fail(message.str());
            return false;
        }
        if (impl_->pending.contains(channel->name())) {
            impl_->fail("async RPC channel " + channel->name()
                        + " received duplicate request for k=" + std::to_string(request.step));
            return false;
        }
        impl_->pending.emplace(channel->name(), std::move(request));
    }
    if (impl_->pending.size() == impl_->channels.size()) {
        return true;
    }
    const auto timeout = impl_->expected == 0
        ? impl_->config.initial_timeout : impl_->config.timeout;
    if (timeout != std::chrono::milliseconds::zero()
        && impl_->now() - impl_->wait_started >= timeout) {
        std::ostringstream message;
        message << "async RPC timeout: missing";
        for (const auto& channel : impl_->channels) {
            if (!impl_->pending.contains(channel->name())) {
                message << ' ' << channel->name();
            }
        }
        message << ", k=" << impl_->expected;
        impl_->fail(message.str());
    }
    return false;
}

void AsyncRpcInput::set_waiting_paused(bool paused) noexcept
{
    impl_->pause_requested.store(paused);
}

bool AsyncRpcInput::apply_ready_requests() noexcept
{
    if (!impl_->failure.empty() || impl_->pending.size() != impl_->channels.size()) {
        if (impl_->failure.empty()) {
            impl_->fail("async RPC requests were applied before the batch was complete");
        }
        return false;
    }
    try {
        auto requests = impl_->ordered_requests();
        std::string error;
        if (!impl_->apply_batch(requests, error)) {
            impl_->fail("async RPC request application failed: " + error);
            return false;
        }
        impl_->applied = true;
        return true;
    } catch (const std::exception& error) {
        impl_->fail(std::string("async RPC request application failed: ") + error.what());
        return false;
    }
}

bool AsyncRpcInput::complete_step() noexcept
{
    if (!impl_->failure.empty() || !impl_->applied) {
        if (impl_->failure.empty()) {
            impl_->fail("async RPC step completed before requests were applied");
        }
        return false;
    }
    try {
        const auto requests = impl_->ordered_requests();
        for (std::size_t index = 0; index < impl_->channels.size(); ++index) {
            AsyncRpcResponse response {impl_->expected, {}};
            std::string error;
            if (!impl_->build_response(requests[index], response, error)) {
                impl_->fail("async RPC response build failed for "
                            + impl_->channels[index]->name() + ": " + error);
                return false;
            }
            if (response.step != impl_->expected) {
                impl_->fail("async RPC response step mismatch for "
                            + impl_->channels[index]->name());
                return false;
            }
            if (!impl_->channels[index]->reply(response, error)) {
                impl_->fail("async RPC response failed for "
                            + impl_->channels[index]->name() + ": " + error);
                return false;
            }
        }
        impl_->write_log(requests);
        ++impl_->expected;
        impl_->pending.clear();
        impl_->applied = false;
        impl_->wait_started = impl_->now();
        return true;
    } catch (const std::exception& error) {
        impl_->fail(std::string("async RPC step completion failed: ") + error.what());
        return false;
    }
}

bool AsyncRpcInput::failed() const noexcept
{
    return !impl_->failure.empty();
}

const std::string& AsyncRpcInput::failure_reason() const noexcept
{
    return impl_->failure;
}

std::uint64_t AsyncRpcInput::expected_step() const noexcept
{
    return impl_->expected;
}

} // namespace hakoniwa::robot_runtime::runner
