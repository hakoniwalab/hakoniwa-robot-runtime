// Stage A of docs/design-async-rpc.md: a cancel reaches the service as an
// event and the service answers it with CANCELED.
//
// The server drives one CoreAsyncRpcServiceChannel directly (no lockstep
// input). AddTwoInts is read as: a = 0 -> answer at once with OK and
// sum = b; a = 1 -> hold the request until the client cancels it, then answer
// with CANCELED and sum = -b.
#include "hakoniwa/robot_runtime/runner/hakoniwa/async_rpc_service_channel.hpp"
#include "runner/hakoniwa/hakoniwa_asset_driver.hpp"

#include "hako_asset_service.h"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsResponsePacket.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

using hakoniwa::robot_runtime::runner::AsyncRpcPollResult;
using hakoniwa::robot_runtime::runner::AsyncRpcRequest;
using hakoniwa::robot_runtime::runner::AsyncRpcResponse;
using hakoniwa::robot_runtime::runner::AsyncRpcResultCode;
using hakoniwa::robot_runtime::runner::detail::HakoniwaAssetDriver;
using hakoniwa::robot_runtime::runner::detail::HakoniwaAssetDriverConfig;
using hakoniwa::robot_runtime::runner::detail::IHakoniwaAssetDriverCallbacks;
using hakoniwa::robot_runtime::runner::hakoniwa::CoreAsyncRpcServiceChannel;

namespace {
constexpr const char* kAsset = "AsyncRpcCancelServer";
constexpr const char* kService = "Service/AsyncA";

std::vector<std::byte> encode_value(const std::int64_t value)
{
    std::vector<std::byte> result(sizeof(value));
    std::memcpy(result.data(), &value, sizeof(value));
    return result;
}

std::int64_t decode_value(std::span<const std::byte> payload)
{
    std::int64_t value = 0;
    if (payload.size() == sizeof(value)) std::memcpy(&value, payload.data(), sizeof(value));
    return value;
}

const char* code_name(AsyncRpcResultCode code)
{
    switch (code) {
    case AsyncRpcResultCode::ok: return "ok";
    case AsyncRpcResultCode::error: return "error";
    case AsyncRpcResultCode::canceled: return "canceled";
    case AsyncRpcResultCode::invalid: return "invalid";
    }
    return "?";
}

std::shared_ptr<CoreAsyncRpcServiceChannel> make_channel()
{
    auto decoder = [](std::span<const std::byte> packet, AsyncRpcRequest& request,
                       std::string& error) {
        HakoCpp_AddTwoIntsRequestPacket decoded {};
        if (!hako::pdu::msgs::hako_srv_msgs::AddTwoIntsRequestPacket {}.pdu2cpp(
                const_cast<char*>(reinterpret_cast<const char*>(packet.data())), decoded)) {
            error = "failed to decode AddTwoInts request";
            return false;
        }
        request.step = static_cast<std::uint64_t>(decoded.body.a);
        request.payload = encode_value(decoded.body.b);
        return true;
    };
    auto encoder = [](const AsyncRpcResponse& response, std::span<std::byte> packet,
                       std::string& error) {
        HakoCpp_AddTwoIntsResponsePacket encoded {};
        auto converter = hako::pdu::msgs::hako_srv_msgs::AddTwoIntsResponsePacket {};
        if (!converter.pdu2cpp(reinterpret_cast<char*>(packet.data()), encoded)) {
            error = "failed to initialize AddTwoInts response";
            return false;
        }
        encoded.body.sum = decode_value(response.payload);
        if (converter.cpp2pdu(
                encoded, reinterpret_cast<char*>(packet.data()),
                static_cast<int>(packet.size())) < 0) {
            error = "failed to encode AddTwoInts response";
            return false;
        }
        return true;
    };
    return std::make_shared<CoreAsyncRpcServiceChannel>(
        "A", kAsset, kService, std::move(decoder), std::move(encoder));
}

class Callbacks final : public IHakoniwaAssetDriverCallbacks {
public:
    explicit Callbacks(std::shared_ptr<CoreAsyncRpcServiceChannel> channel)
        : channel_(std::move(channel)) {}

    int on_initialize() noexcept override
    {
        std::string error;
        if (!channel_->prepare(error)) return fail("prepare failed: " + error);
        std::cout << "SERVER ready" << std::endl;
        return 0;
    }

    int on_step() noexcept override
    {
        ++steps_;
        AsyncRpcRequest request;
        std::string error;
        const auto result = channel_->poll(request, error);
        if (result == AsyncRpcPollResult::error) return fail("poll failed: " + error);
        if (result == AsyncRpcPollResult::request) {
            const auto b = decode_value(request.payload);
            std::cout << "SERVER request a=" << request.step << " b=" << b << std::endl;
            if (request.step == 0) {
                return reply(AsyncRpcResultCode::ok, b);
            }
            held_ = b;
        } else if (result == AsyncRpcPollResult::cancel) {
            if (request.channel != "A") return fail("cancel without the channel name");
            if (!held_) return fail("cancel while nothing is held");
            std::cout << "SERVER cancel b=" << *held_ << std::endl;
            const auto b = *held_;
            held_.reset();
            return reply(AsyncRpcResultCode::canceled, -b);
        }
        return 0;
    }

    int on_reset() noexcept override
    {
        held_.reset();
        return 0;
    }
    bool should_stop() const noexcept override { return stopped_; }
    int stop_result() const noexcept override { return error_.empty() ? 0 : -1; }

    int steps_ {0};

private:
    int reply(AsyncRpcResultCode code, std::int64_t sum) noexcept
    {
        AsyncRpcResponse response;
        response.result_code = code;
        response.payload = encode_value(sum);
        std::string error;
        if (!channel_->reply(response, error)) return fail("reply failed: " + error);
        std::cout << "SERVER replied code=" << code_name(code) << " sum=" << sum << std::endl;
        return 0;
    }

    int fail(std::string message) noexcept
    {
        if (error_.empty()) {
            error_ = std::move(message);
            std::cerr << "ERROR: " << error_ << std::endl;
        }
        stopped_ = true;
        return -1;
    }

    std::shared_ptr<CoreAsyncRpcServiceChannel> channel_;
    std::optional<std::int64_t> held_;
    bool stopped_ {false};
    std::string error_;
};
} // namespace

int main(int argc, char** argv)
{
    std::cout << std::unitbuf;
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <pdu_config.json> <service.json>" << std::endl;
        return 2;
    }
    Callbacks callbacks(make_channel());
    HakoniwaAssetDriver driver(
        HakoniwaAssetDriverConfig {kAsset, argv[1], 1000, HAKO_ASSET_MODEL_PLANT}, callbacks);
    if (driver.register_asset() != 0 || hako_asset_service_initialize(argv[2]) != 0) {
        std::cerr << "ERROR: server registration/service initialization failed" << std::endl;
        return 1;
    }
    std::cout << "SERVER registered" << std::endl;
    const int result = driver.start();
    std::cout << "SERVER exit=" << result << " steps=" << callbacks.steps_ << std::endl;
    return result == 0 ? 0 : 1;
}
