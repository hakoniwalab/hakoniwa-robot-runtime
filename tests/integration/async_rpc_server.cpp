#include "hakoniwa/robot_runtime/runner/async_rpc_input.hpp"
#include "hakoniwa/robot_runtime/runner/hakoniwa/async_rpc_service_channel.hpp"
#include "runner/hakoniwa/hakoniwa_asset_driver.hpp"

#include "hako_asset_service.h"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsResponsePacket.hpp"

#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

using hakoniwa::robot_runtime::runner::AsyncRpcInput;
using hakoniwa::robot_runtime::runner::AsyncRpcInputConfig;
using hakoniwa::robot_runtime::runner::AsyncRpcRequest;
using hakoniwa::robot_runtime::runner::AsyncRpcResponse;
using hakoniwa::robot_runtime::runner::IAsyncRpcChannel;
using hakoniwa::robot_runtime::runner::detail::HakoniwaAssetDriver;
using hakoniwa::robot_runtime::runner::detail::HakoniwaAssetDriverConfig;
using hakoniwa::robot_runtime::runner::detail::IHakoniwaAssetDriverCallbacks;
using hakoniwa::robot_runtime::runner::hakoniwa::CoreAsyncRpcServiceChannel;

namespace {
constexpr const char* kAsset = "AsyncRpcServer";
constexpr const char* kServiceA = "Service/AsyncA";
constexpr const char* kServiceB = "Service/AsyncB";
constexpr std::int64_t kStepPackingBase = 1'000'000;

std::vector<std::byte> encode_value(const std::int64_t value)
{
    std::vector<std::byte> result(sizeof(value));
    std::memcpy(result.data(), &value, sizeof(value));
    return result;
}

bool decode_value(std::span<const std::byte> payload, std::int64_t& value)
{
    if (payload.size() != sizeof(value)) return false;
    std::memcpy(&value, payload.data(), sizeof(value));
    return true;
}

std::shared_ptr<CoreAsyncRpcServiceChannel> make_channel(
    std::string channel,
    const char* service)
{
    auto decoder = [](std::span<const std::byte> packet, AsyncRpcRequest& request,
                       std::string& error) {
        HakoCpp_AddTwoIntsRequestPacket decoded {};
        if (!hako::pdu::msgs::hako_srv_msgs::AddTwoIntsRequestPacket {}.pdu2cpp(
                const_cast<char*>(reinterpret_cast<const char*>(packet.data())), decoded)) {
            error = "failed to decode AddTwoInts request";
            return false;
        }
        if (decoded.body.a < 0) {
            error = "negative step in AddTwoInts request";
            return false;
        }
        request.step = static_cast<std::uint64_t>(decoded.body.a);
        request.payload = encode_value(decoded.body.b);
        return true;
    };
    auto encoder = [](const AsyncRpcResponse& response, std::span<std::byte> packet,
                       std::string& error) {
        std::int64_t value = 0;
        if (!decode_value(response.payload, value)) {
            error = "invalid response value payload";
            return false;
        }
        HakoCpp_AddTwoIntsResponsePacket encoded {};
        auto converter = hako::pdu::msgs::hako_srv_msgs::AddTwoIntsResponsePacket {};
        if (!converter.pdu2cpp(reinterpret_cast<char*>(packet.data()), encoded)) {
            error = "failed to initialize AddTwoInts response";
            return false;
        }
        encoded.body.sum = static_cast<std::int64_t>(response.step) * kStepPackingBase + value;
        if (converter.cpp2pdu(
                encoded, reinterpret_cast<char*>(packet.data()),
                static_cast<int>(packet.size())) < 0) {
            error = "failed to encode AddTwoInts response";
            return false;
        }
        return true;
    };
    return std::make_shared<CoreAsyncRpcServiceChannel>(
        std::move(channel), kAsset, service, std::move(decoder), std::move(encoder));
}

class ServerCallbacks final : public IHakoniwaAssetDriverCallbacks {
public:
    ServerCallbacks(
        std::shared_ptr<AsyncRpcInput> input,
        std::vector<std::shared_ptr<CoreAsyncRpcServiceChannel>> channels,
        std::shared_ptr<std::map<std::string, std::int64_t>> values)
        : input_(std::move(input)), channels_(std::move(channels)), values_(std::move(values)) {}

    int on_initialize() noexcept override
    {
        for (const auto& channel : channels_) {
            std::string error;
            if (!channel->prepare(error)) {
                fail("service preparation failed for " + channel->name() + ": " + error);
                return -1;
            }
        }
        input_->lockstep_prepare();
        std::cout << "SERVER ready channels=A,B" << std::endl;
        return 0;
    }

    bool ready_to_step() noexcept override
    {
        if (input_->lockstep_ready()) return true;
        if (input_->failed()) fail(input_->failure_reason());
        return false;
    }

    int on_step() noexcept override
    {
        if (!input_->apply_ready_requests()) {
            fail(input_->failure_reason());
            return -1;
        }
        ++steps_;
        if (!input_->complete_step()) {
            fail(input_->failure_reason());
            return -1;
        }
        std::cout << "STEP count=" << steps_ << " completed_k=" << (steps_ - 1)
                  << " A=" << values_->at("A") << " B=" << values_->at("B") << std::endl;
        return 0;
    }

    int on_reset() noexcept override
    {
        steps_ = 0;
        values_->clear();
        input_->lockstep_prepare();
        return 0;
    }

    bool should_stop() const noexcept override { return stopped_; }
    int stop_result() const noexcept override { return error_.empty() ? 0 : -1; }

    void fail(std::string message) noexcept
    {
        if (error_.empty()) {
            error_ = std::move(message);
            std::cerr << "ERROR: " << error_ << std::endl;
        }
        stopped_ = true;
    }

    std::shared_ptr<AsyncRpcInput> input_;
    std::vector<std::shared_ptr<CoreAsyncRpcServiceChannel>> channels_;
    std::shared_ptr<std::map<std::string, std::int64_t>> values_;
    std::size_t steps_ {0};
    bool stopped_ {false};
    std::string error_;
};
} // namespace

int main(int argc, char** argv)
{
    std::cout << std::unitbuf;
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <pdu_config.json> <service.json> <step-log.jsonl>" << std::endl;
        return 2;
    }

    auto channel_a = make_channel("A", kServiceA);
    auto channel_b = make_channel("B", kServiceB);
    auto values = std::make_shared<std::map<std::string, std::int64_t>>();
    auto input = std::make_shared<AsyncRpcInput>(
        std::vector<std::shared_ptr<IAsyncRpcChannel>> {channel_a, channel_b},
        AsyncRpcInputConfig {
            std::chrono::seconds(5), std::chrono::seconds(30), argv[3]},
        [values](std::span<const AsyncRpcRequest> requests, std::string& error) {
            for (const auto& request : requests) {
                std::int64_t value = 0;
                if (!decode_value(request.payload, value)) {
                    error = "invalid request value payload for " + request.channel;
                    return false;
                }
                (*values)[request.channel] = value;
            }
            return true;
        },
        [](const AsyncRpcRequest& request, AsyncRpcResponse& response, std::string&) {
            response.step = request.step;
            response.payload = request.payload;
            return true;
        });

    ServerCallbacks callbacks(input, {channel_a, channel_b}, values);
    HakoniwaAssetDriver driver(
        HakoniwaAssetDriverConfig {kAsset, argv[1], 1000, HAKO_ASSET_MODEL_PLANT},
        callbacks);
    if (driver.register_asset() != 0 || hako_asset_service_initialize(argv[2]) != 0) {
        std::cerr << "ERROR: server registration/service initialization failed" << std::endl;
        return 1;
    }
    const int result = driver.start();
    std::cout << "SERVER exit=" << result << " steps=" << callbacks.steps_ << std::endl;
    return result == 0 ? 0 : 1;
}
