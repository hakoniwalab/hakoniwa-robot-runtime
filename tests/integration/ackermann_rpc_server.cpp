#include "hakoniwa/robot_runtime/adapters/endpoint/ackermann_drive_rpc_event_reader.hpp"
#include "hakoniwa/robot_runtime/runner/async_rpc_input.hpp"
#include "runner/hakoniwa/hakoniwa_asset_driver.hpp"

#include "hako_asset_service.h"

#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace hakoniwa::robot_runtime;

namespace {
constexpr const char* kAsset = "AckermannRpcServer";

class Callbacks final : public runner::detail::IHakoniwaAssetDriverCallbacks {
public:
    explicit Callbacks(std::shared_ptr<runner::AsyncRpcInput> input)
        : input_(std::move(input)) {}
    int on_initialize() noexcept override
    {
        if (!input_->prepare_channels()) {
            fail(input_->failure_reason());
            return -1;
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
        if (!input_->apply_ready_requests() || !input_->complete_step()) {
            fail(input_->failure_reason());
            return -1;
        }
        ++steps_;
        std::cout << "STEP count=" << steps_ << std::endl;
        return 0;
    }
    int on_reset() noexcept override { input_->lockstep_prepare(); return 0; }
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
    std::shared_ptr<runner::AsyncRpcInput> input_;
    std::size_t steps_ {0};
    bool stopped_ {false};
    std::string error_;
};
} // namespace

int main(int argc, char** argv)
{
    std::cout << std::unitbuf;
    if (argc != 3) return 2;
    auto reader_a = std::make_shared<adapters::hakoniwa::AckermannDriveRpcEventReader>();
    auto reader_b = std::make_shared<adapters::hakoniwa::AckermannDriveRpcEventReader>();
    const std::map<std::string,
        std::shared_ptr<adapters::hakoniwa::AckermannDriveRpcEventReader>> readers {
        {"A", reader_a}, {"B", reader_b},
    };
    std::map<std::string, double> speeds;
    auto channel_a = adapters::hakoniwa::make_ackermann_step_service_channel("A", kAsset);
    auto channel_b = adapters::hakoniwa::make_ackermann_step_service_channel("B", kAsset);
    auto input = std::make_shared<runner::AsyncRpcInput>(
        std::vector<std::shared_ptr<runner::IAsyncRpcChannel>> {channel_a, channel_b},
        runner::AsyncRpcInputConfig {std::chrono::seconds(5), std::chrono::seconds(30), {}},
        [&readers, &speeds](
            std::span<const runner::AsyncRpcRequest> requests, std::string& error) {
            for (const auto& request : requests) {
                runtime::AckermannDriveCommand command;
                if (!adapters::hakoniwa::decode_ackermann_command(
                        request.payload, command)) {
                    error = "invalid command for " + request.channel;
                    return false;
                }
                readers.at(request.channel)->apply(command);
                const auto applied = readers.at(request.channel)->take_latest();
                if (!applied) return false;
                speeds[request.channel] = applied->speed_m_s;
            }
            return true;
        },
        [&speeds](const runner::AsyncRpcRequest& request,
                  runner::AsyncRpcResponse& response, std::string&) {
            response.step = request.step;
            response.payload = adapters::hakoniwa::encode_ackermann_state({
                (request.step + 1) * 1000,
                {static_cast<double>(request.step), 0.0, 0.0},
                {0.0, 0.0, 0.0, 1.0},
                {speeds.at(request.channel), 0.0, 0.0},
                {},
            });
            return true;
        });
    Callbacks callbacks(input);
    runner::detail::HakoniwaAssetDriver driver(
        {kAsset, argv[1], 1000, HAKO_ASSET_MODEL_PLANT}, callbacks);
    if (driver.register_asset() != 0 || hako_asset_service_initialize(argv[2]) != 0) {
        return 1;
    }
    std::cout << "SERVER registered" << std::endl;
    const int result = driver.start();
    std::cout << "SERVER exit=" << result << " steps=" << callbacks.steps_ << std::endl;
    return result == 0 ? 0 : 1;
}
