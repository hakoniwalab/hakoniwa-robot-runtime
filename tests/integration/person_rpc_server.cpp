#include "hakoniwa/robot_runtime/adapters/endpoint/person_step_rpc_event_reader.hpp"
#include "hakoniwa/robot_runtime/runner/async_rpc_input.hpp"
#include "runner/hakoniwa/hakoniwa_asset_driver.hpp"

#include "hako_asset_service.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace hakoniwa::robot_runtime;

namespace {
constexpr const char* kAsset = "PersonRpcServer";
constexpr double dt = 0.001;

class Callbacks final : public runner::detail::IHakoniwaAssetDriverCallbacks {
public:
    explicit Callbacks(std::shared_ptr<runner::AsyncRpcInput> input)
        : input_(std::move(input)) {}
    int on_initialize() noexcept override
    {
        if (!input_->prepare_channels()) return -1;
        input_->lockstep_prepare();
        std::cout << "SERVER ready channels=A,B" << std::endl;
        return 0;
    }
    bool ready_to_step() noexcept override
    {
        return input_->lockstep_ready();
    }
    int on_step() noexcept override
    {
        if (!input_->apply_ready_requests() || !input_->complete_step()) {
            std::cerr << "ERROR: " << input_->failure_reason() << std::endl;
            return -1;
        }
        std::cout << "STEP count=" << ++steps_ << std::endl;
        return 0;
    }
    int on_reset() noexcept override { input_->lockstep_prepare(); return 0; }
    bool should_stop() const noexcept override { return false; }
    int stop_result() const noexcept override { return 0; }
    std::shared_ptr<runner::AsyncRpcInput> input_;
    std::size_t steps_ {0};
};

struct Pose {
    double x {0.0};
    double y {0.0};
    double yaw {0.0};
    adapters::hakoniwa::PersonStepCommand command;
};
} // namespace

int main(int argc, char** argv)
{
    std::cout << std::unitbuf;
    if (argc != 3) return 2;
    std::map<std::string, Pose> poses {{"A", {}}, {"B", {}}};
    auto input = std::make_shared<runner::AsyncRpcInput>(
        std::vector<std::shared_ptr<runner::IAsyncRpcChannel>> {
            adapters::hakoniwa::make_person_step_service_channel("A", kAsset),
            adapters::hakoniwa::make_person_step_service_channel("B", kAsset),
        },
        runner::AsyncRpcInputConfig {
            std::chrono::seconds(5), std::chrono::seconds(30), {}},
        [&poses](std::span<const runner::AsyncRpcRequest> requests,
                 std::string& error) {
            for (const auto& request : requests) {
                adapters::hakoniwa::PersonStepCommand command;
                if (!adapters::hakoniwa::decode_person_command(
                        request.payload, command)) {
                    error = "invalid command for " + request.channel;
                    return false;
                }
                auto& pose = poses.at(request.channel);
                pose.command = command;
                const double c = std::cos(pose.yaw);
                const double s = std::sin(pose.yaw);
                pose.x += dt * (c * command.linear_velocity.x
                    - s * command.linear_velocity.y);
                pose.y += dt * (s * command.linear_velocity.x
                    + c * command.linear_velocity.y);
                pose.yaw += dt * command.angular_velocity.z;
            }
            return true;
        },
        [&poses](const runner::AsyncRpcRequest& request,
                 runner::AsyncRpcResponse& response, std::string&) {
            const auto& pose = poses.at(request.channel);
            adapters::hakoniwa::PersonStepState state;
            state.simulation_time_usec = (request.step + 1) * 1000;
            state.position = {pose.x, pose.y, 0.0};
            state.orientation = {0.0, 0.0, std::sin(pose.yaw / 2.0),
                std::cos(pose.yaw / 2.0)};
            state.linear_velocity = {
                pose.command.linear_velocity.x,
                pose.command.linear_velocity.y, 0.0};
            state.angular_velocity.z = pose.command.angular_velocity.z;
            if (std::hypot(pose.command.linear_velocity.x,
                    pose.command.linear_velocity.y) > 0.05) {
                state.joint_positions[2] = 0.1;
                state.joint_positions[3] = -0.1;
            }
            response.step = request.step;
            response.payload = adapters::hakoniwa::encode_person_state(state);
            return true;
        });
    Callbacks callbacks(input);
    runner::detail::HakoniwaAssetDriver driver(
        {kAsset, argv[1], 1000, HAKO_ASSET_MODEL_PLANT}, callbacks);
    if (driver.register_asset() != 0
        || hako_asset_service_initialize(argv[2]) != 0) return 1;
    std::cout << "SERVER registered" << std::endl;
    return driver.start() == 0 ? 0 : 1;
}
