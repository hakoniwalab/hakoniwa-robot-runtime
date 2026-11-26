#include "runner/hakoniwa/core_async_service_server.hpp"
#include "runner/hakoniwa/hakoniwa_asset_driver.hpp"
#include "runtime/actuator_runtime.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runtime/service/async_service_pipeline.hpp"

#include "hako_asset_service.h"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsResponsePacket.hpp"

#include <exception>
#include <iostream>
#include <memory>

using namespace hakoniwa::robot_runtime::runtime;
using namespace hakoniwa::robot_runtime::runner::detail;
using hakoniwa::robot_runtime::runner::hakoniwa::CoreAsyncServiceServer;

namespace {

constexpr auto kAsset = "AsyncServiceReplyServer";
constexpr auto kService = "Service/AsyncA";
constexpr auto kSource = "sum-source";
constexpr auto kController = "sum-controller";
constexpr auto kJoint = "joint";
constexpr std::uint64_t kDeltaUsec = 1'000;

class Plant final : public IActuatorPlant {
public:
    std::uint64_t delta_time_usec() const noexcept override { return kDeltaUsec; }

    RobotState read_state() const override
    {
        RobotState state;
        state.sample_time_usec = time_usec_;
        state.actuators.push_back({kJoint, position_, 0, 0, time_usec_});
        return state;
    }

    RobotState step(const std::vector<ActuatorCommand>& commands) override
    {
        if (!commands.empty()) {
            position_ = commands.front().value + 1;
        }
        time_usec_ += kDeltaUsec;
        return read_state();
    }

    RobotState reset(std::uint64_t time_usec) override
    {
        time_usec_ = time_usec;
        position_ = 0;
        return read_state();
    }

private:
    std::uint64_t time_usec_ {0};
    double position_ {0};
};

class SumService final : public IAsyncService {
public:
    explicit SumService(int required_steps) : required_steps_(required_steps) {}

    bool on_request(std::span<const std::byte> bytes) override
    {
        auto* packet = const_cast<char*>(reinterpret_cast<const char*>(bytes.data()));
        HakoCpp_AddTwoIntsRequestPacket request {};
        if (!hako::pdu::msgs::hako_srv_msgs::AddTwoIntsRequestPacket {}.pdu2cpp(packet, request)
            || request.body.a < 0) {
            return false;
        }
        target_ = request.body.a;
        addend_ = request.body.b;
        doing_steps_ = 0;
        canceling_steps_ = 0;
        return true;
    }

    AsyncServiceStepResult on_step(
        const RobotState&, AsyncServiceState state, const RuntimeStepContext& context) override
    {
        if (doing_steps_ == 0) {
            first_step_usec_ = context.simulation_time_usec;
        }
        auto outcome = AsyncServiceStepOutcome::Pending;
        std::int64_t position;
        if (state == AsyncServiceState::Canceling) {
            position = target_ - ++canceling_steps_;
            if (canceling_steps_ == 2) {
                outcome = AsyncServiceStepOutcome::Canceled;
            }
        } else {
            position = target_ + doing_steps_++;
            if (doing_steps_ == required_steps_) {
                outcome = AsyncServiceStepOutcome::Ok;
            }
        }
        return {
            outcome,
            {{kJoint, ActuatorCommandType::Position, static_cast<double>(position),
                {}, context.simulation_time_usec, {}}},
        };
    }

    bool on_response(
        const RobotState& state, const RuntimeStepContext& context,
        std::span<std::byte> bytes) override
    {
        const auto elapsed_steps = static_cast<std::uint64_t>(doing_steps_ + canceling_steps_);
        if (state.sample_time_usec != context.simulation_time_usec
            || state.sample_time_usec != first_step_usec_ + elapsed_steps * kDeltaUsec) {
            return false;
        }
        auto* packet = reinterpret_cast<char*>(bytes.data());
        HakoCpp_AddTwoIntsResponsePacket response {};
        auto converter = hako::pdu::msgs::hako_srv_msgs::AddTwoIntsResponsePacket {};
        if (!converter.pdu2cpp(packet, response)) {
            return false;
        }
        response.body.sum = static_cast<std::int64_t>(state.actuators.at(0).position_rad) + addend_;
        std::cout << "SERVER response doing=" << doing_steps_ << " canceling=" << canceling_steps_
                  << " elapsed_usec=" << elapsed_steps * kDeltaUsec << std::endl;
        return converter.cpp2pdu(response, packet, static_cast<int>(bytes.size())) >= 0;
    }

    void reset() override {}

private:
    int required_steps_;
    int doing_steps_ {0};
    int canceling_steps_ {0};
    std::uint64_t first_step_usec_ {0};
    std::int64_t target_ {0};
    std::int64_t addend_ {0};
};

class Callbacks final : public IHakoniwaAssetDriverCallbacks {
public:
    explicit Callbacks(int required_steps) : required_steps_(required_steps) {}

    int on_initialize() noexcept override
    {
        const int id = hako_asset_service_server_create(kAsset, kService);
        if (id < 0) {
            return fail("service server creation failed");
        }
        try {
            auto server = std::make_shared<CoreAsyncServiceServer>();
            auto source = std::make_shared<AsyncServiceCommandSource>(
                kSource, id, server, std::make_shared<SumService>(required_steps_));
            auto controller = std::make_shared<AsyncServiceController>(kController, source);
            runtime_ = std::make_unique<ActuatorRuntime>(
                std::vector<std::shared_ptr<ICommandSource>> {source},
                std::vector<std::shared_ptr<IController>> {controller},
                std::make_shared<PriorityCommandArbiter>(
                    std::vector<ControllerPriority> {{kController, 10, {}}}),
                std::make_shared<Plant>(),
                std::vector<std::shared_ptr<IStatePublisher>> {
                    std::make_shared<AsyncServiceResponsePublisher>("sum-response", source)});
            return 0;
        } catch (const std::exception& error) {
            return fail(error.what());
        }
    }

    int on_step() noexcept override
    {
        try {
            const auto report = runtime_->step_hakoniwa(1);
            for (const auto& status : report.source_statuses) {
                if (status.state == ComponentState::Error) {
                    return fail(status.detail);
                }
            }
            for (const auto& status : report.controller_statuses) {
                if (status.state == ComponentState::Error) {
                    return fail(status.detail);
                }
            }
            const auto& status = report.publisher_statuses.front();
            return status.state == ComponentState::Error ? fail(status.detail) : 0;
        } catch (const std::exception& error) {
            return fail(error.what());
        }
    }

    int on_reset() noexcept override { return 0; }
    bool should_stop() const noexcept override { return failed_; }
    int stop_result() const noexcept override { return failed_ ? -1 : 0; }

private:
    int fail(std::string_view message) noexcept
    {
        std::cerr << "SERVER ERROR: " << message << std::endl;
        failed_ = true;
        return -1;
    }

    std::unique_ptr<ActuatorRuntime> runtime_;
    int required_steps_;
    bool failed_ {false};
};

} // namespace

int main(int argc, char** argv)
{
    if (argc != 4 || (std::string_view(argv[3]) != "1" && std::string_view(argv[3]) != "5")) {
        std::cerr << "Usage: " << argv[0] << " <pdu_config.json> <service.json> <1|5 steps>\n";
        return 2;
    }
    Callbacks callbacks(argv[3][0] == '1' ? 1 : 5);
    HakoniwaAssetDriver driver(
        HakoniwaAssetDriverConfig {kAsset, argv[1], kDeltaUsec, HAKO_ASSET_MODEL_PLANT, true}, callbacks);
    if (driver.register_asset() != 0 || hako_asset_service_initialize(argv[2]) != 0) {
        return 1;
    }
    std::cout << "SERVER registered" << std::endl;
    return driver.start() == 0 ? 0 : 1;
}
