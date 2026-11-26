#include "async_service_lockstep_fixture.hpp"
#include "runner/hakoniwa/core_async_service_server.hpp"
#include "runner/hakoniwa/hakoniwa_asset_driver.hpp"
#include "runtime/actuator_runtime.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runner/lockstep_gate.hpp"
#include "runtime/service/async_service_pipeline.hpp"
#include "support/single_service_robot.hpp"

#include "hako_asset_service.h"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsResponsePacket.hpp"

#include <array>
#include <chrono>
#include <exception>
#include <iostream>
#include <memory>

using namespace async_service_lockstep_fixture;
namespace test_support = hakoniwa::robot_runtime::test_support;
using namespace hakoniwa::robot_runtime::runtime;
using namespace hakoniwa::robot_runtime::runner::detail;
using hakoniwa::robot_runtime::runner::LockstepGate;
using hakoniwa::robot_runtime::runner::hakoniwa::CoreAsyncServiceServer;

namespace {

constexpr auto kAsset = "AsyncServiceLockstepServer";
constexpr std::array kServices {"Service/AsyncA", "Service/AsyncB"};
constexpr std::array kJoints {"joint-a", "joint-b"};

class Plant final : public IActuatorPlant {
public:
    PlantCapabilities capabilities() const noexcept override { return {}; }
    void apply_operations(const PlantOperations&) override {}

    std::uint64_t delta_time_usec() const noexcept override { return kDeltaUsec; }

    RobotState read_state() const override
    {
        RobotState state;
        state.sample_time_usec = time_usec_;
        for (std::size_t i = 0; i < kJoints.size(); ++i) {
            state.actuators.push_back({kJoints[i], positions_[i], 0, 0, time_usec_});
        }
        return state;
    }

    RobotState step(const std::vector<ActuatorCommand>& commands) override
    {
        for (const auto& command : commands) {
            for (std::size_t i = 0; i < kJoints.size(); ++i) {
                if (command.actuator_id == kJoints[i]) {
                    positions_[i] = command.value + kPositionIncrement;
                }
            }
        }
        time_usec_ += kDeltaUsec;
        return read_state();
    }

    RobotState reset(std::uint64_t time_usec) override
    {
        time_usec_ = time_usec;
        positions_.fill(0);
        return read_state();
    }

private:
    std::uint64_t time_usec_ {0};
    std::array<double, kJoints.size()> positions_ {};
};

class OneStepService final : public test_support::OneService {
public:
    explicit OneStepService(std::size_t channel) : channel_(channel) {}

    bool on_request(std::span<const std::byte> bytes) override
    {
        auto* packet = const_cast<char*>(reinterpret_cast<const char*>(bytes.data()));
        HakoCpp_AddTwoIntsRequestPacket request {};
        if (!hako::pdu::msgs::hako_srv_msgs::AddTwoIntsRequestPacket {}.pdu2cpp(packet, request)) {
            return false;
        }
        target_ = static_cast<double>(request.body.a);
        return true;
    }

    test_support::OneServiceStepResult on_step(
        const RobotState&, AsyncServiceState, const RuntimeStepContext& context) override
    {
        return {
            AsyncServiceStepOutcome::Ok,
            {{kJoints[channel_], ActuatorCommandType::Position, target_,
                {}, context.simulation_time_usec, {}}},
        };
    }

    bool on_response(
        const RobotState& state, const RuntimeStepContext& context,
        std::span<std::byte> bytes) override
    {
        if (state.sample_time_usec != context.simulation_time_usec) {
            return false;
        }
        auto* packet = reinterpret_cast<char*>(bytes.data());
        HakoCpp_AddTwoIntsResponsePacket response {};
        auto converter = hako::pdu::msgs::hako_srv_msgs::AddTwoIntsResponsePacket {};
        if (!converter.pdu2cpp(packet, response)) {
            return false;
        }
        // Reuse the AddTwoInts wire schema. Pack two exact integers in its sum field.
        const auto position = static_cast<std::int64_t>(state.actuators.at(channel_).position_rad);
        response.body.sum = position * kTimeScale + hako_asset_simulation_time();
        return converter.cpp2pdu(response, packet, static_cast<int>(bytes.size())) >= 0;
    }

    void reset() override {}

private:
    std::size_t channel_;
    double target_ {0};
};

class Callbacks final : public IHakoniwaAssetDriverCallbacks {
public:
    int on_initialize() noexcept override
    {
        try {
            auto server = std::make_shared<CoreAsyncServiceServer>();
            std::vector<LockstepGate::Input> gate_inputs;
            std::vector<std::shared_ptr<ICommandSource>> sources;
            std::vector<std::shared_ptr<IController>> controllers;
            std::vector<std::shared_ptr<IStatePublisher>> publishers;
            std::vector<ControllerPriority> priorities;
            for (std::size_t i = 0; i < kServices.size(); ++i) {
                const auto* name = kServices[i];
                const int id = hako_asset_service_server_create(kAsset, name);
                if (id < 0) {
                    return fail("service server creation failed");
                }
                const std::string controller_id = std::string(name) + "/controller";
                auto robot = std::make_shared<test_support::SingleServiceRobot>(std::make_shared<OneStepService>(i));
                auto service_source = std::make_shared<AsyncServiceCommandSource>(
                    std::string(name) + "/0", id, server, robot, 0, true);
                auto source = std::make_shared<RobotServiceSource>(name, std::vector {service_source});
                auto controller = std::make_shared<RobotServiceController>(controller_id, source, robot);
                gate_inputs.push_back({name, service_source});
                sources.push_back(source);
                controllers.push_back(controller);
                publishers.push_back(std::make_shared<AsyncServiceResponsePublisher>(
                    std::string(name) + "/publisher", service_source));
                priorities.push_back({controller_id, 10, "lockstep-joints"});
            }
            LockstepGate::Config config;
            config.timeout = std::chrono::seconds(3);
            config.initial_timeout = std::chrono::seconds(10);
            gate_ = std::make_unique<LockstepGate>(
                std::move(gate_inputs), config);
            runtime_ = std::make_unique<ActuatorRuntime>(
                std::move(sources), std::move(controllers),
                std::make_shared<PriorityCommandArbiter>(std::move(priorities)),
                std::make_shared<Plant>(), std::move(publishers));
            gate_->prepare();
            (void)runtime_->start();
            return 0;
        } catch (const std::exception& error) {
            return fail(error.what());
        }
    }

    bool sleep_before_first_step() const noexcept override { return true; }

    bool ready_to_step() noexcept override
    {
        const bool ready = gate_->ready_to_step();
        if (gate_->failed()) {
            (void)fail(gate_->failure_reason());
        }
        return ready;
    }

    int on_step() noexcept override
    {
        try {
            const auto report = runtime_->step_hakoniwa(1);
            for (const auto* statuses : {
                     &report.source_statuses, &report.controller_statuses,
                     &report.publisher_statuses}) {
                for (const auto& status : *statuses) {
                    if (status.state == ComponentState::Error) {
                        return fail(status.detail);
                    }
                }
            }
            ++steps_;
            std::cout << "SERVER step=" << steps_ << std::endl;
            return 0;
        } catch (const std::exception& error) {
            return fail(error.what());
        }
    }

    int on_reset() noexcept override { return 0; }
    bool should_stop() const noexcept override { return failed_; }
    int stop_result() const noexcept override { return failed_ ? -1 : 0; }
    unsigned steps() const noexcept { return steps_; }

    void stop_runtime()
    {
        if (runtime_) {
            (void)runtime_->stop();
        }
    }

private:
    int fail(std::string_view message) noexcept
    {
        std::cerr << "SERVER ERROR: " << message << std::endl;
        failed_ = true;
        return -1;
    }

    std::unique_ptr<ActuatorRuntime> runtime_;
    std::unique_ptr<LockstepGate> gate_;
    unsigned steps_ {0};
    bool failed_ {false};
};

} // namespace

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <pdu_config.json> <service.json>\n";
        return 2;
    }
    Callbacks callbacks;
    HakoniwaAssetDriver driver(
        {kAsset, argv[1], kDeltaUsec, HAKO_ASSET_MODEL_PLANT, true}, callbacks);
    if (driver.register_asset() != 0 || hako_asset_service_initialize(argv[2]) != 0) {
        return 1;
    }
    std::cout << "SERVER registered" << std::endl;
    const int result = driver.start();
    callbacks.stop_runtime();
    const int exit_code = result == 0 && !callbacks.should_stop() ? 0 : 1;
    std::cout << "SERVER exit=" << exit_code << " steps=" << callbacks.steps() << std::endl;
    return exit_code;
}
