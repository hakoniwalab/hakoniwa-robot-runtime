#include "adapters/endpoint/ackermann_step_service.hpp"
#include "runner/hakoniwa/core_async_service_server.hpp"
#include "runner/hakoniwa/hakoniwa_asset_driver.hpp"
#include "runtime/actuator_runtime.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runtime/service/async_service_lockstep_gate.hpp"
#include "runtime/service/async_service_pipeline.hpp"

#include "hako_asset_service.h"

#include <iostream>
#include <stdexcept>

using namespace ::hakoniwa::robot_runtime::runtime;
using namespace ::hakoniwa::robot_runtime::adapters::hakoniwa;
using namespace ::hakoniwa::robot_runtime::runner::detail;
using ::hakoniwa::robot_runtime::runner::hakoniwa::CoreAsyncServiceServer;

namespace {

constexpr char kAsset[] = "AckermannRpcServer";
constexpr char kVehicle[] = "A";
constexpr hako_time_t kDelta = 1000;

class Plant final : public IActuatorPlant {
public:
    explicit Plant(std::shared_ptr<AckermannDriveRpcEventReader> reader)
        : reader_(std::move(reader)) {}

    std::uint64_t delta_time_usec() const noexcept override { return kDelta; }

    RobotState read_state() const override
    {
        RobotState result;
        result.sample_time_usec = time_;
        result.bodies.push_back({
            .body_id = kVehicle, .position = {static_cast<double>(time_ / kDelta) - 1.0, 0, 0},
            .linear_velocity = {speed_, 0, 0},
        });
        return result;
    }

    RobotState step(const std::vector<ActuatorCommand>&) override
    {
        const auto command = reader_->take_latest();
        if (!command) {
            throw std::runtime_error("expected one new Ackermann command");
        }
        speed_ = command->speed_m_s;
        time_ += kDelta;
        return read_state();
    }

    RobotState reset(std::uint64_t time) override
    {
        time_ = time;
        speed_ = 0;
        return read_state();
    }

private:
    std::shared_ptr<AckermannDriveRpcEventReader> reader_;
    std::uint64_t time_ {0};
    double speed_ {0};
};

class Callbacks final : public IHakoniwaAssetDriverCallbacks {
public:
    Callbacks()
    {
        const auto name = ackermann_step_service_name(kVehicle);
        server_ = std::make_shared<CoreAsyncServiceServer>(kAsset, std::vector {name});
        auto reader = std::make_shared<AckermannDriveRpcEventReader>();
        auto service = std::make_shared<AckermannStepService>(reader, kVehicle);
        auto source = std::make_shared<AsyncServiceCommandSource>(
            name, 0, server_, std::move(service), true);
        auto controller = std::make_shared<AsyncServiceController>("service", source);
        auto publisher = std::make_shared<AsyncServiceResponsePublisher>("reply", source);
        gate_ = std::make_unique<AsyncServiceLockstepGate>(std::vector {source});
        runtime_ = std::make_unique<ActuatorRuntime>(
            std::vector<std::shared_ptr<ICommandSource>> {source},
            std::vector<std::shared_ptr<IController>> {controller},
            std::make_shared<PriorityCommandArbiter>(std::vector<ControllerPriority> {}),
            std::make_shared<Plant>(reader),
            std::vector<std::shared_ptr<IStatePublisher>> {publisher});
    }

    int on_initialize() noexcept override
    {
        try {
            server_->prepare();
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
            if (report.publisher_statuses.at(0).state != ComponentState::Ready) {
                return fail(report.publisher_statuses.at(0).detail);
            }
            std::cout << "STEP count=" << ++steps_ << std::endl;
            return 0;
        } catch (const std::exception& error) {
            return fail(error.what());
        }
    }

    int on_reset() noexcept override { return 0; }
    bool should_stop() const noexcept override { return failed_; }
    int stop_result() const noexcept override { return failed_ ? -1 : 0; }

private:
    int fail(std::string_view reason)
    {
        std::cerr << "SERVER ERROR: " << reason << std::endl;
        failed_ = true;
        return -1;
    }

    std::shared_ptr<CoreAsyncServiceServer> server_;
    std::unique_ptr<AsyncServiceLockstepGate> gate_;
    std::unique_ptr<ActuatorRuntime> runtime_;
    std::size_t steps_ {0};
    bool failed_ {false};
};

} // namespace

int main(int argc, char** argv)
{
    if (argc != 3) return 2;
    Callbacks callbacks;
    HakoniwaAssetDriver driver({kAsset, argv[1], kDelta, HAKO_ASSET_MODEL_PLANT}, callbacks);
    if (driver.register_asset() != 0 || hako_asset_service_initialize(argv[2]) != 0) {
        return 1;
    }
    std::cout << "SERVER registered" << std::endl;
    return driver.start() == 0 && !callbacks.should_stop() ? 0 : 1;
}
