#pragma once

#include "adapters/endpoint/ackermann_step_service.hpp"
#include "runner/hakoniwa/core_async_service_server.hpp"
#include "runtime/actuator_runtime.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runner/lockstep_gate.hpp"
#include "runtime/service/async_service_pipeline.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>

namespace ackermann_step_fixture {

using namespace ::hakoniwa::robot_runtime::runtime;
using ::hakoniwa::robot_runtime::runner::LockstepGate;
using namespace ::hakoniwa::robot_runtime::adapters::hakoniwa;
using ::hakoniwa::robot_runtime::runner::hakoniwa::CoreAsyncServiceServer;


constexpr char kAsset[] = "AckermannRpcServer";
constexpr std::uint64_t kDelta = 1000;

class Plant final : public IActuatorPlant {
public:
    struct Vehicle {
        std::string name;
        std::shared_ptr<AckermannDriveRpcEventReader> reader;
        double speed {0};
    };

    explicit Plant(std::vector<Vehicle> vehicles) : vehicles_(std::move(vehicles)) {}

    std::uint64_t delta_time_usec() const noexcept override { return kDelta; }

    RobotState read_state() const override
    {
        RobotState result;
        result.sample_time_usec = time_;
        for (const auto& vehicle : vehicles_) {
            result.bodies.push_back({
                .body_id = vehicle.name,
                .position = {static_cast<double>(time_ / kDelta) - 1.0, 0, 0},
                .linear_velocity = {vehicle.speed, 0, 0},
            });
        }
        return result;
    }

    RobotState step(const std::vector<ActuatorCommand>&) override
    {
        for (auto& vehicle : vehicles_) {
            const auto command = vehicle.reader->take_latest();
            if (!command) {
                throw std::runtime_error("expected a new Ackermann command for " + vehicle.name);
            }
            vehicle.speed = command->speed_m_s;
        }
        time_ += kDelta;
        return read_state();
    }

    RobotState reset(std::uint64_t time) override
    {
        time_ = time;
        for (auto& vehicle : vehicles_) {
            vehicle.speed = 0;
        }
        return read_state();
    }

private:
    std::vector<Vehicle> vehicles_;
    std::uint64_t time_ {0};
};

class RuntimeFixture {
public:
    explicit RuntimeFixture(
        const std::vector<std::string>& vehicle_names,
        std::shared_ptr<::hakoniwa::robot_runtime::runner::ILockstepInput> step_permission = {})
    {
        std::vector<std::string> service_names;
        for (const auto& vehicle : vehicle_names) {
            service_names.push_back(ackermann_step_service_name(vehicle));
        }
        server_ = std::make_shared<CoreAsyncServiceServer>(kAsset, service_names);
        std::vector<Plant::Vehicle> vehicles;
        std::vector<LockstepGate::Input> gate_inputs;
        std::vector<std::shared_ptr<ICommandSource>> sources;
        std::vector<std::shared_ptr<IController>> controllers;
        std::vector<std::shared_ptr<IStatePublisher>> publishers;
        for (std::size_t index = 0; index < vehicle_names.size(); ++index) {
            const auto& vehicle = vehicle_names[index];
            const auto& name = service_names[index];
            auto reader = std::make_shared<AckermannDriveRpcEventReader>();
            auto service = std::make_shared<AckermannStepService>(reader, vehicle);
            auto service_source = std::make_shared<AsyncServiceCommandSource>(
                name + "/0", static_cast<int>(index), server_, service, 0, true);
            auto source = std::make_shared<RobotServiceSource>(name, std::vector {service_source});
            vehicles.push_back({vehicle, std::move(reader)});
            gate_inputs.push_back({name, service_source});
            sources.push_back(source);
            controllers.push_back(std::make_shared<RobotServiceController>(name + ":controller", source, service));
            publishers.push_back(std::make_shared<AsyncServiceResponsePublisher>(name + ":reply", service_source));
        }
        if (step_permission) {
            gate_inputs.push_back({"test step permission", std::move(step_permission)});
        }
        LockstepGate::Config config;
        config.timeout = std::chrono::seconds(3);
        gate_ = std::make_unique<LockstepGate>(std::move(gate_inputs), config);
        runtime_ = std::make_unique<ActuatorRuntime>(
            std::move(sources), std::move(controllers),
            std::make_shared<PriorityCommandArbiter>(std::vector<ControllerPriority> {}),
            std::make_shared<Plant>(std::move(vehicles)), std::move(publishers));
    }

    int on_initialize() noexcept
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

    bool ready_to_step() noexcept
    {
        const bool ready = gate_->ready_to_step();
        if (gate_->failed()) {
            (void)fail(gate_->failure_reason());
        }
        return ready;
    }

    int on_step() noexcept
    {
        try {
            const auto report = runtime_->step_hakoniwa(1);
            for (const auto& status : report.publisher_statuses) {
                if (status.state != ComponentState::Ready) {
                    return fail(status.component_id + ": " + status.detail);
                }
            }
            std::cout << "STEP count=" << ++steps_ << std::endl;
            return 0;
        } catch (const std::exception& error) {
            return fail(error.what());
        }
    }

    bool failed() const noexcept { return failed_; }
    auto service_state(int service_id) noexcept { return server_->status(service_id); }

private:
    int fail(std::string_view reason)
    {
        std::cerr << "SERVER ERROR: " << reason << std::endl;
        failed_ = true;
        return -1;
    }

    std::shared_ptr<CoreAsyncServiceServer> server_;
    std::unique_ptr<LockstepGate> gate_;
    std::unique_ptr<ActuatorRuntime> runtime_;
    std::size_t steps_ {0};
    bool failed_ {false};
};

} // namespace ackermann_step_fixture
