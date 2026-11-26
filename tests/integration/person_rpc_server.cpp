#include "hakoniwa/robot_runtime/adapters/endpoint/person_step_rpc_event_reader.hpp"
#include "runner/hakoniwa/hakoniwa_asset_driver.hpp"

#include "adapters/endpoint/person_step_service.hpp"
#include "runner/hakoniwa/core_async_service_server.hpp"
#include "runtime/actuator_runtime.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runner/lockstep_gate.hpp"
#include "runtime/service/async_service_pipeline.hpp"

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

struct Pose {
    double x {0.0};
    double y {0.0};
    double yaw {0.0};
    adapters::hakoniwa::PersonStepCommand command;
};
void apply(Pose& pose, const adapters::hakoniwa::PersonStepCommand& command)
{
    pose.command = command;
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    pose.x += dt * (c * command.linear_velocity.x - s * command.linear_velocity.y);
    pose.y += dt * (s * command.linear_velocity.x + c * command.linear_velocity.y);
    pose.yaw += dt * command.angular_velocity.z;
}

adapters::hakoniwa::PersonStepState read(const Pose& pose, std::uint64_t time)
{
    adapters::hakoniwa::PersonStepState state;
    state.simulation_time_usec = time;
    state.position = {pose.x, pose.y, 0.0};
    state.orientation = {0.0, 0.0, std::sin(pose.yaw / 2.0), std::cos(pose.yaw / 2.0)};
    state.linear_velocity = {
        pose.command.linear_velocity.x, pose.command.linear_velocity.y, 0.0};
    state.angular_velocity.z = pose.command.angular_velocity.z;
    if (std::hypot(pose.command.linear_velocity.x, pose.command.linear_velocity.y) > 0.05) {
        state.joint_positions[2] = 0.1;
        state.joint_positions[3] = -0.1;
    }
    return state;
}

int run(const char* pdu_config, const char* service_config,
    runner::detail::IHakoniwaAssetDriverCallbacks& callbacks)
{
    runner::detail::HakoniwaAssetDriver driver(
        {kAsset, pdu_config, 1000, HAKO_ASSET_MODEL_PLANT}, callbacks);
    if (driver.register_asset() != 0
        || hako_asset_service_initialize(service_config) != 0) return 1;
    std::cout << "SERVER registered" << std::endl;
    return driver.start() == 0 ? 0 : 1;
}

using namespace runtime;

class Plant final : public IActuatorPlant {
public:
    struct Person {
        Pose pose;
        adapters::hakoniwa::PersonStepCommand command;
    };
    std::map<std::string, Person> people {{"A", {}}, {"B", {}}};

    std::uint64_t delta_time_usec() const noexcept override { return 1000; }
    RobotState read_state() const override { return {.sample_time_usec = time_}; }
    RobotState step(const std::vector<ActuatorCommand>&) override
    {
        for (auto& [name, person] : people) {
            apply(person.pose, person.command);
        }
        time_ += delta_time_usec();
        return read_state();
    }
    RobotState reset(std::uint64_t time) override
    {
        time_ = time;
        for (auto& [name, person] : people) {
            person = {};
        }
        return read_state();
    }
private:
    std::uint64_t time_ {0};
};

class ServiceCallbacks final : public runner::detail::IHakoniwaAssetDriverCallbacks {
public:
    ServiceCallbacks()
    {
        const auto plant = std::make_shared<Plant>();
        const std::vector<std::string> names {
            adapters::hakoniwa::person_step_service_name("A"),
            adapters::hakoniwa::person_step_service_name("B"),
        };
        server_ = std::make_shared<runner::hakoniwa::CoreAsyncServiceServer>(kAsset, names);
        std::vector<runner::LockstepGate::Input> gate_inputs;
        std::vector<std::shared_ptr<ICommandSource>> sources;
        std::vector<std::shared_ptr<IController>> controllers;
        std::vector<std::shared_ptr<IStatePublisher>> publishers;
        int id = 0;
        for (const auto& [person, input] : plant->people) {
            const auto& name = names.at(id);
            auto service = std::make_shared<adapters::hakoniwa::PersonStepService>(person,
                [plant, person](const auto& command) {
                    plant->people.at(person).command = command;
                    return 0;
                },
                [plant, person](std::uint32_t) {
                    return read(plant->people.at(person).pose, plant->read_state().sample_time_usec);
                });
            auto source = std::make_shared<AsyncServiceCommandSource>(
                name, id++, server_, std::move(service), true);
            gate_inputs.push_back({name, source});
            sources.push_back(source);
            controllers.push_back(std::make_shared<AsyncServiceController>(name + ":controller", source));
            publishers.push_back(std::make_shared<AsyncServiceResponsePublisher>(name + ":response", source));
        }
        gate_ = std::make_unique<runner::LockstepGate>(
            std::move(gate_inputs), runner::LockstepGate::Config {});
        runtime_ = std::make_unique<ActuatorRuntime>(
            std::move(sources), std::move(controllers),
            std::make_shared<PriorityCommandArbiter>(std::vector<ControllerPriority> {}),
            plant, std::move(publishers));
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
    bool ready_to_step() noexcept override
    {
        const auto ready = gate_->ready_to_step();
        if (gate_->failed()) {
            (void)fail(gate_->failure_reason());
        }
        return ready;
    }
    int on_step() noexcept override
    {
        try {
            const auto report = runtime_->step_hakoniwa(1);
            for (const auto& status : report.publisher_statuses) {
                if (status.state != ComponentState::Ready) {
                    return fail(status.detail);
                }
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
    int fail(std::string_view error)
    {
        std::cerr << "SERVER ERROR: " << error << std::endl;
        failed_ = true;
        return -1;
    }

    std::shared_ptr<runner::hakoniwa::CoreAsyncServiceServer> server_;
    std::unique_ptr<runner::LockstepGate> gate_;
    std::unique_ptr<ActuatorRuntime> runtime_;
    std::size_t steps_ {0};
    bool failed_ {false};
};
} // namespace

int main(int argc, char** argv)
{
    if (argc != 3) return 2;
    std::cout << std::unitbuf;
    ServiceCallbacks callbacks;
    return run(argv[1], argv[2], callbacks);
}
