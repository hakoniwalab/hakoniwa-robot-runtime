// Stage 2c-0: Plant operations are defined by the common layer and carried
// beside the arbitrated commands (notes 2026-11-27, plan A). Written before the
// implementation. Builds without HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE.
#undef NDEBUG

#include "runtime/actuator_runtime.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runtime/plant/plant_operations.hpp"
#include "runtime/service/async_service_pipeline.hpp"
#include "runtime/service/robot_services.hpp"

#include <cassert>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace hakoniwa::robot_runtime::runtime;

namespace {

constexpr auto kJoint = "joint";
constexpr std::uint64_t kDelta = 1'000;

enum class Call { Operations, Step };

/** Records what reaches the Plant and in which order. */
class FakePlant final : public IActuatorPlant {
public:
    PlantCapabilities capability {true, true, true};
    std::vector<Call> calls;
    std::vector<PlantOperations> applied;
    std::vector<std::vector<ActuatorCommand>> stepped;

    std::uint64_t delta_time_usec() const noexcept override { return kDelta; }
    PlantCapabilities capabilities() const noexcept override { return capability; }

    void apply_operations(const PlantOperations& operations) override
    {
        calls.push_back(Call::Operations);
        applied.push_back(operations);
    }

    RobotState read_state() const override
    {
        RobotState state;
        state.sample_time_usec = stepped.size() * kDelta;
        state.actuators.push_back({kJoint, 0.0, 0.0, 0.0, state.sample_time_usec});
        return state;
    }

    RobotState step(const std::vector<ActuatorCommand>& commands) override
    {
        calls.push_back(Call::Step);
        stepped.push_back(commands);
        return read_state();
    }

    RobotState reset(std::uint64_t) override
    {
        stepped.clear();
        return read_state();
    }
};

/** A controller with a fixed output; the test sets what it returns. */
class FakeController final : public IController {
public:
    explicit FakeController(std::string id) : id_(std::move(id)) {}

    ComponentState state {ComponentState::Ready};
    std::optional<double> command;
    PlantOperations operations;

    std::string_view id() const noexcept override { return id_; }
    std::string_view source_id() const noexcept override { return {}; }
    bool accepts(const IControllerInput&) const noexcept override { return false; }

    ControllerOutput update(std::shared_ptr<const IControllerInput>, const RobotState&,
        const RuntimeStepContext& context) override
    {
        ControllerOutput output {id_, {id_, state, {}}, {}, operations};
        if (command) {
            output.commands.push_back({kJoint, ActuatorCommandType::Position, *command, {},
                context.simulation_time_usec, context.simulation_time_usec + kDelta});
        }
        return output;
    }

    void reset(const RobotState&) override {}

private:
    std::string id_;
};

SurfaceFrictionOperation friction(std::string surface, double value)
{
    return {{std::move(surface)}, value};
}

HumanoidMotionOperation walk(std::string humanoid, double speed)
{
    HumanoidMotionOperation motion;
    motion.humanoid = std::move(humanoid);
    motion.linear_velocity = {speed, 0.0, 0.0};
    motion.animation = "walk";
    return motion;
}

struct Fixture {
    Fixture()
    {
        high->command = 1.0;
        low->command = 2.0;
        runtime.emplace(std::vector<std::shared_ptr<ICommandSource>> {},
            std::vector<std::shared_ptr<IController>> {high, low},
            std::make_shared<PriorityCommandArbiter>(std::vector<ControllerPriority> {
                {"high", 20, {}}, {"low", 10, {}},
            }),
            plant, std::vector<std::shared_ptr<IStatePublisher>> {});
    }

    std::shared_ptr<FakePlant> plant = std::make_shared<FakePlant>();
    std::shared_ptr<FakeController> high = std::make_shared<FakeController>("high");
    std::shared_ptr<FakeController> low = std::make_shared<FakeController>("low");
    std::optional<ActuatorRuntime> runtime;
};

// Operations are applied once per physics step, right before the step, also when empty.
void operations_are_applied_once_before_each_physics_step()
{
    Fixture f;
    (void)f.runtime->step_hakoniwa(3);
    assert(f.plant->calls == std::vector<Call>({
        Call::Operations, Call::Step, Call::Operations, Call::Step, Call::Operations, Call::Step}));
    for (const auto& operations : f.plant->applied) {
        assert(operations.surface_frictions.empty() && operations.body_motions.empty()
            && operations.humanoid_motions.empty());
    }
}

// Operations are not arbitrated: a controller that loses arbitration still has them applied.
void operations_of_a_controller_that_lost_arbitration_still_apply()
{
    Fixture f;
    f.low->operations.surface_frictions.push_back(friction("tyre", 0.5));
    const auto report = f.runtime->step();
    assert(report.arbitration.selected_commands.size() == 1);
    assert(report.arbitration.selected_commands.front().value == 1.0);  // high won
    assert(f.plant->applied.back().surface_frictions.size() == 1);
    assert(f.plant->applied.back().surface_frictions.front().surfaces
        == std::vector<std::string> {"tyre"});
    assert(f.plant->applied.back().surface_frictions.front().sliding_friction == 0.5);
}

// Operations of several controllers are merged in controller order, per kind.
void operations_are_merged_in_controller_order()
{
    Fixture f;
    f.high->operations.humanoid_motions.push_back(walk("A", 1.0));
    f.low->operations.humanoid_motions.push_back(walk("B", 2.0));
    f.low->operations.surface_frictions.push_back(friction("road", 0.9));
    (void)f.runtime->step();
    const auto& applied = f.plant->applied.back();
    assert(applied.humanoid_motions.size() == 2);
    assert(applied.humanoid_motions[0].humanoid == "A");
    assert(applied.humanoid_motions[1].humanoid == "B");
    assert(applied.humanoid_motions[1].linear_velocity.x == 2.0);
    assert(applied.surface_frictions.size() == 1);
}

// A controller in Error contributes neither commands nor operations.
void an_erroring_controller_contributes_no_operations()
{
    Fixture f;
    f.high->state = ComponentState::Error;
    f.high->operations.surface_frictions.push_back(friction("tyre", 0.1));
    f.low->operations.surface_frictions.push_back(friction("road", 0.9));
    (void)f.runtime->step();
    const auto& applied = f.plant->applied.back();
    assert(applied.surface_frictions.size() == 1);
    assert(applied.surface_frictions.front().surfaces == std::vector<std::string> {"road"});
}

// An operation the Plant does not implement stops the run before physics.
void an_operation_the_plant_cannot_do_stops_the_run()
{
    Fixture f;
    f.plant->capability.humanoid_motion = false;
    f.low->operations.humanoid_motions.push_back(walk("A", 1.0));
    bool threw = false;
    try {
        (void)f.runtime->step();
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    assert(f.plant->stepped.empty());
}

// A robot's step operations reach the Plant through its controller.
class OperatingRobot final : public IRobotServices {
public:
    bool on_request(std::size_t, std::span<const std::byte>) override { return true; }
    RobotStepResult on_step(const RobotState&, const RuntimeStepContext&, std::span<RobotServiceSlot>) override
    {
        RobotStepResult result;
        result.operations.humanoid_motions.push_back(walk("Person-1", 0.75));
        return result;
    }
    bool on_response(std::size_t, const RobotState&, const RuntimeStepContext&, std::span<std::byte>) override
    {
        return true;
    }
    void reset() override {}
};

class IdleServer final : public IAsyncServiceServer {
public:
    AsyncServiceEvent poll(int) noexcept override { return AsyncServiceEvent::None; }
    std::optional<AsyncServiceState> status(int) noexcept override { return AsyncServiceState::Idle; }
    bool get_request(int, std::vector<std::byte>&) override { return false; }
    bool reply(int, AsyncServiceResultCode, const AsyncServiceResponseWriter&) override { return false; }
};

void robot_step_operations_reach_the_plant()
{
    auto plant = std::make_shared<FakePlant>();
    auto robot = std::make_shared<OperatingRobot>();
    auto service = std::make_shared<AsyncServiceCommandSource>(
        "person/0", 3, std::make_shared<IdleServer>(), robot, 0);
    auto source = std::make_shared<RobotServiceSource>(
        "person", std::vector<std::shared_ptr<AsyncServiceCommandSource>> {service});
    auto controller = std::make_shared<RobotServiceController>("person:controller", source, robot);
    ActuatorRuntime runtime({source}, {controller},
        std::make_shared<PriorityCommandArbiter>(std::vector<ControllerPriority> {{"person:controller", 10, {}}}),
        plant, {});
    const auto report = runtime.step();
    assert(plant->applied.back().humanoid_motions.size() == 1);
    assert(plant->applied.back().humanoid_motions.front().humanoid == "Person-1");
    assert(plant->applied.back().humanoid_motions.front().linear_velocity.x == 0.75);
    // A robot that only operates the Plant is not "waiting": it did something this step.
    assert(report.controller_statuses[0].state == ComponentState::Ready);
}

// The applied operations are part of the step report, outside the arbitration.
void the_report_carries_the_applied_operations()
{
    Fixture f;
    f.low->operations.body_motions.push_back({"crate", {1.0, 2.0, 3.0}, {}, {}, {}});
    const auto report = f.runtime->step();
    assert(report.plant_operations.body_motions.size() == 1);
    assert(report.plant_operations.body_motions.front().body == "crate");
    assert(report.plant_operations.body_motions.front().position.z == 3.0);
}

} // namespace

int main()
{
    operations_are_applied_once_before_each_physics_step();
    operations_of_a_controller_that_lost_arbitration_still_apply();
    operations_are_merged_in_controller_order();
    an_erroring_controller_contributes_no_operations();
    an_operation_the_plant_cannot_do_stops_the_run();
    robot_step_operations_reach_the_plant();
    the_report_carries_the_applied_operations();
    std::cout << "plant operations: all passed" << std::endl;
    return 0;
}
