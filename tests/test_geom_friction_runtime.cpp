#include "runtime/actuator_runtime.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runtime/controller/geom_friction_controller.hpp"
#include "runtime/controller/scalar_pdu_command_controller.hpp"
#include "runtime/runtime_factory.hpp"
#include "runtime/source/scalar_pdu_command_source.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace hakoniwa::robot_runtime::runtime;

namespace {

/** Returns one scripted sample per poll; nullopt means no new PDU write. */
class ScriptedFloat64Reader final : public IFloat64EventReader {
public:
    std::vector<std::optional<double>> samples;
    std::size_t next {0};
    bool was_reset {false};

    std::optional<double> take_latest() override
    {
        if (next >= samples.size()) {
            return std::nullopt;
        }
        return samples[next++];
    }

    void reset() noexcept override
    {
        was_reset = true;
    }
};

class RecordingPlant final : public IActuatorPlant {
public:
    std::uint64_t time_usec {0};
    std::vector<ActuatorCommand> actuator_commands;
    std::vector<std::shared_ptr<const GeomFrictionDirective>> friction_directives;
    std::optional<std::uint64_t> directives_applied_at_usec;

    std::uint64_t delta_time_usec() const noexcept override
    {
        return 1'000;
    }

    RobotState read_state() const override
    {
        RobotState state;
        state.sample_time_usec = time_usec;
        state.actuators.push_back({"joint1", 0.0, 0.0, 0.0, time_usec});
        return state;
    }

    void apply_directives(const PlantDirectiveList& directives) override
    {
        friction_directives = directives_of<GeomFrictionDirective>(directives);
        directives_applied_at_usec = time_usec;
    }

    RobotState step(const std::vector<ActuatorCommand>& commands) override
    {
        actuator_commands = commands;
        time_usec += delta_time_usec();
        return read_state();
    }

    RobotState reset(const std::uint64_t requested_time_usec) override
    {
        time_usec = requested_time_usec;
        actuator_commands.clear();
        friction_directives.clear();
        directives_applied_at_usec.reset();
        return read_state();
    }
};

/** Records exactly what the Arbiter is offered. */
class RecordingArbiter final : public ICommandArbiter {
public:
    explicit RecordingArbiter(std::shared_ptr<ICommandArbiter> inner)
        : inner_(std::move(inner))
    {
    }

    mutable std::vector<ControllerOutput> offered;

    ArbitrationResult arbitrate(
        const std::vector<ControllerOutput>& outputs,
        const RobotState& current_state,
        const RuntimeStepContext& context) const override
    {
        offered = outputs;
        return inner_->arbitrate(outputs, current_state, context);
    }

private:
    std::shared_ptr<ICommandArbiter> inner_;
};

const std::vector<std::string> tires {
    "car_3_front_left_tire", "car_3_front_right_tire"};

void test_directive_bypasses_arbiter_and_rejects_invalid_values()
{
    auto friction_reader = std::make_shared<ScriptedFloat64Reader>();
    friction_reader->samples = {
        0.8,
        std::nullopt,
        std::numeric_limits<double>::quiet_NaN(),
        -0.1,
        std::numeric_limits<double>::infinity(),
        1.2,
    };
    auto actuator_reader = std::make_shared<ScriptedFloat64Reader>();
    actuator_reader->samples = {0.5};

    auto friction_source = std::make_shared<ScalarPduCommandSource>(
        "geom-friction-pdu:tires", 1'000, friction_reader);
    auto actuator_source = std::make_shared<ScalarPduCommandSource>(
        "plant-pdu:joint1", 100'000, actuator_reader);
    auto friction_controller = std::make_shared<GeomFrictionController>(
        "tires", std::string(friction_source->id()), tires);
    auto actuator_controller = std::make_shared<ScalarPduCommandController>(
        "scalar-command:joint1", std::string(actuator_source->id()),
        "joint1", ActuatorCommandType::Position);
    auto plant = std::make_shared<RecordingPlant>();
    auto arbiter = std::make_shared<RecordingArbiter>(
        std::make_shared<PriorityCommandArbiter>(std::vector<ControllerPriority> {
            {"scalar-command:joint1", 0, "scalar-pdu-direct"}}));
    ActuatorRuntime runtime(
        {friction_source, actuator_source},
        {actuator_controller},
        arbiter,
        plant,
        {},
        {friction_controller});

    const auto first = runtime.step();
    // Only the arbitrated Controller output is offered to the Arbiter.
    assert(arbiter->offered.size() == 1);
    assert(arbiter->offered[0].controller_id == "scalar-command:joint1");
    assert(first.arbitration.selected_commands.size() == 1);
    assert(first.arbitration.selected_commands[0].actuator_id == "joint1");
    assert(plant->actuator_commands.size() == 1);
    // The friction value reaches the Plant only as a directive.
    assert(first.plant_directives.size() == 1);
    assert(first.plant_directives[0]->type_name() == "hakoniwa/GeomFriction");
    assert(first.directive_controller_statuses.size() == 1);
    assert(first.directive_controller_statuses[0].state == ComponentState::Ready);
    assert(plant->directives_applied_at_usec == 0);
    assert(plant->friction_directives.size() == 1);
    assert(plant->friction_directives[0]->geom_names() == tires);
    assert(plant->friction_directives[0]->sliding_friction() == 0.8);
    assert(plant->friction_directives[0]->created_at_usec() == 0);

    // No new value: no directive, the Plant keeps the applied friction.
    const auto idle = runtime.step();
    assert(idle.plant_directives.empty());
    assert(plant->friction_directives.empty());
    assert(idle.directive_controller_statuses[0].state == ComponentState::Ready);

    for (int index = 0; index < 3; ++index) {
        const auto rejected = runtime.step();
        assert(rejected.plant_directives.empty());
        assert(plant->friction_directives.empty());
        assert(rejected.directive_controller_statuses[0].state
            == ComponentState::Degraded);
    }

    const auto updated = runtime.step();
    assert(updated.plant_directives.size() == 1);
    assert(plant->friction_directives.size() == 1);
    assert(plant->friction_directives[0]->sliding_friction() == 1.2);
    assert(plant->friction_directives[0]->created_at_usec() == 5'000);
    assert(arbiter->offered.size() == 1);

    (void)runtime.reset(0);
    assert(friction_reader->was_reset);
    const auto after_reset = runtime.step();
    assert(after_reset.plant_directives.empty());
    assert(after_reset.directive_controller_statuses[0].state
        == ComponentState::WaitingForInput);
}

void test_directives_of_picks_one_kind()
{
    PlantDirectiveList mixed {
        std::make_shared<const GeomFrictionDirective>(tires, 0.9, 0),
        std::make_shared<const MirrorBodyDirective>(MirrorBodyCommand {}),
        std::make_shared<const GeomFrictionDirective>(tires, 1.1, 0),
    };
    const auto frictions = directives_of<GeomFrictionDirective>(mixed);
    assert(frictions.size() == 2);
    assert(frictions[0]->sliding_friction() == 0.9);
    assert(frictions[1]->sliding_friction() == 1.1);
    assert(directives_of<MirrorBodyDirective>(mixed).size() == 1);
}

void test_factory_wires_geom_friction_as_directive()
{
    RuntimeDefinition definition;
    RuntimeGeomFrictionConfig friction;
    friction.component_id = "car-3-tire-friction";
    friction.pdu_robot = "Car-3";
    friction.pdu_name = "tire_friction";
    friction.geoms = tires;
    definition.geom_frictions.push_back(friction);

    auto reader = std::make_shared<ScriptedFloat64Reader>();
    reader->samples = {0.7};
    auto plant = std::make_shared<RecordingPlant>();
    const auto unused_actuator_reader =
        [](const RuntimeActuatorConfig&) -> std::shared_ptr<IFloat64EventReader> {
        return nullptr;
    };
    auto runtime = build_runtime(
        definition,
        plant,
        unused_actuator_reader,
        {}, {}, {}, {}, {}, {}, {},
        [&](const RuntimeGeomFrictionConfig& config) {
            assert(config.component_id == "car-3-tire-friction");
            return reader;
        });

    const auto report = runtime->step();
    assert(report.source_statuses.size() == 1);
    assert(report.source_statuses[0].component_id
        == "geom-friction-pdu:car-3-tire-friction");
    assert(report.controller_statuses.empty());
    assert(report.arbitration.selected_commands.empty());
    assert(report.directive_controller_statuses.size() == 1);
    assert(report.directive_controller_statuses[0].component_id
        == "car-3-tire-friction");
    assert(plant->friction_directives.size() == 1);
    assert(plant->friction_directives[0]->sliding_friction() == 0.7);

    bool rejected = false;
    try {
        (void)build_runtime(definition, plant, unused_actuator_reader);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);
}

} // namespace

int main()
{
    test_directive_bypasses_arbiter_and_rejects_invalid_values();
    test_directives_of_picks_one_kind();
    test_factory_wires_geom_friction_as_directive();
    return 0;
}
