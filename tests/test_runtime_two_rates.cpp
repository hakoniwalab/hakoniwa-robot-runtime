// Stage B0 of docs/design-async-rpc.md: two rates.
//
// One Hakoniwa step runs N physics steps. Input and output follow the Hakoniwa
// step: command sources are polled once, in the first physics step, and the
// publishers run once, after the last one. Control, arbitration and the plant
// follow the physics step and run N times.
//
// Built only with HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE=1.
#include "runtime/actuator_runtime.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

using namespace hakoniwa::robot_runtime::runtime;

namespace {

constexpr std::uint64_t kDeltaUsec = 1'000;
constexpr const char* kSource = "source";
constexpr const char* kController = "controller";
constexpr const char* kActuator = "joint";

class ValueInput final : public IControllerInput {
public:
    ValueInput(const std::uint64_t sequence, const std::uint64_t time_usec, const double value)
        : value_(value)
    {
        metadata_.source_id = kSource;
        metadata_.sequence = sequence;
        metadata_.created_at_usec = time_usec;
    }

    const ControllerInputMetadata& metadata() const noexcept override { return metadata_; }
    std::string_view type_name() const noexcept override { return "value"; }
    double value() const noexcept { return value_; }

private:
    ControllerInputMetadata metadata_;
    double value_;
};

/** Counts polls; every poll returns a new input whose value is the poll count. */
class CountingSource final : public ICommandSource {
public:
    int polls {0};
    std::vector<std::uint64_t> polled_at_usec;

    std::string_view id() const noexcept override { return kSource; }

    CommandSourcePollResult poll(const RuntimeStepContext& context) override
    {
        ++polls;
        polled_at_usec.push_back(context.simulation_time_usec);
        CommandSourcePollResult result;
        result.input = std::make_shared<ValueInput>(
            static_cast<std::uint64_t>(polls), context.simulation_time_usec,
            static_cast<double>(polls));
        result.status.state = ComponentState::Ready;
        return result;
    }

    void reset() noexcept override
    {
        polls = 0;
        polled_at_usec.clear();
    }
};

/** Holds the last value it was given and commands it every step. */
class HoldingController final : public IController {
public:
    int updates {0};
    /** One entry per update: true when that update got a new input. */
    std::vector<bool> got_input;
    double held {0.0};

    std::string_view id() const noexcept override { return kController; }
    std::string_view source_id() const noexcept override { return kSource; }
    bool accepts(const IControllerInput& input) const noexcept override
    {
        return input.type_name() == "value";
    }

    ControllerOutput update(
        std::shared_ptr<const IControllerInput> input,
        const RobotState&,
        const RuntimeStepContext& context) override
    {
        ++updates;
        got_input.push_back(input != nullptr);
        if (input != nullptr) {
            held = static_cast<const ValueInput&>(*input).value();
        }
        ControllerOutput output;
        output.controller_id = kController;
        output.status.state = ComponentState::Ready;
        ActuatorCommand command;
        command.actuator_id = kActuator;
        command.value = held;
        command.source_id = kSource;
        command.created_at_usec = context.simulation_time_usec;
        output.commands.push_back(command);
        return output;
    }

    void reset(const RobotState&) override
    {
        updates = 0;
        got_input.clear();
        held = 0.0;
    }
};

class CountingPlant final : public IActuatorPlant {
public:
    std::uint64_t time_usec {0};
    int steps {0};
    std::vector<double> applied;

    std::uint64_t delta_time_usec() const noexcept override { return kDeltaUsec; }

    RobotState read_state() const override
    {
        RobotState state;
        state.sample_time_usec = time_usec;
        return state;
    }

    RobotState step(const std::vector<ActuatorCommand>& commands) override
    {
        ++steps;
        applied.push_back(commands.empty() ? -1.0 : commands.front().value);
        time_usec += kDeltaUsec;
        return read_state();
    }

    RobotState reset(const std::uint64_t requested_time_usec) override
    {
        time_usec = requested_time_usec;
        steps = 0;
        applied.clear();
        return read_state();
    }
};

class CountingPublisher final : public IStatePublisher {
public:
    int publishes {0};
    std::vector<std::uint64_t> published_at_usec;

    std::string_view id() const noexcept override { return "publisher"; }

    ComponentStatus publish(const RobotState& state, const RuntimeStepContext&) noexcept override
    {
        ++publishes;
        published_at_usec.push_back(state.sample_time_usec);
        ComponentStatus status;
        status.state = ComponentState::Ready;
        return status;
    }

    void reset() noexcept override
    {
        publishes = 0;
        published_at_usec.clear();
    }
};

struct Fixture {
    std::shared_ptr<CountingSource> source = std::make_shared<CountingSource>();
    std::shared_ptr<HoldingController> controller = std::make_shared<HoldingController>();
    std::shared_ptr<CountingPlant> plant = std::make_shared<CountingPlant>();
    std::shared_ptr<CountingPublisher> publisher = std::make_shared<CountingPublisher>();
    ActuatorRuntime runtime {
        {source},
        {controller},
        std::make_shared<PriorityCommandArbiter>(
            std::vector<ControllerPriority> {{kController, 10, ""}}),
        plant,
        {publisher}};
};

// substeps = 4: one poll, four controller updates, four plant steps, one publish.
void test_one_hakoniwa_step_polls_once_and_runs_physics_n_times()
{
    Fixture f;
    const auto report = f.runtime.step_hakoniwa(4);

    assert(f.source->polls == 1);
    assert(f.source->polled_at_usec == std::vector<std::uint64_t>({0}));
    assert(f.controller->updates == 4);
    assert(f.plant->steps == 4);
    assert(f.publisher->publishes == 1);
    assert(f.publisher->published_at_usec == std::vector<std::uint64_t>({4'000}));

    // The report is the last physics step's, with the sources' statuses of the
    // first one (the only poll) and the one publish.
    assert(report.completed_step.simulation_time_usec == 3'000);
    assert(report.next_step.simulation_time_usec == 4'000);
    assert(report.source_statuses.size() == 1);
    assert(report.publisher_statuses.size() == 1);
}

// The input reaches the controller in the first physics step only; in the
// others the controller gets no input (not an error) and holds its value.
void test_later_substeps_get_no_input_and_controller_holds()
{
    Fixture f;
    (void)f.runtime.step_hakoniwa(4);

    assert(f.controller->got_input == std::vector<bool>({true, false, false, false}));
    assert(f.plant->applied == std::vector<double>({1.0, 1.0, 1.0, 1.0}));
}

// Two Hakoniwa steps: two polls, at the start of each, and two publishes.
void test_consecutive_hakoniwa_steps()
{
    Fixture f;
    (void)f.runtime.step_hakoniwa(4);
    (void)f.runtime.step_hakoniwa(4);

    assert(f.source->polls == 2);
    assert(f.source->polled_at_usec == std::vector<std::uint64_t>({0, 4'000}));
    assert(f.controller->updates == 8);
    assert(f.plant->steps == 8);
    assert(f.plant->applied
        == std::vector<double>({1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0}));
    assert(f.publisher->published_at_usec == std::vector<std::uint64_t>({4'000, 8'000}));
}

// substeps = 1 behaves like step(): one poll, one update, one plant step, one publish.
void test_one_substep_is_one_plain_step()
{
    Fixture f;
    const auto report = f.runtime.step_hakoniwa(1);

    assert(f.source->polls == 1);
    assert(f.controller->updates == 1);
    assert(f.plant->steps == 1);
    assert(f.publisher->publishes == 1);
    assert(report.next_step.simulation_time_usec == 1'000);
}

// substeps = 0 is refused before anything runs.
void test_zero_substeps_is_refused()
{
    Fixture f;
    bool thrown = false;
    try {
        (void)f.runtime.step_hakoniwa(0);
    } catch (const std::invalid_argument&) {
        thrown = true;
    }
    assert(thrown);
    assert(f.source->polls == 0);
    assert(f.controller->updates == 0);
    assert(f.plant->steps == 0);
    assert(f.publisher->publishes == 0);
}

// step() itself is unchanged: every call polls (direct callers keep today's behaviour).
void test_plain_step_still_polls_every_call()
{
    Fixture f;
    for (int i = 0; i < 4; ++i) {
        (void)f.runtime.step(i == 3);
    }
    assert(f.source->polls == 4);
    assert(f.controller->got_input == std::vector<bool>({true, true, true, true}));
    assert(f.plant->steps == 4);
    assert(f.publisher->publishes == 1);
}

// After reset the next Hakoniwa step polls again at its first physics step.
void test_reset_then_hakoniwa_step()
{
    Fixture f;
    (void)f.runtime.step_hakoniwa(4);
    (void)f.runtime.reset(0);
    (void)f.runtime.step_hakoniwa(2);

    assert(f.source->polls == 1);
    assert(f.controller->got_input == std::vector<bool>({true, false}));
    assert(f.plant->steps == 2);
    assert(f.publisher->publishes == 1);
}

} // namespace

int main()
{
    test_one_hakoniwa_step_polls_once_and_runs_physics_n_times();
    test_later_substeps_get_no_input_and_controller_holds();
    test_consecutive_hakoniwa_steps();
    test_one_substep_is_one_plain_step();
    test_zero_substeps_is_refused();
    test_plain_step_still_polls_every_call();
    test_reset_then_hakoniwa_step();
    std::cout << "runtime two-rates tests passed" << std::endl;
    return 0;
}
