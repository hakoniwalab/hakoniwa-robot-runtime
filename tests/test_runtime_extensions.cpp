#include "runtime/runtime_factory.hpp"
#include "runtime/controller/scalar_pdu_command_controller.hpp"
#include "runtime/source/non_lockstep_request_queue.hpp"

#include <cassert>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

using namespace hakoniwa::robot_runtime::runtime;

namespace {

class Reader final : public IFloat64EventReader {
public:
    std::optional<double> take_latest() override {
        if (read_) return std::nullopt;
        read_ = true;
        return 0.3;
    }
    void reset() noexcept override { read_ = false; }
private:
    bool read_ {false};
};

class Plant final : public IActuatorPlant {
public:
    std::uint64_t delta_time_usec() const noexcept override { return 1000; }
    RobotState read_state() const override {
        return {{{"joint1", 0.0, 0.0, 0.0, time_}}, time_};
    }
    RobotState step(const std::vector<ActuatorCommand>& commands) override {
        applied = commands;
        time_ += delta_time_usec();
        return read_state();
    }
    RobotState reset(std::uint64_t time = 0) override {
        time_ = time;
        applied.clear();
        return read_state();
    }
    std::vector<ActuatorCommand> applied;
private:
    std::uint64_t time_ {0};
};

class TestRequest final : public IControllerInput {
public:
    explicit TestRequest(ControllerInputMetadata value) : value_(std::move(value)) {}
    const ControllerInputMetadata& metadata() const noexcept override { return value_; }
    std::string_view type_name() const noexcept override { return "test/request"; }
private:
    ControllerInputMetadata value_;
};

class RequestSource final : public ICommandSource {
public:
    std::string_view id() const noexcept override { return "test-source"; }
    CommandSourcePollResult poll(const RuntimeStepContext&) override {
        auto value = queue.try_take();
        if (!value) {
            return {nullptr, {"test-source", ComponentState::WaitingForInput, {}}};
        }
        return {std::make_shared<TestRequest>(std::move(*value)),
            {"test-source", ComponentState::Ready, {}}};
    }
    void reset() noexcept override { (void)queue.drain(); }
    NonLockstepRequestQueue<ControllerInputMetadata> queue {4};
};

class RequestController final : public IController {
public:
    std::string_view id() const noexcept override { return "test-controller"; }
    std::string_view source_id() const noexcept override { return "test-source"; }
    bool accepts(const IControllerInput& input) const noexcept override {
        return input.type_name() == "test/request" && accept;
    }
    ControllerOutput update(std::shared_ptr<const IControllerInput> input,
        const RobotState&, const RuntimeStepContext& context) override {
        if (!input) {
            return {"test-controller",
                {"test-controller", ComponentState::WaitingForInput, {}}, {}};
        }
        return {"test-controller", {"test-controller", ComponentState::Ready, {}},
            {{"joint1", ActuatorCommandType::Position, 0.3, "test-source",
                context.simulation_time_usec,
                context.simulation_time_usec + context.delta_time_usec}}};
    }
    void reset(const RobotState&) override {}
    bool accept {true};
};

class RequestPublisher final : public IRequestPublisher {
public:
    std::string_view id() const noexcept override { return "test-replies"; }
    ComponentStatus publish(const RobotState& state, const RuntimeStepContext& context,
        const std::vector<ProcessedInput>& inputs,
        const ArbitrationResult& arbitration) noexcept override {
        assert(state.sample_time_usec == context.simulation_time_usec);
        assert(inputs.size() <= 1);
        ++step_calls;
        if (!inputs.empty()) {
            sequences.push_back(inputs.front().metadata.sequence);
            deliveries.push_back(inputs.front().delivery);
            selections.push_back(arbitration.selected_control_id);
        }
        return {"test-replies", fail ? ComponentState::Error : ComponentState::Ready,
            fail ? "transport unavailable" : ""};
    }
    void reset() noexcept override { ++resets; }
    int step_calls {0};
    int resets {0};
    bool fail {false};
    std::vector<std::uint64_t> sequences;
    std::vector<InputDelivery> deliveries;
    std::vector<std::optional<std::string>> selections;
};

RuntimeDefinition definition() {
    RuntimeDefinition result;
    RuntimeActuatorConfig actuator;
    actuator.component_id = "joint1";
    actuator.command_type = ActuatorCommandType::Position;
    actuator.command_timeout_usec = 100'000;
    result.actuators.push_back(actuator);
    return result;
}

void test_standard_controls_unchanged() {
    auto plant = std::make_shared<Plant>();
    int subscribed = 0;
    auto runtime = build_runtime(definition(), plant,
        [&](const RuntimeActuatorConfig&) -> std::shared_ptr<IFloat64EventReader> {
            ++subscribed;
            return std::make_shared<Reader>();
        });
    const auto report = runtime->step();
    assert(subscribed == 1);
    assert(report.arbitration.selected_control_id == "scalar-pdu-direct");
    assert(plant->applied.size() == 1 && plant->applied[0].value == 0.3);
}

void test_replacement_uses_standard_pipeline() {
    auto plant = std::make_shared<Plant>();
    auto reader = std::make_shared<Reader>();
    RuntimeExtensions extensions;
    extensions.replaced_control_components.insert("joint1");
    extensions.sources.push_back(std::make_shared<ScalarPduCommandSource>(
        "rpc-source", 100'000, reader));
    extensions.controllers.push_back(std::make_shared<ScalarPduCommandController>(
        "rpc-controller", "rpc-source", "joint1", ActuatorCommandType::Position));
    extensions.priorities.push_back({"rpc-controller", 0, {}});
    auto runtime = build_runtime(definition(), plant,
        [](const RuntimeActuatorConfig&) -> std::shared_ptr<IFloat64EventReader> {
            throw std::runtime_error("replaced PDU input was subscribed");
        }, {}, {}, {}, {}, {}, std::move(extensions));
    const auto first = runtime->step();
    assert(first.source_statuses.size() == 1);
    assert(first.controller_statuses.size() == 1);
    assert(first.arbitration.selected_control_id == "rpc-controller");
    assert(plant->applied.size() == 1);
    assert(plant->applied[0].source_id == "rpc-source");
    const auto idle = runtime->step();
    assert(idle.source_statuses[0].state == ComponentState::WaitingForInput);
    assert(plant->read_state().sample_time_usec == 2000);

    RuntimeExtensions all_replaced;
    all_replaced.replaced_control_components.insert("joint1");
    auto without_reader = build_runtime(definition(), std::make_shared<Plant>(),
        {}, {}, {}, {}, {}, {}, std::move(all_replaced));
    assert(without_reader->step().source_statuses.empty());
}

void test_invalid_replacements_fail() {
    RuntimeExtensions extensions;
    extensions.replaced_control_components.insert("missing");
    try {
        (void)build_runtime(definition(), std::make_shared<Plant>(),
            [](const RuntimeActuatorConfig&) {
                return std::make_shared<Reader>();
            }, {}, {}, {}, {}, {}, std::move(extensions));
        assert(false && "unknown control replacement must fail");
    } catch (const std::invalid_argument&) {
    }
}

void test_non_lockstep_requests_and_responses() {
    NonLockstepRequestQueue<int> queue(2);
    assert(queue.try_push(10) && queue.try_push(20) && !queue.try_push(30));
    const auto removed = queue.remove_if([](int value) { return value == 10; });
    assert(removed.size() == 1 && removed[0] == 10);
    assert(queue.try_push(30));
    assert(queue.try_take() == 20 && queue.try_take() == 30);
    assert(!queue.try_take());
    assert(queue.try_push(40));
    assert(queue.drain() == std::vector<int>{40});

    auto source = std::make_shared<RequestSource>();
    auto controller = std::make_shared<RequestController>();
    auto replies = std::make_shared<RequestPublisher>();
    auto plant = std::make_shared<Plant>();
    RuntimeExtensions extensions;
    extensions.replaced_control_components.insert("joint1");
    extensions.sources.push_back(source);
    extensions.controllers.push_back(controller);
    extensions.priorities.push_back({"test-controller", 0, {}});
    extensions.request_publishers.push_back(replies);
    auto runtime = build_runtime(definition(), plant, {}, {}, {}, {}, {}, {},
        std::move(extensions));

    assert(source->queue.try_push({"test-source", 41, 0, {}}));
    const auto first = runtime->step(false);
    assert(first.processed_inputs.size() == 1);
    assert(first.processed_inputs[0].metadata.sequence == 41);
    assert(first.processed_inputs[0].delivery == InputDelivery::Delivered);
    assert(first.processed_inputs[0].controller_statuses[0].state == ComponentState::Ready);
    assert(first.request_publisher_statuses.size() == 1);
    assert(first.publisher_statuses.empty());
    assert(replies->step_calls == 1 && replies->sequences == std::vector<std::uint64_t>{41});
    assert(replies->selections.back() == "test-controller");
    assert(plant->applied[0].value == 0.3);

    const auto idle = runtime->step();
    assert(idle.processed_inputs.empty() && replies->step_calls == 2);

    controller->accept = false;
    assert(source->queue.try_push({"test-source", 42, 0, {}}));
    const auto unsupported = runtime->step();
    assert(unsupported.processed_inputs[0].delivery == InputDelivery::Unsupported);
    assert(unsupported.processed_inputs[0].controller_statuses.empty());
    assert(!replies->selections.back());

    assert(source->queue.try_push({"test-source", 43, 10'000, {}}));
    const auto inactive = runtime->step();
    assert(inactive.processed_inputs[0].delivery == InputDelivery::Inactive);
    assert(inactive.source_statuses[0].state == ComponentState::Degraded);
    assert(replies->sequences == (std::vector<std::uint64_t>{41, 42, 43}));

    assert(source->queue.try_push({"test-source", 44, 0, {}}));
    const auto timed_out = source->queue.remove_if([](const auto& request) {
        return request.sequence == 44;
    });
    assert(timed_out.size() == 1);
    const auto resumed = runtime->step();
    assert(resumed.processed_inputs.empty());
    assert(resumed.arbitration.selected_commands.empty());
    assert(replies->sequences.size() == 3);

    controller->accept = true;
    replies->fail = true;
    assert(source->queue.try_push({"test-source", 46, 0, {}}));
    const auto before_failure = plant->read_state().sample_time_usec;
    const auto failed_reply = runtime->step();
    assert(failed_reply.request_publisher_statuses[0].state == ComponentState::Error);
    assert(plant->read_state().sample_time_usec == before_failure + 1'000);
    assert(failed_reply.processed_inputs[0].metadata.sequence == 46);
    replies->fail = false;

    assert(source->queue.try_push({"test-source", 45, 0, {}}));
    (void)runtime->reset();
    assert(replies->resets == 1);
    assert(!source->queue.try_take());
    assert(runtime->step().processed_inputs.empty());
}

} // namespace

int main() {
    test_standard_controls_unchanged();
    test_replacement_uses_standard_pipeline();
    test_invalid_replacements_fail();
    test_non_lockstep_requests_and_responses();
}
