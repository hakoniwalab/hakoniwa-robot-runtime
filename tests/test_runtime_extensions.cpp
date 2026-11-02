#include "runtime/runtime_factory.hpp"
#include "runtime/controller/scalar_pdu_command_controller.hpp"

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

} // namespace

int main() {
    test_standard_controls_unchanged();
    test_replacement_uses_standard_pipeline();
    test_invalid_replacements_fail();
}
