// Controller plugin contract: PluginLibrary validation, PluginController
// timing / tick flags / errors, the manifest loader and build_runtime().
//
// Build in Debug: the checks are assert()s and Release compiles them out.
#include "plugin/plugin_library.hpp"
#include "runtime/controller/plugin_controller.hpp"
#include "runtime/runtime_factory.hpp"
#include "runtime/runtime_factory_internal.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <dlfcn.h>

#ifdef NDEBUG
#error "test_controller_plugin must be built without NDEBUG (assert() is the check)"
#endif

using namespace hakoniwa::robot_runtime;
using namespace hakoniwa::robot_runtime::runtime;

namespace {

constexpr std::uint64_t step_usec = 2'000;
constexpr std::uint64_t step_ns = step_usec * 1'000;
constexpr double epsilon = 1.0e-12;

// Same layout as HoldTargetStats in hold_target_plugin.c.
struct HoldTargetStats {
    std::uint64_t update_count;
    std::uint64_t reset_count;
    std::uint64_t last_step_index;
    std::uint64_t last_sim_time_ns;
    std::uint8_t last_selected;
    std::uint8_t last_reset;
};

using StatsFunction = void (*)(HoldTargetStats*);

/** Reads the sample plugin's test hook through a second handle to the same library. */
HoldTargetStats plugin_stats()
{
    void* handle = dlopen(HOLD_TARGET_PLUGIN_PATH, RTLD_NOW | RTLD_LOCAL);
    assert(handle != nullptr);
    auto function = reinterpret_cast<StatsFunction>(
        dlsym(handle, "hold_target_plugin_test_stats"));
    assert(function != nullptr);
    HoldTargetStats stats {};
    function(&stats);
    dlclose(handle);
    return stats;
}

std::filesystem::path temp_dir()
{
    auto path = std::filesystem::temp_directory_path() / "hako_controller_plugin_test";
    std::filesystem::create_directories(path);
    return path;
}

std::string write_file(const std::string& name, const std::string& text)
{
    const auto path = temp_dir() / name;
    std::ofstream output(path);
    output << text;
    return path.string();
}

void expect_open_failure(
    const std::string& path,
    std::uint32_t kind,
    const std::string& expected_text)
{
    try {
        auto library = plugin::PluginLibrary::open(path, kind);
        (void)library;
    } catch (const std::runtime_error& error) {
        const std::string message = error.what();
        std::cout << "  refused: " << message << std::endl;
        assert(message.find(path) != std::string::npos);
        assert(message.find(expected_text) != std::string::npos);
        return;
    }
    assert(false && "PluginLibrary::open should have failed");
}

RobotState make_state(std::uint64_t time_usec, double position0, double position1)
{
    RobotState state;
    state.sample_time_usec = time_usec;
    state.actuators.push_back({"joint_a", position0, 0.1, 0.2, time_usec});
    state.actuators.push_back({"joint_b", position1, 0.3, 0.4, time_usec});
    return state;
}

RuntimeStepContext make_context(std::uint64_t step_index, bool plugin_selected)
{
    RuntimeStepContext context;
    context.delta_time_usec = step_usec;
    context.step_index = step_index;
    context.simulation_time_usec = step_index * step_usec;
    if (plugin_selected) {
        context.previous_selected_control_id = "plugin_ctrl";
    }
    return context;
}

std::shared_ptr<PluginController> make_controller(const std::string& config_text)
{
    PluginControllerOptions options;
    options.controller_id = "plugin_ctrl";
    options.library = plugin::PluginLibrary::open(
        HOLD_TARGET_PLUGIN_PATH, HAKO_PLUGIN_KIND_CONTROLLER);
    options.plugin_config_path = write_file("hold_target.cfg", config_text);
    options.actuator_ids = {"joint_a", "joint_b"};
    options.physics_step_ns = step_ns;
    return std::make_shared<PluginController>(std::move(options));
}

void test_library_validation()
{
    std::cout << "library validation" << std::endl;
    expect_open_failure(
        (temp_dir() / "no_such_plugin.so").string(),
        HAKO_PLUGIN_KIND_CONTROLLER, "cannot load");
    expect_open_failure(
        HOLD_TARGET_NO_ENTRY_PATH, HAKO_PLUGIN_KIND_CONTROLLER,
        "has no entry hako_robot_runtime_plugin_v1");
    expect_open_failure(
        HOLD_TARGET_BAD_KIND_PATH, HAKO_PLUGIN_KIND_CONTROLLER,
        "is a data adapter plugin");
    expect_open_failure(
        HOLD_TARGET_SMALL_TABLE_PATH, HAKO_PLUGIN_KIND_CONTROLLER,
        "controller table of");
    expect_open_failure(
        HOLD_TARGET_BAD_ABI_PATH, HAKO_PLUGIN_KIND_CONTROLLER,
        "controller ABI version 99");
    // The data adapter kind is reserved: no host loads it yet.
    expect_open_failure(
        HOLD_TARGET_BAD_KIND_PATH, HAKO_PLUGIN_KIND_DATA_ADAPTER,
        "cannot load data adapter plugins");

    auto library = plugin::PluginLibrary::open(
        HOLD_TARGET_PLUGIN_PATH, HAKO_PLUGIN_KIND_CONTROLLER);
    assert(library->plugin_id() == "hold_target");
    assert(library->header().kind == HAKO_PLUGIN_KIND_CONTROLLER);
    assert(library->controller_table().update != nullptr);
}

void test_update_period_and_flags()
{
    std::cout << "update period, selected, reset" << std::endl;
    // 10 ms control period, 2 ms physics step: one update every 5 steps.
    auto controller = make_controller("period_ns 10000000\ntarget 0.5 -0.25\n");
    assert(controller->steps_per_update() == 5);
    assert(controller->source_id().empty());

    // Step 0: first update after create, reset = 1, holds the current position.
    auto output = controller->update(nullptr, make_state(0, 1.0, 2.0), make_context(0, false));
    auto stats = plugin_stats();
    assert(stats.update_count == 1);
    assert(stats.last_reset == 1);
    assert(stats.last_selected == 0);
    assert(stats.last_sim_time_ns == 0);
    assert(output.controller_id == "plugin_ctrl");
    assert(output.status.state == ComponentState::Ready);
    assert(output.commands.size() == 2);
    assert(output.commands[0].actuator_id == "joint_a");
    assert(output.commands[0].type == ActuatorCommandType::Position);
    assert(std::abs(output.commands[0].value - 1.0) < epsilon);
    assert(std::abs(output.commands[1].value - 2.0) < epsilon);
    assert(output.commands[0].source_id.empty());

    // Steps 1..4: no plugin call, previous commands restamped for the step.
    for (std::uint64_t step = 1; step < 5; ++step) {
        output = controller->update(nullptr, make_state(step * step_usec, 9.0, 9.0),
            make_context(step, true));
        assert(plugin_stats().update_count == 1);
        assert(output.status.state == ComponentState::Ready);
        assert(output.commands.size() == 2);
        assert(std::abs(output.commands[0].value - 1.0) < epsilon);
        assert(output.commands[0].created_at_usec == step * step_usec);
        assert(output.commands[0].expires_at_usec == (step + 1) * step_usec);
    }

    // Step 5: second update, selected, not reset: the targets.
    output = controller->update(nullptr, make_state(5 * step_usec, 1.0, 2.0),
        make_context(5, true));
    stats = plugin_stats();
    assert(stats.update_count == 2);
    assert(stats.last_step_index == 5);
    assert(stats.last_sim_time_ns == 5 * step_ns);
    assert(stats.last_selected == 1);
    assert(stats.last_reset == 0);
    assert(std::abs(output.commands[0].value - 0.5) < epsilon);
    assert(std::abs(output.commands[1].value + 0.25) < epsilon);

    // Step 10: another control took over (not selected): the plugin drops
    // its targets and holds the current position.
    output = controller->update(nullptr, make_state(10 * step_usec, 0.75, 0.0),
        make_context(10, false));
    stats = plugin_stats();
    assert(stats.update_count == 3);
    assert(stats.last_selected == 0);
    assert(std::abs(output.commands[0].value - 0.75) < epsilon);

    // Reset: the next update has reset = 1 and no commands are repeated before it.
    controller->reset(make_state(0, 0.1, 0.2));
    stats = plugin_stats();
    assert(stats.reset_count == 1);
    output = controller->update(nullptr, make_state(step_usec, 0.1, 0.2), make_context(1, false));
    assert(output.status.state == ComponentState::WaitingForInput);
    assert(output.commands.empty());
    output = controller->update(nullptr, make_state(0, 0.1, 0.2), make_context(0, false));
    stats = plugin_stats();
    assert(stats.update_count == 4);
    assert(stats.last_reset == 1);
    assert(std::abs(output.commands[0].value - 0.1) < epsilon);
}

void test_every_step_when_period_is_zero()
{
    std::cout << "period 0 = every step" << std::endl;
    auto controller = make_controller("");
    assert(controller->steps_per_update() == 1);
    for (std::uint64_t step = 0; step < 3; ++step) {
        const auto output = controller->update(
            nullptr, make_state(step * step_usec, 0.3, 0.4), make_context(step, false));
        assert(output.status.state == ComponentState::Ready);
    }
    assert(plugin_stats().update_count == 3);
}

void test_errors()
{
    std::cout << "errors" << std::endl;
    auto controller = make_controller("target 0.5 -0.25\nfail_on_update 2\n");
    auto output = controller->update(nullptr, make_state(0, 1.0, 2.0), make_context(0, false));
    assert(output.status.state == ComponentState::Ready);

    // The plugin fails: Error with its message, last commands kept.
    output = controller->update(nullptr, make_state(step_usec, 1.0, 2.0), make_context(1, true));
    assert(output.status.state == ComponentState::Error);
    assert(!output.status.can_produce_commands());
    assert(output.status.detail.find("injected failure") != std::string::npos);
    assert(output.commands.size() == 2);
    assert(std::abs(output.commands[0].value - 1.0) < epsilon);

    // The next successful update clears the error.
    output = controller->update(nullptr, make_state(2 * step_usec, 1.0, 2.0), make_context(2, true));
    assert(output.status.state == ComponentState::Ready);
    assert(std::abs(output.commands[0].value - 0.5) < epsilon);

    // A joint missing from the state is an error, not a crash.
    RobotState partial = make_state(3 * step_usec, 1.0, 2.0);
    partial.actuators.pop_back();
    output = controller->update(nullptr, partial, make_context(3, true));
    assert(output.status.state == ComponentState::Error);
    assert(output.status.detail.find("joint_b") != std::string::npos);

    // A physics step other than the one given at create is an error.
    auto context = make_context(4, true);
    context.delta_time_usec = 1'000;
    output = controller->update(nullptr, make_state(4 * step_usec, 1.0, 2.0), context);
    assert(output.status.state == ComponentState::Error);

    // A control period that is not a whole multiple of the physics step.
    bool refused = false;
    try {
        auto bad = make_controller("period_ns 3000000\n");
        (void)bad;
    } catch (const std::runtime_error& error) {
        refused = std::string(error.what()).find("not a whole multiple") != std::string::npos;
    }
    assert(refused);

    // create failing (bad target count) is reported with the plugin's message.
    refused = false;
    try {
        auto bad = make_controller("target 1.0\n");
        (void)bad;
    } catch (const std::runtime_error& error) {
        refused = std::string(error.what()).find("target count") != std::string::npos;
    }
    assert(refused);
}

RuntimeActuatorConfig position_actuator(const std::string& id)
{
    RuntimeActuatorConfig config;
    config.component_id = id;
    config.component_type = "joint_position_actuator";
    config.command_type = ActuatorCommandType::Position;
    config.limits = {-3.0, 3.0};
    config.command_timeout_usec = 100'000;
    return config;
}

void test_manifest_loader()
{
    std::cout << "manifest loader" << std::endl;
    const auto dir = temp_dir() / "manifest";
    std::filesystem::create_directories(dir / "config");
    // Relative plugin paths resolve against the config file's directory.
    std::filesystem::copy_file(
        HOLD_TARGET_PLUGIN_PATH, dir / "config" / "libhold_target.so",
        std::filesystem::copy_options::overwrite_existing);
    write_file("manifest/config/hold.cfg", "target 0.5 -0.25\n");

    auto load = [&](const std::string& config_text, RuntimeDefinition& definition,
                    std::string& error) {
        write_file("manifest/config/plugin.json", config_text);
        detail::RuntimeParseContext context;
        context.base_path = dir;
        context.components.push_back({
            {"id", "plugin_ctrl"},
            {"kind", "controller"},
            {"type", "plugin"},
            {"config", "config/plugin.json"},
        });
        definition = {};
        definition.actuators = {position_actuator("joint_a"), position_actuator("joint_b")};
        return detail::load_plugin_controller_definitions(context, definition, &error);
    };

    RuntimeDefinition definition;
    std::string error;
    assert(load(R"({"plugin": {"path": "libhold_target.so", "config": "hold.cfg"},
                    "joints": ["joint_b", "joint_a"]})", definition, error));
    assert(definition.plugin_controllers.size() == 1);
    const auto& config = definition.plugin_controllers.front();
    assert(config.component_id == "plugin_ctrl");
    assert(config.plugin_path == (dir / "config" / "libhold_target.so").lexically_normal().string());
    assert(config.plugin_config_path == (dir / "config" / "hold.cfg").lexically_normal().string());
    assert((config.actuator_ids == std::vector<std::string> {"joint_b", "joint_a"}));
    assert(config.priority == 10);
    assert(config.joint_state_source == "truth");

    assert(load(R"({"plugin": {"path": "libhold_target.so"}, "joints": ["joint_a"],
                    "priority": 15, "joint_state_source": "truth"})", definition, error));
    assert(definition.plugin_controllers.front().priority == 15);
    assert(definition.plugin_controllers.front().plugin_config_path.empty());

    assert(!load(R"({"plugin": {"path": "missing.so"}, "joints": ["joint_a"]})",
        definition, error));
    assert(error.find("missing.so") != std::string::npos);
    assert(!load(R"({"plugin": {"path": "libhold_target.so"}, "joints": ["joint_x"]})",
        definition, error));
    assert(error.find("joint_x") != std::string::npos);
    assert(!load(R"({"plugin": {"path": "libhold_target.so"}, "joints": ["joint_a", "joint_a"]})",
        definition, error));
    assert(!load(R"({"plugin": {"path": "libhold_target.so"}, "joints": []})",
        definition, error));
    assert(!load(R"({"joints": ["joint_a"]})", definition, error));
    assert(!load(R"({"plugin": {"path": "libhold_target.so"}, "joints": ["joint_a"],
                     "joint_state_source": "observed"})", definition, error));
    assert(error.find("not supported yet") != std::string::npos);
    assert(!load(R"({"plugin": {"path": "libhold_target.so"}, "joints": ["joint_a"],
                     "priority": 1.5})", definition, error));
}

class EmptyScalarReader final : public IFloat64EventReader {
public:
    std::optional<double> take_latest() override { return std::nullopt; }
    void reset() noexcept override { }
};

class RecordingPlant final : public IActuatorPlant {
public:
    RecordingPlant()
    {
        state_ = make_state(0, 1.0, 2.0);
    }

    std::uint64_t delta_time_usec() const noexcept override { return step_usec; }
    RobotState read_state() const override { return state_; }

    RobotState step(const std::vector<ActuatorCommand>& commands) override
    {
        applied = commands;
        state_.sample_time_usec += step_usec;
        return state_;
    }

    RobotState reset(const std::uint64_t time = 0) override
    {
        state_.sample_time_usec = time;
        return state_;
    }

    std::vector<ActuatorCommand> applied;

private:
    RobotState state_;
};

void test_build_runtime()
{
    std::cout << "build_runtime" << std::endl;
    RuntimeDefinition definition;
    definition.actuators = {position_actuator("joint_a"), position_actuator("joint_b")};
    RuntimePluginControllerConfig config;
    config.component_id = "plugin_ctrl";
    config.plugin_path = HOLD_TARGET_PLUGIN_PATH;
    config.plugin_config_path = write_file(
        "runtime.cfg", "period_ns 10000000\ntarget 0.5 -0.25\n");
    config.actuator_ids = {"joint_a", "joint_b"};
    definition.plugin_controllers.push_back(config);

    auto plant = std::make_shared<RecordingPlant>();
    auto runtime = build_runtime(
        definition,
        plant,
        [](const RuntimeActuatorConfig&) {
            return std::make_shared<EmptyScalarReader>();
        });

    // Step 0: the plugin holds the current position and is selected (it
    // outranks the Hold Controller, priority 10 against -10).
    auto report = runtime->step();
    assert(report.arbitration.selected_control_id == "plugin_ctrl");
    assert(plant->applied.size() == 2);
    assert(std::abs(plant->applied[0].value - 1.0) < epsilon);

    // Steps 1..4 repeat; step 5 is the next plugin update, which sees that
    // it was selected and returns the targets.
    for (int step = 1; step < 5; ++step) {
        report = runtime->step();
        assert(report.arbitration.selected_control_id == "plugin_ctrl");
        assert(std::abs(plant->applied[0].value - 1.0) < epsilon);
    }
    report = runtime->step();
    const auto stats = plugin_stats();
    assert(stats.update_count == 2);
    assert(stats.last_selected == 1);
    assert(report.arbitration.selected_control_id == "plugin_ctrl");
    assert(std::abs(plant->applied[0].value - 0.5) < epsilon);
    assert(std::abs(plant->applied[1].value + 0.25) < epsilon);

    // reset() resets the plugin; its next update is flagged as reset.
    (void)runtime->reset(0);
    report = runtime->step();
    assert(plugin_stats().last_reset == 1);
    assert(plugin_stats().reset_count == 1);
}

} // namespace

int main()
{
    test_library_validation();
    test_update_period_and_flags();
    test_every_step_when_period_is_zero();
    test_errors();
    test_manifest_loader();
    test_build_runtime();
    std::cout << "test_controller_plugin: all checks passed" << std::endl;
    return 0;
}
