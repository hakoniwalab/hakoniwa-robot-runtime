#undef NDEBUG

#include "runtime/actuator_runtime.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runtime/runtime_factory_manifest.hpp"
#include "runtime/service/service_plugin_adapter.hpp"
#include "runtime/service/async_service_pipeline.hpp"

#include <array>
#include <cassert>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>

#include <nlohmann/json.hpp>

using namespace hakoniwa::robot_runtime;
using namespace hakoniwa::robot_runtime::runtime;

namespace {

class Server final : public IAsyncServiceServer {
public:
    AsyncServiceEvent poll(int) noexcept override
    {
        if (cancel) {
            cancel = false;
            state = AsyncServiceState::Canceling;
            return AsyncServiceEvent::Cancel;
        }
        if (request) {
            state = AsyncServiceState::Doing;
            return AsyncServiceEvent::Request;
        }
        return AsyncServiceEvent::None;
    }

    std::optional<AsyncServiceState> status(int) noexcept override { return state; }

    bool get_request(int, std::vector<std::byte>& bytes) override
    {
        bytes = {*request};
        request.reset();
        return true;
    }

    bool reply(int, AsyncServiceResultCode code, const AsyncServiceResponseWriter& write) override
    {
        packet = {std::byte{0}, std::byte{'O'}, std::byte{'K'}, std::byte{0}};
        wrote_reply = static_cast<bool>(write);
        assert(!write || write(packet));
        assert(packet[1] == std::byte{'O'} && packet[2] == std::byte{'K'});
        result = code;
        state = AsyncServiceState::Idle;
        return true;
    }

    AsyncServiceState state {AsyncServiceState::Idle};
    std::optional<std::byte> request;
    bool cancel {false};
    bool wrote_reply {false};
    std::optional<AsyncServiceResultCode> result;
    std::array<std::byte, 4> packet {};
};

class Plant final : public IActuatorPlant {
public:
    PlantCapabilities capabilities() const noexcept override { return {}; }
    void apply_operations(const PlantOperations&) override {}

    std::uint64_t delta_time_usec() const noexcept override { return 1000; }
    RobotState read_state() const override
    {
        RobotState state;
        state.sample_time_usec = time;
        state.actuators.push_back({"joint", position, 0, 0, time});
        return state;
    }
    RobotState step(const std::vector<ActuatorCommand>& commands) override
    {
        assert(commands.size() <= 1);
        if (!commands.empty()) {
            position = commands.front().value;
        }
        time += delta_time_usec();
        return read_state();
    }
    RobotState reset(std::uint64_t time_usec) override
    {
        time = time_usec;
        position = 0;
        return read_state();
    }
    double position {0};
    std::uint64_t time {0};
};

class Manual final : public IController {
public:
    std::string_view id() const noexcept override { return "manual"; }
    std::string_view source_id() const noexcept override { return {}; }
    bool accepts(const IControllerInput&) const noexcept override { return false; }
    void reset(const RobotState&) override {}
    ControllerOutput update(std::shared_ptr<const IControllerInput>, const RobotState&,
                            const RuntimeStepContext& context) override
    {
        ControllerOutput output {"manual", {"manual", ComponentState::Ready, {}}, {}};
        if (enabled) {
            output.commands.push_back({"joint", ActuatorCommandType::Position,
                9.0, {}, context.simulation_time_usec, std::nullopt});
        }
        return output;
    }
    bool enabled {false};
};

void expect_load_error(const char* path, std::string_view message)
{
    try {
        (void)plugin::PluginLibrary::open(path, HAKO_PLUGIN_KIND_SERVICE);
        assert(false);
    } catch (const std::runtime_error& error) {
        const std::string_view text = error.what();
        assert(text.find(std::filesystem::path(path).filename().string()) != text.npos);
        assert(text.find(message) != text.npos);
    }
}

RuntimeServiceConfig parse_manifest(std::filesystem::path path, const char* wrong_kind)
{
    const auto base = path.parent_path();
    const auto fixture = std::filesystem::current_path();
    const auto runtime_path = fixture / "service-test-runtime.json";
    const auto pdu_path = fixture / "service-test-pdus.json";
    const auto actuator_path = fixture / "service-test-actuator.json";
    std::ofstream(runtime_path) << R"({"actuators":{"joint":{"command_timeout_sec":0.1}}})";
    std::ofstream(pdu_path) << R"({
        "paths":[{"id":"types","path":"service-test-types.json"}],
        "robots":[{"name":"robot","pdutypes_id":"types"}]
    })";
    std::ofstream(fixture / "service-test-types.json")
        << R"([{"name":"command","type":"std_msgs/Float64","pdu_size":32}])";
    std::ofstream(actuator_path) << R"({
        "spec":{"joint_name":"joint","type":"position","limit":{"lower":-10,"upper":10}},
        "mjcf_binding":{},
        "pdu_config":{"pdu_name":"command","message_type":"std_msgs/Float64","update_rate_hz":50}
    })";
    const auto actuator = nlohmann::json::array({{
        {"id", "joint"}, {"kind", "actuator"}, {"type", "joint_position_actuator"},
        {"config", actuator_path.string()}, {"pdu_robot", "robot"},
    }});
    const auto entries = [&](const std::string& library) {
        using nlohmann::json;
        return json::array({
            {{"name", "any/route"}, {"type", "plugin"}, {"pdu_robot", "robot"},
                {"priority", 16}, {"plugin", {{"library", library}, {"config", {{"mode", "test"}}}}}},
            {{"name", "CarService/AckermannStep/car"}, {"type", "ackermann_step"}, {"pdu_robot", "car"}},
            {{"name", "PersonService/PersonStep/person"}, {"type", "person_step"}, {"pdu_robot", "person"}},
        });
    };
    RuntimeFactoryInput input {base.string(), "test.xml", pdu_path.string(), runtime_path.string(),
        actuator.dump(), entries(path.filename().string()).dump()};
    RuntimeDefinition definition;
    std::string error;
    assert(resolve_runtime_definition(input, definition, &error));
    assert(definition.services.size() == 3);
    const auto config = definition.services.front();
    assert(config.type == RuntimeServiceType::Plugin && config.priority == 16);
    assert(config.plugin_library == path && config.plugin_config_json == "{\"mode\":\"test\"}");

    input.services_json = entries(std::filesystem::path(wrong_kind).filename().string()).dump();
    assert(resolve_runtime_definition(input, definition, &error));
    expect_load_error(definition.services.front().plugin_library.c_str(), "needs a service plugin");
    auto invalid = entries(path.filename().string());
    invalid[0]["priority"] = 2147483648LL;
    input.services_json = invalid.dump();
    assert(!resolve_runtime_definition(input, definition, &error));
    assert(error.find("priority") != error.npos);
    return config;
}

void test_pipeline(RuntimeServiceConfig config, bool lockstep)
{
    auto library = plugin::PluginLibrary::open(config.plugin_library, HAKO_PLUGIN_KIND_SERVICE);
    auto service = std::make_shared<ServicePlugin>(
        std::move(library), config.name, config.plugin_config_json,
        std::vector<std::string>{"joint"}, 1'000'000, 1, 2);
    auto server = std::make_shared<Server>();
    auto source = std::make_shared<AsyncServiceCommandSource>(config.name + "/0", 0, server, service, 0, lockstep);
    auto robot_source = std::make_shared<RobotServiceSource>(config.name, std::vector {source});
    auto controller = std::make_shared<RobotServiceController>(config.name + ":controller", robot_source, service);
    auto publisher = std::make_shared<AsyncServiceResponsePublisher>(config.name + ":reply", source);
    auto plant = std::make_shared<Plant>();
    auto manual = std::make_shared<Manual>();
    auto arbiter = std::make_shared<PriorityCommandArbiter>(std::vector<ControllerPriority>{
        {std::string(controller->id()), config.priority, {}}, {"manual", 20, {}}});
    ActuatorRuntime runtime({robot_source}, {controller, manual}, arbiter, plant, {publisher});
    (void)runtime.reset(0);
    const auto step = [&]() { (void)runtime.step(); };

    if (lockstep) {
        server->request = std::byte{0};
        assert(source->lockstep_ready());
        server->cancel = true;
        assert(!source->lockstep_ready());
        assert(server->result == AsyncServiceResultCode::Canceled && !server->wrote_reply);
        server->request = std::byte{0};
        assert(source->lockstep_ready());
        step(); // A staged cancel did not advance the fixture's request phase.
        assert(server->result == AsyncServiceResultCode::Ok);
        return;
    }

    step();
    assert(plant->position == 3.0 && !server->result);
    step();
    assert(plant->position == 4.0);
    manual->enabled = true;
    step();
    assert(plant->position == 9.0);
    manual->enabled = false;
    step();
    assert(plant->position == 3.0);
    server->request = std::byte{0};
    step();
    assert(plant->position == 7.0 && server->result == AsyncServiceResultCode::Ok);
    assert(server->packet[0] == std::byte{1} && server->packet[3] == std::byte{7});

    server->request = std::byte{1};
    step();
    assert(server->state == AsyncServiceState::Doing);
    server->cancel = true;
    step();
    assert(server->result == AsyncServiceResultCode::Canceled && server->packet[3] == std::byte{5});
    server->request = std::byte{2};
    step();
    assert(server->result == AsyncServiceResultCode::Invalid && !server->wrote_reply);
}

// A failing periodic update is reported, not dropped: Error alone, Degraded beside RPC commands.
void test_failing_update(RuntimeServiceConfig config, const char* path)
{
    auto library = plugin::PluginLibrary::open(path, HAKO_PLUGIN_KIND_SERVICE);
    auto service = std::make_shared<ServicePlugin>(
        std::move(library), config.name, config.plugin_config_json,
        std::vector<std::string>{"joint"}, 1'000'000, 1, 2);
    auto server = std::make_shared<Server>();
    auto source = std::make_shared<AsyncServiceCommandSource>(config.name + "/0", 0, server, service, 0);
    auto robot_source = std::make_shared<RobotServiceSource>(config.name, std::vector {source});
    auto controller = std::make_shared<RobotServiceController>(config.name + ":controller", robot_source, service);
    auto publisher = std::make_shared<AsyncServiceResponsePublisher>(config.name + ":reply", source);
    auto plant = std::make_shared<Plant>();
    auto manual = std::make_shared<Manual>();
    auto arbiter = std::make_shared<PriorityCommandArbiter>(std::vector<ControllerPriority>{
        {std::string(controller->id()), config.priority, {}}, {"manual", 20, {}}});
    ActuatorRuntime runtime({robot_source}, {controller, manual}, arbiter, plant, {publisher});
    (void)runtime.reset(0);

    auto report = runtime.step();
    assert(report.controller_statuses[0].state == ComponentState::Error);
    assert(report.controller_statuses[0].detail.find("fixture update failed") != std::string::npos);

    server->request = std::byte{0};
    report = runtime.step();
    assert(report.controller_statuses[0].state == ComponentState::Degraded);
    assert(report.controller_statuses[0].detail.find("fixture update failed") != std::string::npos);
    assert(plant->position == 7.0 && server->result == AsyncServiceResultCode::Ok);
}

} // namespace

int main(int argc, char** argv)
{
    assert(argc == 9);
    const auto config = parse_manifest(argv[1], argv[2]);
    expect_load_error(argv[3], "needs a service plugin");
    expect_load_error(argv[4], "service ABI version");
    expect_load_error(argv[5], "service table of");
    expect_load_error(argv[6], "on_step");
    const auto optional = plugin::PluginLibrary::open(argv[7], HAKO_PLUGIN_KIND_SERVICE);
    assert(!optional->service_table().update && !optional->service_table().on_cancel);
    test_pipeline(config, false);
    test_pipeline(config, true);
    test_failing_update(config, argv[8]);
}
