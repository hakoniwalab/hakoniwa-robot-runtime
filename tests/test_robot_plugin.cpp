// Stage 2c-1: the robot service plugin interface (C), loaded dynamically and
// as a builtin. Written before the implementation (notes 2026-11-27). The same
// scenario runs through both paths and must give the same results.
#undef NDEBUG

#include "plugin/plugin_library.hpp"
#include "runtime/actuator_runtime.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runtime/controller/hold_controller.hpp"
#include "runtime/service/async_service_pipeline.hpp"
#include "runtime/service/service_plugin_adapter.hpp"

#include <cassert>
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

extern "C" const HakoPluginHeaderV1* hako_robot_plugin_robot_fixture(void);

using namespace hakoniwa::robot_runtime;
using namespace hakoniwa::robot_runtime::runtime;

namespace {

constexpr int kMoveId = 50;
constexpr int kStopId = 51;
constexpr auto kJoint = "joint";
constexpr double kHold = 0.25;
constexpr std::uint64_t kDelta = 1'000;
constexpr std::size_t kMeta = 24;

/** A PDU with 24 bytes of metadata (base_off = 24) and `body` after it. */
std::vector<std::byte> packet(std::vector<std::byte> body)
{
    std::vector<std::byte> bytes(kMeta);
    const std::uint32_t base_off = kMeta;
    std::memcpy(bytes.data() + 8, &base_off, sizeof(base_off));
    bytes.insert(bytes.end(), body.begin(), body.end());
    return bytes;
}

std::vector<std::byte> request(signed char value)
{
    return packet({static_cast<std::byte>(value)});
}

class FakePlant final : public IActuatorPlant {
public:
    std::vector<PlantOperations> applied;
    std::vector<std::vector<ActuatorCommand>> stepped;

    std::uint64_t delta_time_usec() const noexcept override { return kDelta; }
    PlantCapabilities capabilities() const noexcept override { return {true, true, true}; }
    void apply_operations(const PlantOperations& operations) override { applied.push_back(operations); }

    RobotState read_state() const override
    {
        RobotState state;
        state.sample_time_usec = stepped.size() * kDelta;
        state.actuators.push_back({kJoint, kHold, 0.0, 0.0, state.sample_time_usec});
        BodyState base;
        base.body_id = "base";
        base.position = {0.3, 0.0, 0.0};
        state.bodies.push_back(base);
        HumanoidState human;
        human.name = "H";
        state.humanoids.push_back(human);
        return state;
    }

    RobotState step(const std::vector<ActuatorCommand>& commands) override
    {
        stepped.push_back(commands);
        return read_state();
    }

    RobotState reset(std::uint64_t) override
    {
        stepped.clear();
        return read_state();
    }
};

class FakeServer final : public IAsyncServiceServer {
public:
    struct Service {
        AsyncServiceState state {AsyncServiceState::Idle};
        std::optional<std::vector<std::byte>> request;
        bool cancel {false};
        std::vector<AsyncServiceResultCode> replies;
        std::vector<std::vector<std::byte>> bodies;  // reply bytes after the metadata
    };
    std::map<int, Service> services {{kMoveId, {}}, {kStopId, {}}};

    AsyncServiceEvent poll(int id) noexcept override
    {
        auto& service = services.at(id);
        if (std::exchange(service.cancel, false)) {
            service.state = AsyncServiceState::Canceling;
            return AsyncServiceEvent::Cancel;
        }
        if (service.request) {
            service.state = AsyncServiceState::Doing;
            return AsyncServiceEvent::Request;
        }
        return AsyncServiceEvent::None;
    }

    std::optional<AsyncServiceState> status(int id) noexcept override { return services.at(id).state; }

    bool get_request(int id, std::vector<std::byte>& bytes) override
    {
        bytes = std::move(*services.at(id).request);
        services.at(id).request.reset();
        return true;
    }

    bool reply(int id, AsyncServiceResultCode code, const AsyncServiceResponseWriter& write) override
    {
        auto& service = services.at(id);
        auto bytes = packet(std::vector<std::byte>(8, std::byte {0}));
        if (write && !write(bytes)) {
            return false;
        }
        service.replies.push_back(code);
        service.bodies.emplace_back(bytes.begin() + kMeta, bytes.end());
        service.state = AsyncServiceState::Idle;
        return true;
    }
};

ServicePluginConfig fixture_config()
{
    ServicePluginConfig config;
    config.robot = "R1";
    config.actuators = {{kJoint, ActuatorCommandType::Position, {-1.5, 1.5}}};
    config.bodies = {"base"};
    config.humanoids = {"H"};
    config.services = {{"svc/move", "t/Move", false}, {"svc/stop", "t/Stop", false}};
    config.config_json = R"({"mode":"test"})";
    config.physics_step_ns = 1'000'000;
    return config;
}

struct Fixture {
    explicit Fixture(std::shared_ptr<plugin::PluginLibrary> library)
        : robot(std::make_shared<ServicePlugin>(std::move(library), fixture_config()))
    {
        move = std::make_shared<AsyncServiceCommandSource>("R1/0", kMoveId, server, robot, 0);
        stop = std::make_shared<AsyncServiceCommandSource>("R1/1", kStopId, server, robot, 1);
        source = std::make_shared<RobotServiceSource>("R1",
            std::vector<std::shared_ptr<AsyncServiceCommandSource>> {move, stop});
        controller = std::make_shared<RobotServiceController>("R1:controller", source, robot);
        runtime.emplace(std::vector<std::shared_ptr<ICommandSource>> {source},
            std::vector<std::shared_ptr<IController>> {controller,
                std::make_shared<HoldController>("hold", std::vector<HoldActuatorBinding> {{kJoint}})},
            std::make_shared<PriorityCommandArbiter>(std::vector<ControllerPriority> {
                {"R1:controller", 10, {}}, {"hold", 0, {}},
            }),
            plant,
            std::vector<std::shared_ptr<IStatePublisher>> {
                std::make_shared<AsyncServiceResponsePublisher>("R1/0:response", move),
                std::make_shared<AsyncServiceResponsePublisher>("R1/1:response", stop),
            });
        (void)runtime->reset(0);
    }

    std::shared_ptr<FakePlant> plant = std::make_shared<FakePlant>();
    std::shared_ptr<FakeServer> server = std::make_shared<FakeServer>();
    std::shared_ptr<ServicePlugin> robot;
    std::shared_ptr<AsyncServiceCommandSource> move;
    std::shared_ptr<AsyncServiceCommandSource> stop;
    std::shared_ptr<RobotServiceSource> source;
    std::shared_ptr<RobotServiceController> controller;
    std::optional<ActuatorRuntime> runtime;

    FakeServer::Service& core(int id) { return server->services.at(id); }
    double applied_joint() const
    {
        assert(plant->stepped.back().size() == 1);
        return plant->stepped.back().front().value;
    }
};

/** What one scenario observed, for comparing the dynamic and the builtin path. */
struct Observed {
    std::vector<double> joints;
    std::vector<AsyncServiceResultCode> move_replies;
    std::vector<AsyncServiceResultCode> stop_replies;
    std::vector<std::vector<std::byte>> move_bodies;
    std::vector<std::vector<std::byte>> stop_bodies;
    std::size_t frictions {0};
    std::size_t humanoid_motions {0};

    bool operator==(const Observed&) const = default;
};

Observed run_scenario(std::shared_ptr<plugin::PluginLibrary> library)
{
    Fixture f(std::move(library));
    Observed seen;
    const auto step = [&]() {
        (void)f.runtime->step();
        seen.joints.push_back(f.applied_joint());
        seen.frictions += f.plant->applied.back().surface_frictions.size();
        seen.humanoid_motions += f.plant->applied.back().humanoid_motions.size();
    };

    // 1. No request: the robot steps anyway; its first step sets the tyre friction.
    step();
    assert(f.plant->applied.back().surface_frictions.size() == 1);
    assert(f.plant->applied.back().surface_frictions.front().surfaces == std::vector<std::string> {"tyre"});
    assert(f.plant->applied.back().surface_frictions.front().sliding_friction == 0.6);
    assert(f.applied_joint() == kHold);

    // 2. move 0.5: accepted in its first step, a humanoid motion goes out, the joint follows.
    f.core(kMoveId).request = request(5);
    step();
    assert(f.core(kMoveId).replies == std::vector {AsyncServiceResultCode::Ok});
    assert(f.core(kMoveId).bodies.back()[0] == std::byte {0x40});
    assert(f.core(kMoveId).bodies.back()[1] == std::byte {5});
    assert(f.core(kMoveId).bodies.back()[2] == std::byte {3});  // body x 0.3, read from the state
    assert(f.plant->applied.back().humanoid_motions.size() == 1);
    assert(f.plant->applied.back().humanoid_motions.front().humanoid == "H");
    assert(f.plant->applied.back().humanoid_motions.front().linear_velocity.x == 0.5);
    assert(f.plant->applied.back().humanoid_motions.front().animation == "walk");
    assert(f.applied_joint() == 0.5);

    // 3. Idle step: the motion outlives its reply.
    step();
    assert(f.applied_joint() == 0.5);

    // 4. stop on service 1: answered on its own Core id. The robot stops commanding, so the
    // hold controller takes over and keeps the last selected position (0.5), not the measured angle.
    f.core(kStopId).request = request(0);
    step();
    assert(f.core(kStopId).replies == std::vector {AsyncServiceResultCode::Ok});
    assert(f.core(kStopId).bodies.back()[0] == std::byte {0x41});
    assert(f.core(kMoveId).replies.size() == 1);
    assert(f.plant->stepped.back().size() == 1);
    assert(f.plant->stepped.back().front().source_id.empty());  // the hold controller's command
    assert(f.applied_joint() == 0.5);

    // 5. move 2.0 stays open; a cancel ends it as CANCELED.
    f.core(kMoveId).request = request(20);
    step();
    assert(f.core(kMoveId).replies.size() == 1);
    f.core(kMoveId).cancel = true;
    step();
    assert(f.core(kMoveId).replies.back() == AsyncServiceResultCode::Canceled);

    // 6. move 10.1 is refused: INVALID without a written body.
    f.core(kMoveId).request = request(101);
    step();
    assert(f.core(kMoveId).replies.back() == AsyncServiceResultCode::Invalid);

    seen.move_replies = f.core(kMoveId).replies;
    seen.stop_replies = f.core(kStopId).replies;
    seen.move_bodies = f.core(kMoveId).bodies;
    seen.stop_bodies = f.core(kStopId).bodies;
    return seen;
}

void expect_error(const std::function<void()>& action, std::string_view text)
{
    try {
        action();
    } catch (const std::runtime_error& error) {
        if (std::string_view(error.what()).find(text) == std::string_view::npos) {
            std::cerr << "unexpected error: " << error.what() << " (wanted: " << text << ")\n";
            assert(false);
        }
        return;
    }
    assert(false && "expected an error");
}

const plugin::BuiltinPlugin kBuiltins[] = {{"robot_fixture", hako_robot_plugin_robot_fixture}};

} // namespace

int main(int argc, char** argv)
{
    // argv: fixture, bad_kind, bad_abi, small_table, missing_step, no_cancel, failing_step
    assert(argc == 8);
    const std::string dynamic_path = argv[1];

    // The same definition, loaded as a shared library and as a builtin, behaves the same.
    const auto dynamic = run_scenario(plugin::PluginLibrary::open(dynamic_path, HAKO_PLUGIN_KIND_SERVICE));
    const auto builtin = run_scenario(
        plugin::PluginLibrary::open_builtin("robot_fixture", HAKO_PLUGIN_KIND_SERVICE, kBuiltins));
    assert(dynamic == builtin);
    assert(dynamic.frictions == 1 && dynamic.humanoid_motions == 1);

    // The builtin list is searched by name.
    expect_error([] {
        (void)plugin::PluginLibrary::open_builtin("nothing", HAKO_PLUGIN_KIND_SERVICE, kBuiltins);
    }, "no builtin plugin");

    // create gets the robot's own arguments; a wrong robot fails with the plugin's reason.
    expect_error([&] {
        auto config = fixture_config();
        config.robot = "R2";
        (void)ServicePlugin(plugin::PluginLibrary::open(dynamic_path, HAKO_PLUGIN_KIND_SERVICE), config);
    }, "fixture: unexpected create arguments");

    // The table checks are the same for every loaded table.
    expect_error([&] { (void)plugin::PluginLibrary::open(argv[2], HAKO_PLUGIN_KIND_SERVICE); }, "service plugin");
    expect_error([&] { (void)plugin::PluginLibrary::open(argv[3], HAKO_PLUGIN_KIND_SERVICE); }, "ABI version");
    expect_error([&] { (void)plugin::PluginLibrary::open(argv[4], HAKO_PLUGIN_KIND_SERVICE); }, "table of");
    expect_error([&] { (void)plugin::PluginLibrary::open(argv[5], HAKO_PLUGIN_KIND_SERVICE); }, "on_step");
    const auto no_cancel = plugin::PluginLibrary::open(argv[6], HAKO_PLUGIN_KIND_SERVICE);
    assert(no_cancel->service_table().on_cancel == nullptr);

    // A failing on_step is reported with its reason.
    Fixture failing(plugin::PluginLibrary::open(argv[7], HAKO_PLUGIN_KIND_SERVICE));
    const auto report = failing.runtime->step();
    assert(report.controller_statuses[0].state == ComponentState::Error);
    assert(report.controller_statuses[0].detail.find("fixture step failed") != std::string::npos);

    std::cout << "robot plugin: all passed" << std::endl;
    return 0;
}
