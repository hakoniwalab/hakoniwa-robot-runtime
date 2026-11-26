// Stage 2c-4: the C++ template layer. A robot written as a C++ class receives
// and answers HakoCpp types; the layer decodes and encodes the PDU packets with
// the generated converters and keeps Core's response header. Written before
// the implementation. The packets here are built with the same generated
// converters, independently of the plugin.
#undef NDEBUG

#include "plugin/plugin_library.hpp"
#include "runtime/actuator_runtime.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runtime/service/async_service_pipeline.hpp"
#include "runtime/service/service_plugin_adapter.hpp"

#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsResponsePacket.hpp"

#include <cassert>
#include <functional>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace hakoniwa::robot_runtime;
using namespace hakoniwa::robot_runtime::runtime;

namespace {

constexpr int kAddId = 60;
constexpr auto kJoint = "joint";
constexpr std::uint64_t kDelta = 1'000;
constexpr int kRequestSize = 288;   // hako_srv_msgs/AddTwoIntsRequestPacket pdu_size
constexpr int kResponseSize = 280;  // hako_srv_msgs/AddTwoIntsResponsePacket pdu_size

std::vector<std::byte> add_request(std::uint32_t request_id, long long a, long long b)
{
    HakoCpp_AddTwoIntsRequestPacket packet {};
    packet.header.request_id = request_id;
    packet.header.service_name = "Robot/Add/R1";
    packet.header.client_name = "client";
    packet.body.a = a;
    packet.body.b = b;
    std::vector<std::byte> bytes(kRequestSize);
    hako::pdu::msgs::hako_srv_msgs::AddTwoIntsRequestPacket converter;
    const int size = converter.cpp2pdu(packet, reinterpret_cast<char*>(bytes.data()), kRequestSize);
    assert(size > 0);
    return bytes;
}

/** What Core prepares: the response header only, in a buffer sized for the packet. */
std::vector<std::byte> prepared_response(std::uint32_t request_id)
{
    HakoCpp_AddTwoIntsResponsePacket packet {};
    packet.header.request_id = request_id;
    packet.header.service_name = "Robot/Add/R1";
    packet.header.client_name = "client";
    packet.header.status = 2;
    packet.header.result_code = 0;
    std::vector<std::byte> bytes(kResponseSize);
    hako::pdu::msgs::hako_srv_msgs::AddTwoIntsResponsePacket converter;
    assert(converter.cpp2pdu(packet, reinterpret_cast<char*>(bytes.data()), kResponseSize) > 0);
    return bytes;
}

HakoCpp_AddTwoIntsResponsePacket decode_response(std::vector<std::byte>& bytes)
{
    HakoCpp_AddTwoIntsResponsePacket packet {};
    hako::pdu::msgs::hako_srv_msgs::AddTwoIntsResponsePacket converter;
    assert(converter.pdu2cpp(reinterpret_cast<char*>(bytes.data()), packet));
    return packet;
}

/** The joint follows the last applied position command. */
class FakePlant final : public IActuatorPlant {
public:
    double position {0.0};
    std::vector<std::vector<ActuatorCommand>> stepped;

    std::uint64_t delta_time_usec() const noexcept override { return kDelta; }
    PlantCapabilities capabilities() const noexcept override { return {true, true, true}; }
    void apply_operations(const PlantOperations&) override {}

    RobotState read_state() const override
    {
        RobotState state;
        state.sample_time_usec = stepped.size() * kDelta;
        state.actuators.push_back({kJoint, position, 0.0, 0.0, state.sample_time_usec});
        return state;
    }

    RobotState step(const std::vector<ActuatorCommand>& commands) override
    {
        stepped.push_back(commands);
        for (const auto& command : commands) {
            position = command.value;
        }
        return read_state();
    }

    RobotState reset(std::uint64_t) override
    {
        stepped.clear();
        position = 0.0;
        return read_state();
    }
};

class FakeServer final : public IAsyncServiceServer {
public:
    AsyncServiceState state {AsyncServiceState::Idle};
    std::optional<std::vector<std::byte>> request;
    std::uint32_t request_id {0};
    std::vector<AsyncServiceResultCode> replies;
    std::vector<std::vector<std::byte>> packets;

    AsyncServiceEvent poll(int) noexcept override
    {
        if (request) {
            state = AsyncServiceState::Doing;
            return AsyncServiceEvent::Request;
        }
        return AsyncServiceEvent::None;
    }
    std::optional<AsyncServiceState> status(int) noexcept override { return state; }
    bool get_request(int, std::vector<std::byte>& bytes) override
    {
        bytes = std::move(*request);
        request.reset();
        return true;
    }
    bool reply(int, AsyncServiceResultCode code, const AsyncServiceResponseWriter& write) override
    {
        auto bytes = prepared_response(request_id);
        if (write && !write(bytes)) {
            return false;
        }
        replies.push_back(code);
        packets.push_back(std::move(bytes));
        state = AsyncServiceState::Idle;
        return true;
    }
};

ServicePluginConfig adder_config()
{
    ServicePluginConfig config;
    config.robot = "R1";
    config.actuators = {{kJoint, ActuatorCommandType::Position, {-10.0, 10.0}}};
    config.services = {{"Robot/Add/R1", "hako_srv_msgs/AddTwoInts", false}};
    config.physics_step_ns = 1'000'000;
    return config;
}

struct Fixture {
    explicit Fixture(const std::string& path)
        : robot(std::make_shared<ServicePlugin>(
            plugin::PluginLibrary::open(path, HAKO_PLUGIN_KIND_SERVICE), adder_config()))
    {
        service = std::make_shared<AsyncServiceCommandSource>("Robot/Add/R1", kAddId, server, robot, 0);
        source = std::make_shared<RobotServiceSource>("R1",
            std::vector<std::shared_ptr<AsyncServiceCommandSource>> {service});
        controller = std::make_shared<RobotServiceController>("R1:controller", source, robot);
        runtime.emplace(std::vector<std::shared_ptr<ICommandSource>> {source},
            std::vector<std::shared_ptr<IController>> {controller},
            std::make_shared<PriorityCommandArbiter>(std::vector<ControllerPriority> {{"R1:controller", 10, {}}}),
            plant,
            std::vector<std::shared_ptr<IStatePublisher>> {
                std::make_shared<AsyncServiceResponsePublisher>("Robot/Add/R1:response", service)});
        (void)runtime->reset(0);
    }

    std::shared_ptr<FakePlant> plant = std::make_shared<FakePlant>();
    std::shared_ptr<FakeServer> server = std::make_shared<FakeServer>();
    std::shared_ptr<ServicePlugin> robot;
    std::shared_ptr<AsyncServiceCommandSource> service;
    std::shared_ptr<RobotServiceSource> source;
    std::shared_ptr<RobotServiceController> controller;
    std::optional<ActuatorRuntime> runtime;
};

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

// The robot class gets the HakoCpp request body and answers with a HakoCpp body.
void a_cpp_robot_answers_with_hakocpp_types(const std::string& path)
{
    Fixture f(path);
    f.server->request_id = 7;
    f.server->request = add_request(7, 2, 3);
    const auto report = f.runtime->step();
    assert(report.controller_statuses[0].state == ComponentState::Ready);
    assert(f.server->replies == std::vector {AsyncServiceResultCode::Ok});
    // The command (a + b) / 10 reached the Plant through the robot's step().
    assert(f.plant->stepped.back().size() == 1 && f.plant->stepped.back().front().value == 0.5);
    // The reply body was encoded by the layer; Core's header is kept.
    auto reply = decode_response(f.server->packets.back());
    assert(reply.body.sum == 505);  // sum * 100 + post-physics position * 10
    assert(reply.header.request_id == 7);
    assert(reply.header.service_name == "Robot/Add/R1");
    assert(reply.header.client_name == "client");
    // The motion outlives the reply.
    (void)f.runtime->step();
    assert(f.plant->stepped.back().front().value == 0.5);
}

// A request the robot refuses is answered INVALID without a body.
void a_refused_request_is_invalid(const std::string& path)
{
    Fixture f(path);
    f.server->request_id = 8;
    f.server->request = add_request(8, -1, 3);
    (void)f.runtime->step();
    assert(f.server->replies == std::vector {AsyncServiceResultCode::Invalid});
}

// Binding a service type the robot does not have fails create with the types named.
void binding_the_wrong_type_fails_create(const std::string& path)
{
    expect_error([&] { (void)Fixture(path); }, "hako_srv_msgs/GetWorldTime");
}

// An exception from step() is reported as the robot's step failure.
void an_exception_in_step_is_reported(const std::string& path)
{
    Fixture f(path);
    const auto report = f.runtime->step();
    assert(report.controller_statuses[0].state == ComponentState::Error);
    assert(report.controller_statuses[0].detail.find("adder step failed") != std::string::npos);
}

// An exception from the constructor fails create with its message.
void an_exception_in_the_constructor_fails_create(const std::string& path)
{
    auto config = adder_config();
    config.actuators.push_back({"other", ActuatorCommandType::Position, {-1.0, 1.0}});
    expect_error([&] {
        (void)ServicePlugin(plugin::PluginLibrary::open(path, HAKO_PLUGIN_KIND_SERVICE), config);
    }, "adder needs exactly one actuator");
}

} // namespace

int main(int argc, char** argv)
{
    // argv: adder, adder_wrong_type, adder_throwing_step
    assert(argc == 4);
    a_cpp_robot_answers_with_hakocpp_types(argv[1]);
    a_refused_request_is_invalid(argv[1]);
    binding_the_wrong_type_fails_create(argv[2]);
    an_exception_in_step_is_reported(argv[3]);
    an_exception_in_the_constructor_fails_create(argv[1]);
    std::cout << "robot plugin template: all passed" << std::endl;
    return 0;
}
