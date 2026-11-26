#undef NDEBUG

#include "support/builtin_step_plugins.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/ackermann_drive_rpc_event_reader.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runtime/actuator_runtime.hpp"
#include "runner/lockstep_gate.hpp"
#include "runtime/service/async_service_pipeline.hpp"
#include "support/single_robot_service_calls.hpp"

#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepResponsePacket.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <utility>

namespace {

using namespace ::hakoniwa::robot_runtime;
using namespace ::hakoniwa::robot_runtime::runtime;
using namespace ::hakoniwa::robot_runtime::adapters::hakoniwa;

using plugins::ackermann_step::AckermannDriveCommand;
using Packet = std::array<std::byte, 1024>;
constexpr char kBody[] = "car-body";
constexpr char kService[] = "CarService/AckermannStep/A";
constexpr char kDriveSource[] = "ackermann-pdu:drive";
constexpr std::uint64_t kDelta = 500;
constexpr std::uint32_t kSubsteps = 4;
const RuntimeAckermannControllerConfig kDrive {
    .component_id = "drive",
    .pdu_robot = "A",
    .pdu_name = "ackermann_cmd",
    .input_timeout_usec = 2500,
    .geometry = {2.0, 1.0, 0.3, 0.6, 50.0},
    .actuators = {"left-steer", "right-steer", "left-wheel", "right-wheel"},
};

RobotState car_state(std::uint64_t time = 0)
{
    RobotState state {.sample_time_usec = time};
    state.bodies.push_back({.body_id = kBody});
    for (const auto& actuator : test_support::ackermann_actuators(kDrive.actuators)) {
        state.actuators.push_back({.actuator_id = actuator.component_id});
    }
    return state;
}

Packet request_packet(std::uint64_t step, const AckermannDriveCommand& command, double friction = 0.0)
{
    Packet packet {};
    HakoCpp_AckermannStepRequestPacket message {};
    message.header.request_id = 19;
    message.header.service_name = kService;
    message.header.client_name = "A-client";
    message.body.step = step;
    message.body.drive.steering_angle = command.steering_angle_rad;
    message.body.drive.steering_angle_velocity = command.steering_angle_velocity_rad_s;
    message.body.drive.speed = command.speed_m_s;
    message.body.drive.acceleration = command.acceleration_m_s2;
    message.body.drive.jerk = command.jerk_m_s3;
    message.body.tire_friction = friction;
    assert(hako::pdu::msgs::hako_srv_msgs::AckermannStepRequestPacket {}.cpp2pdu(
        message, reinterpret_cast<char*>(packet.data()), packet.size()) > 0);
    return packet;
}

Packet response_envelope()
{
    Packet packet {};
    HakoCpp_AckermannStepResponsePacket message {};
    message.header.request_id = 19;
    message.header.service_name = kService;
    message.header.client_name = "A-client";
    message.header.status = 2;
    message.header.result_code = 3;
    message.body.step = 99;
    assert(hako::pdu::msgs::hako_srv_msgs::AckermannStepResponsePacket {}.cpp2pdu(
        message, reinterpret_cast<char*>(packet.data()), packet.size()) > 0);
    return packet;
}

class Plant final : public IActuatorPlant {
public:
    std::vector<std::vector<ActuatorCommand>> commands;
    PlantCapabilities capabilities() const noexcept override { return {}; }
    void apply_operations(const PlantOperations&) override {}

    std::uint64_t delta_time_usec() const noexcept override { return kDelta; }
    RobotState read_state() const override { return car_state(time_); }
    RobotState step(const std::vector<ActuatorCommand>& applied) override
    {
        commands.push_back(applied);
        time_ += kDelta;
        return read_state();
    }
    RobotState reset(std::uint64_t time) override
    {
        time_ = time;
        return read_state();
    }
private:
    std::uint64_t time_ {0};
};

class Server final : public IAsyncServiceServer {
public:
    void send(std::span<const std::byte> bytes) { request_.assign(bytes.begin(), bytes.end()); }
    AsyncServiceEvent poll(int) noexcept override
    {
        if (!request_.empty()) {
            state_ = AsyncServiceState::Doing;
            return AsyncServiceEvent::Request;
        }
        return AsyncServiceEvent::None;
    }
    std::optional<AsyncServiceState> status(int) noexcept override { return state_; }
    bool get_request(int, std::vector<std::byte>& out) override
    {
        out = std::exchange(request_, {});
        return true;
    }
    bool reply(int, AsyncServiceResultCode code, const AsyncServiceResponseWriter& write) override
    {
        result = code;
        has_writer = static_cast<bool>(write);
        state_ = AsyncServiceState::Idle;
        return true;
    }
    std::optional<AsyncServiceResultCode> result;
    bool has_writer {false};
private:
    AsyncServiceState state_ {AsyncServiceState::Idle};
    std::vector<std::byte> request_;
};

struct ServicePipeline {
    std::shared_ptr<Server> server = std::make_shared<Server>();
    std::shared_ptr<ServicePlugin> robot = test_support::make_ackermann_plugin(kDrive, kBody, kService, kDelta);
    std::unique_ptr<test_support::SingleRobotServiceCalls> service =
        std::make_unique<test_support::SingleRobotServiceCalls>(*robot);
    std::shared_ptr<AsyncServiceCommandSource> service_source =
        std::make_shared<AsyncServiceCommandSource>(std::string(kService) + "/0", 0, server, robot, 0, true);
    std::shared_ptr<RobotServiceSource> source =
        std::make_shared<RobotServiceSource>(kDriveSource, std::vector {service_source});
    std::shared_ptr<RobotServiceController> controller =
        std::make_shared<RobotServiceController>(kDrive.component_id, source, robot);
    std::shared_ptr<AsyncServiceResponsePublisher> publisher =
        std::make_shared<AsyncServiceResponsePublisher>("reply", service_source);
    runner::LockstepGate gate {{{kService, service_source}}, {}};
};

void same_response_bytes()
{
    ServicePipeline pipeline;
    const auto request = request_packet(41, {.steering_angle_rad = 0.1, .speed_m_s = 4.0});
    assert(pipeline.service->on_request(request));
    const auto step = pipeline.service->on_step(car_state(), AsyncServiceState::Doing, {});
    assert(step.outcome == AsyncServiceStepOutcome::Ok && step.commands.size() == 4);
    const double radius = 2.0 / std::tan(static_cast<double>(0.1F));
    const double left_radius = radius - 0.5;
    const double right_radius = radius + 0.5;
    const double wheel_velocity = 4.0 / 0.3;
    const std::array expected_values {
        std::atan(2.0 / left_radius), std::atan(2.0 / right_radius),
        wheel_velocity * left_radius / radius, wheel_velocity * right_radius / radius,
    };
    const std::array<std::string_view, 4> expected_ids {"left-steer", "right-steer", "left-wheel", "right-wheel"};
    for (std::size_t i = 0; i < step.commands.size(); ++i) {
        assert(step.commands[i].actuator_id == expected_ids[i]);
        assert(step.commands[i].type == (i < 2 ? ActuatorCommandType::Position : ActuatorCommandType::Velocity));
        assert(std::abs(step.commands[i].value - expected_values[i]) < 1e-12);
    }

    const AckermannStepState expected {12'345, {1.5, -2.0, 3.0},
        {0.1, 0.2, -0.3, 0.9}, {4.0, -5.0, 6.0}, {0.4, -0.5, 0.6}};
    auto state = car_state(expected.simulation_time_usec);
    state.bodies.clear();
    state.bodies.push_back({.body_id = "other"});
    state.bodies.push_back({.body_id = kBody, .position = expected.position,
        .orientation = expected.orientation, .linear_velocity = expected.linear_velocity,
        .angular_velocity = expected.angular_velocity});
    auto reference = response_envelope();
    HakoCpp_AckermannStepResponsePacket reference_packet {};
    auto converter = hako::pdu::msgs::hako_srv_msgs::AckermannStepResponsePacket {};
    assert(converter.pdu2cpp(reinterpret_cast<char*>(reference.data()), reference_packet));
    auto& response = reference_packet.body;
    response.step = 41;
    response.result_code = 0;
    response.sim_time_usec = expected.simulation_time_usec;
    response.pose.translation = {expected.position.x, expected.position.y, expected.position.z};
    response.pose.rotation = {expected.orientation.x, expected.orientation.y, expected.orientation.z,
        expected.orientation.w};
    response.twist.linear = {expected.linear_velocity.x, expected.linear_velocity.y, expected.linear_velocity.z};
    response.twist.angular = {expected.angular_velocity.x, expected.angular_velocity.y, expected.angular_velocity.z};
    assert(converter.cpp2pdu(reference_packet, reinterpret_cast<char*>(reference.data()),
        reference.size()) > 0);
    auto service = response_envelope();
    assert(pipeline.service->on_response(state, {}, service));
    assert(reference == service);
    HakoCpp_AckermannStepResponsePacket envelope {};
    assert(hako::pdu::msgs::hako_srv_msgs::AckermannStepResponsePacket {}.pdu2cpp(
        reinterpret_cast<char*>(service.data()), envelope));
    Packet core_buffer {};
    assert(hako::pdu::msgs::hako_srv_msgs::ServiceResponseHeader {}.cpp2pdu(
        envelope.header, reinterpret_cast<char*>(core_buffer.data()), core_buffer.size()) > 0);
    assert(pipeline.service->on_response(state, {}, core_buffer));
    assert(reference == core_buffer);
    constexpr auto body_offset = HAKO_PDU_META_DATA_SIZE()
        + offsetof(Hako_AckermannStepResponsePacket, body);
    auto with_metadata = response_envelope();
    with_metadata[offsetof(HakoPduMetaDataType, epoch)] = std::byte {7};
    const auto original = with_metadata;
    assert(pipeline.service->on_response(state, {}, with_metadata));
    assert(std::memcmp(with_metadata.data(), original.data(), body_offset) == 0);
}

void malformed_request_returns_invalid()
{
    ServicePipeline pipeline;
    const auto request = request_packet(0, {.speed_m_s = 5.0});
    assert(!pipeline.service->on_request(std::span(request).first(1)));
    auto bad_offset = request;
    HakoPduMetaDataType metadata {};
    std::memcpy(&metadata, bad_offset.data(), sizeof(metadata));
    metadata.base_off = 5000;
    std::memcpy(bad_offset.data(), &metadata, sizeof(metadata));
    pipeline.server->send(bad_offset);
    assert(pipeline.gate.ready_to_step());
    ActuatorRuntime runtime({pipeline.source}, {pipeline.controller},
        std::make_shared<PriorityCommandArbiter>(std::vector<ControllerPriority> {}),
        std::make_shared<Plant>(), {pipeline.publisher});
    (void)runtime.step_hakoniwa(1);
    assert(pipeline.server->result == AsyncServiceResultCode::Invalid);
    assert(!pipeline.server->has_writer && !plugins::ackermann_step::decode_request(bad_offset));
}

void rejects_friction_without_a_bound_vehicle_input()
{
    ServicePipeline pipeline;
    assert(!pipeline.service->on_request(request_packet(0, {}, 0.5)));
    RobotServiceSlot slot;
    const auto step = pipeline.robot->on_step(car_state(), {}, std::span(&slot, 1));
    assert(step.commands.empty() && step.operations.surface_frictions.empty());
}

void accepts_request_without_cpp_tail_padding()
{
    ServicePipeline pipeline;
    const AckermannDriveCommand command {
        .steering_angle_rad = 0.25,
        .steering_angle_velocity_rad_s = 0.5,
        .speed_m_s = 5.0,
        .acceleration_m_s2 = 2.0,
        .jerk_m_s3 = 1.5,
    };
    auto packet = request_packet(41, command);
    // The new unpadded Python packet ends at tire_friction.
    constexpr auto header = HAKO_PDU_META_DATA_SIZE() + offsetof(Hako_AckermannStepRequestPacket, body);
    constexpr auto old_end = header + offsetof(Hako_AckermannStepRequest, tire_friction);
    constexpr auto size = old_end + sizeof(Hako_AckermannStepRequest::tire_friction);
    HakoPduMetaDataType metadata {};
    std::memcpy(&metadata, packet.data(), sizeof(metadata));
    for (const auto legacy_size : {old_end,
             HAKO_PDU_META_DATA_SIZE()
                 + HAKO_ALIGN_SIZE(old_end - HAKO_PDU_META_DATA_SIZE(), HAKO_ALIGNMENT_SIZE)}) {
        metadata.heap_off = metadata.total_size = legacy_size;
        std::memcpy(packet.data(), &metadata, sizeof(metadata));
        assert(!pipeline.service->on_request(std::span(packet).first(legacy_size)));
    }
    metadata.heap_off = size;
    metadata.total_size = size;
    std::memcpy(packet.data(), &metadata, sizeof(metadata));

    assert(!pipeline.service->on_request(std::span(packet).first(size - 1)));
    assert(pipeline.service->on_request(std::span(packet).first(size)));
    const auto decoded = plugins::ackermann_step::decode_request(std::span(packet).first(size));
    assert(decoded && decoded->step == 41 && decoded->tire_friction == 0.0);
    assert(decoded->drive.steering_angle_rad == command.steering_angle_rad);
    assert(decoded->drive.steering_angle_velocity_rad_s == command.steering_angle_velocity_rad_s);
    assert(decoded->drive.speed_m_s == command.speed_m_s);
    assert(decoded->drive.acceleration_m_s2 == command.acceleration_m_s2);
    assert(decoded->drive.jerk_m_s3 == command.jerk_m_s3);
}

void friction_is_staged_only_for_positive_values()
{
    const auto robot = test_support::make_ackermann_plugin(kDrive, kBody, kService, kDelta, {"tire"});
    test_support::SingleRobotServiceCalls service(*robot);
    const auto friction = [&]() -> std::optional<double> {
        RobotServiceSlot slot {AsyncServiceState::Doing};
        const auto step = robot->on_step(car_state(), {}, std::span(&slot, 1));
        assert(slot.result == AsyncServiceStepOutcome::Ok);
        const auto& operations = step.operations.surface_frictions;
        if (operations.empty()) {
            return std::nullopt;
        }
        assert(operations.size() == 1);
        return operations.front().sliding_friction;
    };
    assert(service.on_request(request_packet(0, {}, 0.5)));
    assert(friction() == 0.5);
    assert(service.on_request(request_packet(1, {}, 0.0)));
    assert(!friction());
    assert(service.on_request(request_packet(2, {}, -0.1)));
    assert(!friction());
    assert(service.on_request(request_packet(3, {}, 0.9)));
    assert(friction() == 0.9);
    std::cout << "F1-F3: friction staged as 0.5, no update at zero or negative, then 0.9\n";
}

void commands_match_from_first_substep()
{
    const auto make_arbiter = [] {
        return std::make_shared<PriorityCommandArbiter>(
            std::vector<ControllerPriority> {{kDrive.component_id, 10, ackermann_control_group}});
    };
    auto topic_plant = std::make_shared<Plant>();
    auto topic_reader = std::make_shared<test_support::AckermannTopicReader>();
    auto topic_car = test_support::make_topic_car(
        kDrive, test_support::ackermann_actuators(kDrive.actuators), topic_reader, kDelta);
    ActuatorRuntime topic({topic_car.source}, {topic_car.controller}, make_arbiter(), topic_plant, {});

    ServicePipeline pipeline;
    auto service_plant = std::make_shared<Plant>();
    ActuatorRuntime service({pipeline.source}, {pipeline.controller},
        make_arbiter(), service_plant, {pipeline.publisher});

    const std::array<AckermannDriveCommand, 2> inputs {{
        {.steering_angle_rad = 0.25, .speed_m_s = 4.0},
        {.steering_angle_rad = -0.375, .speed_m_s = 7.0},
    }};
    for (std::size_t step = 0; step < inputs.size(); ++step) {
        topic_reader->send(inputs[step]);
        (void)topic.step_hakoniwa(kSubsteps);
        pipeline.server->send(request_packet(step, inputs[step]));
        assert(pipeline.gate.ready_to_step());
        (void)service.step_hakoniwa(kSubsteps);
    }
    assert(topic_plant->commands.size() == inputs.size() * kSubsteps);
    assert(topic_plant->commands.size() == service_plant->commands.size());
    for (std::size_t substep = 0; substep < topic_plant->commands.size(); ++substep) {
        const auto& expected = topic_plant->commands.at(substep);
        const auto& actual = service_plant->commands.at(substep);
        assert(expected.size() == 4 && actual.size() == expected.size());
        for (std::size_t i = 0; i < expected.size(); ++i) {
            assert(expected[i].actuator_id == actual[i].actuator_id);
            assert(expected[i].type == actual[i].type);
            assert(expected[i].value == actual[i].value);
        }
    }

    (void)topic.step();
    (void)service.step();
    assert(topic_plant->commands.back().size() == 4 && service_plant->commands.back().size() == 4);
    (void)topic.step();
    (void)service.step();
    assert(topic_plant->commands.back().empty());
    assert(service_plant->commands.back().size() == 4);
}

} // namespace

int main()
{
    same_response_bytes();
    malformed_request_returns_invalid();
    rejects_friction_without_a_bound_vehicle_input();
    accepts_request_without_cpp_tail_padding();
    friction_is_staged_only_for_positive_values();
    commands_match_from_first_substep();
    std::cout << "AckermannStep service P1-1 through P1-3 passed\n";
}
