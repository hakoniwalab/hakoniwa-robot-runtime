#undef NDEBUG

#include "adapters/endpoint/ackermann_step_service.hpp"
#include "adapters/endpoint/ackermann_step_packet.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runtime/controller/ackermann_controller.hpp"
#include "runtime/actuator_runtime.hpp"
#include "runner/lockstep_gate.hpp"
#include "runtime/service/async_service_pipeline.hpp"

#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepResponsePacket.hpp"

#include <array>
#include <cassert>
#include <cstring>
#include <iostream>
#include <utility>

namespace {

using namespace ::hakoniwa::robot_runtime;
using namespace ::hakoniwa::robot_runtime::runtime;
using namespace ::hakoniwa::robot_runtime::adapters::hakoniwa;

using Packet = std::array<std::byte, 1024>;
constexpr char kBody[] = "car-body";
constexpr char kService[] = "CarService/AckermannStep/A";
constexpr char kDriveSource[] = "ackermann-pdu:drive";
constexpr std::uint64_t kDelta = 500;
constexpr std::uint32_t kSubsteps = 4;

Packet request_packet(std::uint64_t step, const AckermannDriveCommand& command)
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
    std::uint64_t delta_time_usec() const noexcept override { return kDelta; }
    RobotState read_state() const override { return {.sample_time_usec = time_}; }
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
    std::shared_ptr<AckermannDriveRpcEventReader> reader =
        std::make_shared<AckermannDriveRpcEventReader>();
    std::shared_ptr<Server> server = std::make_shared<Server>();
    std::shared_ptr<AckermannStepService> service =
        std::make_shared<AckermannStepService>(reader, kBody);
    std::shared_ptr<AsyncServiceCommandSource> source =
        std::make_shared<AsyncServiceCommandSource>(kService, 0, server, service, true);
    std::shared_ptr<AsyncServiceController> controller =
        std::make_shared<AsyncServiceController>("service", source);
    std::shared_ptr<AsyncServiceResponsePublisher> publisher =
        std::make_shared<AsyncServiceResponsePublisher>("reply", source);
    runner::LockstepGate gate {{{kService, source}}, {}};
};

void same_response_bytes()
{
    ServicePipeline pipeline;
    const auto request = request_packet(41, {.steering_angle_rad = 0.1, .speed_m_s = 4.0});
    assert(pipeline.service->on_request(request));
    const auto step = pipeline.service->on_step({}, AsyncServiceState::Doing, {});
    assert(step.outcome == AsyncServiceStepOutcome::Ok && step.commands.empty());

    const AckermannStepState expected {12'345, {1.5, -2.0, 3.0},
        {0.1, 0.2, -0.3, 0.9}, {4.0, -5.0, 6.0}, {0.4, -0.5, 0.6}};
    RobotState state;
    state.sample_time_usec = expected.simulation_time_usec;
    state.bodies.push_back({.body_id = "other"});
    state.bodies.push_back({.body_id = kBody, .position = expected.position,
        .orientation = expected.orientation, .linear_velocity = expected.linear_velocity,
        .angular_velocity = expected.angular_velocity});
    auto reference = response_envelope();
    HakoCpp_AckermannStepResponsePacket reference_packet {};
    auto converter = hako::pdu::msgs::hako_srv_msgs::AckermannStepResponsePacket {};
    assert(converter.pdu2cpp(reinterpret_cast<char*>(reference.data()), reference_packet));
    reference_packet.body = ackermann_step_response_body(41, expected);
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
    assert(!pipeline.server->has_writer && !pipeline.reader->take_latest());
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
    // The generated Python converter writes 300 bytes after the metadata,
    // ending at the last field of drive, without C++ tail alignment.
    constexpr auto size = HAKO_PDU_META_DATA_SIZE() + 300;
    HakoPduMetaDataType metadata {};
    std::memcpy(&metadata, packet.data(), sizeof(metadata));
    metadata.heap_off = size;
    metadata.total_size = size;
    std::memcpy(packet.data(), &metadata, sizeof(metadata));

    assert(!pipeline.service->on_request(std::span(packet).first(size - 1)));
    assert(pipeline.service->on_request(std::span(packet).first(size)));
    const auto applied = pipeline.reader->take_latest();
    assert(applied);
    assert(encode_ackermann_command(*applied) == encode_ackermann_command(command));
}

void commands_match_from_first_substep()
{
    const RuntimeAckermannControllerConfig config {
        .component_id = "drive",
        .input_timeout_usec = 2500,
        .geometry = {2.0, 1.0, 0.3, 0.6, 50.0},
        .actuators = {"left-steer", "right-steer", "left-wheel", "right-wheel"},
    };
    const auto make_arbiter = [] {
        return std::make_shared<PriorityCommandArbiter>(
            std::vector<ControllerPriority> {{ "drive", 10, {} }});
    };
    auto legacy_plant = std::make_shared<Plant>();
    auto legacy_reader = std::make_shared<AckermannDriveRpcEventReader>();
    auto legacy_source = std::make_shared<AckermannDriveCommandSource>(
        kDriveSource, config.input_timeout_usec, legacy_reader);
    ActuatorRuntime legacy({legacy_source}, {std::make_shared<AckermannController>(config)},
        make_arbiter(), legacy_plant, {});

    ServicePipeline pipeline;
    auto service_plant = std::make_shared<Plant>();
    auto service_drive = std::make_shared<AckermannDriveCommandSource>(
        kDriveSource, config.input_timeout_usec, pipeline.reader);
    ActuatorRuntime service({pipeline.source, service_drive},
        {std::make_shared<AckermannController>(config), pipeline.controller},
        make_arbiter(), service_plant, {pipeline.publisher});

    const std::array<AckermannDriveCommand, 2> inputs {{
        {.steering_angle_rad = 0.25, .speed_m_s = 4.0},
        {.steering_angle_rad = -0.375, .speed_m_s = 7.0},
    }};
    for (std::size_t step = 0; step < inputs.size(); ++step) {
        legacy_reader->apply(inputs[step]);
        (void)legacy.step_hakoniwa(kSubsteps);
        pipeline.server->send(request_packet(step, inputs[step]));
        assert(pipeline.gate.ready_to_step());
        (void)service.step_hakoniwa(kSubsteps);
    }
    assert(legacy_plant->commands.size() == inputs.size() * kSubsteps);
    assert(legacy_plant->commands.size() == service_plant->commands.size());
    for (std::size_t substep = 0; substep < legacy_plant->commands.size(); ++substep) {
        const auto& expected = legacy_plant->commands.at(substep);
        const auto& actual = service_plant->commands.at(substep);
        assert(expected.size() == 4 && actual.size() == expected.size());
        for (std::size_t i = 0; i < expected.size(); ++i) {
            assert(expected[i].actuator_id == actual[i].actuator_id);
            assert(expected[i].type == actual[i].type);
            assert(expected[i].value == actual[i].value);
            assert(expected[i].source_id == actual[i].source_id);
            assert(expected[i].created_at_usec == actual[i].created_at_usec);
            assert(expected[i].expires_at_usec == actual[i].expires_at_usec);
        }
    }
}

} // namespace

int main()
{
    same_response_bytes();
    malformed_request_returns_invalid();
    accepts_request_without_cpp_tail_padding();
    commands_match_from_first_substep();
    std::cout << "AckermannStep service P1-1 through P1-3 passed\n";
}
