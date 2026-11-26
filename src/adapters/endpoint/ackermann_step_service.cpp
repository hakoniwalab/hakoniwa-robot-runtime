#include "adapters/endpoint/ackermann_step_service.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE

#include "adapters/endpoint/ackermann_step_packet.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepResponsePacket.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {
namespace {

std::optional<HakoPduMetaDataType> packet_metadata(
    std::span<const std::byte> packet, std::size_t base_size)
{
    if (packet.size() < sizeof(HakoPduMetaDataType)) {
        return std::nullopt;
    }
    HakoPduMetaDataType metadata {};
    std::memcpy(&metadata, packet.data(), sizeof(metadata));
    const bool valid = metadata.magicno == HAKO_PDU_META_DATA_MAGICNO
        && metadata.version == HAKO_PDU_META_DATA_VERSION
        && metadata.base_off == HAKO_PDU_META_DATA_SIZE()
        && metadata.heap_off >= metadata.base_off + base_size
        && metadata.heap_off <= metadata.total_size && metadata.total_size <= packet.size();
    return valid ? std::optional {metadata} : std::nullopt;
}

} // namespace

AckermannStepService::AckermannStepService(
    std::shared_ptr<AckermannDriveRpcEventReader> reader, std::string body_id)
    : reader_(std::move(reader)), body_id_(std::move(body_id))
{
    if (!reader_ || body_id_.empty()) {
        throw std::invalid_argument("AckermannStep requires a reader and a MultiDOF body ID");
    }
}

bool AckermannStepService::on_request(std::span<const std::byte> request)
{
    if (!packet_metadata(request, sizeof(Hako_AckermannStepRequestPacket))) {
        return false;
    }
    // Core owns the request header. Read the fixed-size Ackermann body after
    // validating the envelope, without decoding the header strings again.
    Hako_AckermannStepRequest wire {};
    std::memcpy(&wire, request.data() + HAKO_PDU_META_DATA_SIZE()
        + offsetof(Hako_AckermannStepRequestPacket, body), sizeof(wire));
    HakoCpp_AckermannStepRequest decoded {};
    cpp_pdu2cpp_AckermannStepRequest(nullptr, wire, decoded);
    reader_->apply({
        decoded.drive.steering_angle, decoded.drive.steering_angle_velocity,
        decoded.drive.speed, decoded.drive.acceleration, decoded.drive.jerk,
    });
    step_ = decoded.step;
    return true;
}

runtime::AsyncServiceStepResult AckermannStepService::on_step(
    const runtime::RobotState&, runtime::AsyncServiceState,
    const runtime::RuntimeStepContext&)
{
    return {runtime::AsyncServiceStepOutcome::Ok, {}};
}

bool AckermannStepService::on_response(
    const runtime::RobotState& state, const runtime::RuntimeStepContext&,
    std::span<std::byte> packet)
{
    const auto body = std::find_if(state.bodies.begin(), state.bodies.end(),
        [this](const auto& candidate) { return candidate.body_id == body_id_; });
    // Core initializes only the response header inside a buffer sized for the
    // full packet. A complete AckermannStep response has no dynamic heap.
    constexpr auto full_size = HAKO_PDU_META_DATA_SIZE()
        + HAKO_ALIGN_SIZE(sizeof(Hako_AckermannStepResponsePacket), HAKO_ALIGNMENT_SIZE);
    auto metadata = packet_metadata(packet, sizeof(Hako_ServiceResponseHeader));
    if (body == state.bodies.end() || !metadata || packet.size() < full_size) {
        return false;
    }
    auto response = ackermann_step_response_body(step_, {
        state.sample_time_usec, body->position, body->orientation,
        body->linear_velocity, body->angular_velocity,
    });
    Hako_AckermannStepResponse wire {};
    PduDynamicMemory memory;
    if (!cpp_cpp2pdu_AckermannStepResponse(response, wire, memory)) {
        return false;
    }
    if (metadata->heap_off < full_size) {
        metadata->heap_off = full_size;
        metadata->total_size = full_size;
        std::memcpy(packet.data(), &*metadata, sizeof(*metadata));
    }
    // Keep the Core service header byte for byte, including its request ID and result code.
    std::memcpy(packet.data() + HAKO_PDU_META_DATA_SIZE()
        + offsetof(Hako_AckermannStepResponsePacket, body), &wire, sizeof(wire));
    return true;
}

void AckermannStepService::reset()
{
    step_ = 0;
    reader_->reset();
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
