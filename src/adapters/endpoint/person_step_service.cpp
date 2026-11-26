#include "adapters/endpoint/person_step_service.hpp"
#include "adapters/endpoint/person_step_packet.hpp"

#include "hako_srv_msgs/pdu_ctype_PersonStepRequestPacket.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {
namespace {

std::string read_text(const char (&text)[HAKO_STRING_SIZE])
{
    return {text, std::find(text, text + HAKO_STRING_SIZE, '\0')};
}

} // namespace

PersonStepService::PersonStepService(std::string person, ApplyCommand apply_command,
    ReadState read_state, PublishOutputs publish_outputs)
    : person_(std::move(person)),
      apply_command_(std::move(apply_command)),
      read_state_(std::move(read_state)),
      publish_outputs_(std::move(publish_outputs))
{
    if (person_.empty() || !apply_command_ || !read_state_) {
        throw std::invalid_argument("PersonStep requires a person and plant bindings");
    }
}

bool PersonStepService::on_request(std::span<const std::byte> request)
{
    constexpr auto body_offset = offsetof(Hako_PersonStepRequestPacket, body);
    // Python stops at the last wire field instead of rounding the packet up.
    constexpr auto body_size = offsetof(Hako_PersonStepRequest, ride)
        + sizeof(Hako_PersonStepRequest::ride);
    HakoPduMetaDataType metadata {};
    if (!read_person_step_packet_metadata(request, body_offset + body_size, metadata)) {
        return false;
    }
    Hako_PersonStepRequest wire {};
    std::memcpy(&wire, request.data() + metadata.base_off + body_offset, body_size);
    result_code_ = apply_command_({
        {wire.cmd_vel.linear.x, wire.cmd_vel.linear.y, wire.cmd_vel.linear.z},
        {wire.cmd_vel.angular.x, wire.cmd_vel.angular.y, wire.cmd_vel.angular.z},
        read_text(wire.animation), read_text(wire.ride),
    });
    step_ = wire.step;
    return true;
}

runtime::AsyncServiceStepResult PersonStepService::on_step(
    const runtime::RobotState&, runtime::AsyncServiceState,
    const runtime::RuntimeStepContext&)
{
    return {runtime::AsyncServiceStepOutcome::Ok, {}};
}

bool PersonStepService::on_response(
    const runtime::RobotState&, const runtime::RuntimeStepContext&, std::span<std::byte> packet)
{
    if (!encode_person_step_response(person_, step_, read_state_(result_code_), packet)) {
        return false;
    }
    if (publish_outputs_) {
        publish_outputs_();
    }
    return true;
}

void PersonStepService::reset()
{
    step_ = 0;
    result_code_ = 0;
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
