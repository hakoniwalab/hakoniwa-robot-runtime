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

PersonStepService::PersonStepService(std::string person)
    : person_(std::move(person))
{
    if (person_.empty()) {
        throw std::invalid_argument("PersonStep requires a person");
    }
}

bool PersonStepService::on_request(std::size_t, std::span<const std::byte> request)
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
    motion_ = runtime::HumanoidMotionOperation {
        person_,
        {wire.cmd_vel.linear.x, wire.cmd_vel.linear.y, wire.cmd_vel.linear.z},
        {wire.cmd_vel.angular.x, wire.cmd_vel.angular.y, wire.cmd_vel.angular.z},
        read_text(wire.animation), read_text(wire.ride),
    };
    step_ = wire.step;
    return true;
}

runtime::RobotStepResult PersonStepService::on_step(
    const runtime::RobotState&, const runtime::RuntimeStepContext&,
    std::span<runtime::RobotServiceSlot> slots)
{
    runtime::RobotStepResult result;
    if (slots[0].state != runtime::AsyncServiceState::Idle) {
        slots[0].result = runtime::AsyncServiceStepOutcome::Ok;
        if (motion_) {
            result.operations.humanoid_motions.push_back(std::move(*motion_));
            motion_.reset();
        }
    }
    return result;
}

bool PersonStepService::on_response(
    std::size_t, const runtime::RobotState& state, const runtime::RuntimeStepContext&, std::span<std::byte> packet)
{
    const auto person = std::ranges::find(state.humanoids, person_, &runtime::HumanoidState::name);
    if (person == state.humanoids.end()) {
        return false;
    }
    return encode_person_step_response(person_, step_, {
        person->last_motion_result,
        state.sample_time_usec,
        person->position,
        person->orientation,
        person->linear_velocity,
        person->angular_velocity,
        person->joint_positions,
    }, packet);
}

void PersonStepService::reset()
{
    step_ = 0;
    motion_.reset();
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
