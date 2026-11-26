#pragma once

#include "hakoniwa/robot_runtime/adapters/endpoint/ackermann_drive_rpc_event_reader.hpp"
#include "hako_srv_msgs/pdu_cpptype_AckermannStepResponse.hpp"

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

HakoCpp_AckermannStepResponse ackermann_step_response_body(
    std::uint64_t step, const AckermannStepState& state);

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
