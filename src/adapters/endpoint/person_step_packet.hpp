#pragma once

#include "hakoniwa/robot_runtime/adapters/endpoint/person_step_rpc_event_reader.hpp"
#include "pdu_primitive_ctypes.h"

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

bool read_person_step_packet_metadata(std::span<const std::byte> packet,
    std::size_t base_size, HakoPduMetaDataType& metadata);

bool encode_person_step_response(
    std::string_view person_name, std::uint64_t step, const PersonStepState& state,
    std::span<std::byte> packet);

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
