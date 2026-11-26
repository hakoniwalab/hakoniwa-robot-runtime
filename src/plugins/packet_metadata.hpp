#pragma once

#include "pdu_primitive_ctypes.h"

#include <cstddef>
#include <cstring>
#include <optional>
#include <span>

namespace hakoniwa::robot_runtime::plugins {

inline std::optional<HakoPduMetaDataType> packet_metadata(
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

} // namespace hakoniwa::robot_runtime::plugins
