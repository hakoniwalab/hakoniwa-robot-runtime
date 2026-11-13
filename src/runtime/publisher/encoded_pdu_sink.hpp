#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace hakoniwa::robot_runtime::runtime {

/**
 * Receives the encoded bytes of a PDU that a state or sensor writer has just
 * encoded for its Endpoint, so that an in-process consumer (a controller
 * plugin) gets the same bytes without a second encoder.
 *
 * Called from the simulation thread, inside the writer's send() (the
 * Runtime's publish stage); data is valid only during the call.
 */
class IEncodedPduSink {
public:
    virtual ~IEncodedPduSink() = default;
    /** A new sample: the encoded PDU and the time the value was measured. */
    virtual void store(std::span<const std::byte> data, std::uint64_t sample_time_usec) = 0;
    /** The writer was reset: forget the last sample. */
    virtual void clear() noexcept = 0;
};

} // namespace hakoniwa::robot_runtime::runtime
