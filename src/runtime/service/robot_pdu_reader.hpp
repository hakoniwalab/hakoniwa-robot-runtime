#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

struct RobotPduSample {
    std::vector<std::byte> bytes;
    std::uint64_t sample_time_usec;
};

/** Each poll consumes the latest pending PDU. Unstamped endpoints use the poll's simulation time. */
class IRobotPduReader {
public:
    virtual ~IRobotPduReader() = default;
    [[nodiscard]] virtual std::optional<RobotPduSample> take_latest(std::uint64_t poll_time_usec) = 0;
    virtual void reset() noexcept = 0;
};

} // namespace hakoniwa::robot_runtime::runtime
