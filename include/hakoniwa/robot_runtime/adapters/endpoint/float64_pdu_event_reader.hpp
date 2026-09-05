#pragma once

#include "runtime/source/scalar_pdu_command_source.hpp"

#include <memory>
#include <optional>
#include <string>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

/** Latest-wins receive-event adapter for a std_msgs/Float64 PDU channel. */
class Float64PduEventReader final : public runtime::IFloat64EventReader {
public:
    Float64PduEventReader(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name);
    ~Float64PduEventReader() override;

    void subscribe();
    [[nodiscard]] std::optional<double> take_latest() override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
