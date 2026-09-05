#pragma once

#include "runtime/source/joy_command_source.hpp"

#include <memory>
#include <optional>
#include <string>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

/** Latest-wins receive-event adapter for a sensor_msgs/Joy PDU channel. */
class JoyPduEventReader final : public runtime::IJoyEventReader {
public:
    JoyPduEventReader(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name);
    ~JoyPduEventReader() override;

    void subscribe();
    [[nodiscard]] std::optional<runtime::JoyState> take_latest() override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
