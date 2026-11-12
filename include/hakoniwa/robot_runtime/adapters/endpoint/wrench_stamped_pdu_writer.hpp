#pragma once

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#include "runtime/publisher/sensor_wrench_publisher.hpp"

#include "runtime/publisher/encoded_pdu_sink.hpp"

#include <memory>
#include <string>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

/** Writes a WrenchOutput as geometry_msgs/WrenchStamped. */
class WrenchStampedPduWriter final : public runtime::IWrenchWriter {
public:
    /**
     * sink (optional) receives the encoded bytes of every sample sent and is
     * cleared on reset (a controller plugin's PDU channel).
     */
    WrenchStampedPduWriter(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name,
        std::shared_ptr<runtime::IEncodedPduSink> sink = nullptr);
    ~WrenchStampedPduWriter() override;

    [[nodiscard]] bool send(const runtime::WrenchOutput& output) noexcept override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
