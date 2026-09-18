#pragma once

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR

#include "runtime/source/mirror_body_state_source.hpp"

#include <memory>
#include <optional>
#include <string>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class MirrorBodyTwistPduReader final
    : public runtime::IMirrorBodyStateReader {
public:
    MirrorBodyTwistPduReader(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pose_pdu_name,
        std::optional<std::string> velocity_pdu_name);
    ~MirrorBodyTwistPduReader() override;

    [[nodiscard]] std::optional<runtime::MirrorBodySample>
        read_latest() override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
