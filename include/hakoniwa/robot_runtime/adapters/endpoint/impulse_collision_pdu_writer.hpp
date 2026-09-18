#pragma once

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR

#include "runtime/publisher/impulse_collision_publisher.hpp"

#include <memory>
#include <string>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class ImpulseCollisionPduWriter final
    : public runtime::IImpulseCollisionWriter {
public:
    ImpulseCollisionPduWriter(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name);
    ~ImpulseCollisionPduWriter() override;

    [[nodiscard]] bool send(
        const runtime::ImpulseCollisionOutput& output) noexcept override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
