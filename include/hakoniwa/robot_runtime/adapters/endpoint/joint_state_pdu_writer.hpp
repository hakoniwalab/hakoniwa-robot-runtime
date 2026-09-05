#pragma once

#include "runtime/publisher/joint_state_publisher.hpp"

#include <memory>
#include <string>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class JointStatePduWriter final : public runtime::IJointStateWriter {
public:
    JointStatePduWriter(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name);
    ~JointStatePduWriter() override;

    [[nodiscard]] bool send(
        const runtime::JointStateOutput& output) noexcept override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
