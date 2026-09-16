#pragma once

#include "runtime/publisher/multi_dof_joint_state_publisher.hpp"

#include <memory>
#include <string>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class MultiDofJointStatePduWriter final
    : public runtime::IMultiDofJointStateWriter {
public:
    MultiDofJointStatePduWriter(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name);
    ~MultiDofJointStatePduWriter() override;

    [[nodiscard]] bool send(
        const runtime::MultiDofJointStateOutput& output) noexcept override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
