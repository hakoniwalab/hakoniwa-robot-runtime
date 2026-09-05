#pragma once

#include "runtime/source/joint_trajectory_command_source.hpp"

#include <memory>
#include <optional>
#include <string>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class JointTrajectoryPduEventReader final
    : public runtime::IJointTrajectoryEventReader {
public:
    JointTrajectoryPduEventReader(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name);
    ~JointTrajectoryPduEventReader() override;

    void subscribe();
    [[nodiscard]] std::optional<runtime::JointTrajectory> take_latest() override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
