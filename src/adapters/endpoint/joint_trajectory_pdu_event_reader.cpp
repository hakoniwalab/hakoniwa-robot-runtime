#include "hakoniwa/robot_runtime/adapters/endpoint/joint_trajectory_pdu_event_reader.hpp"

#include "hakoniwa/pdu/adapter/trajectory_msgs/joint_trajectory.hpp"
#include "hakoniwa/pdu/endpoint.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class JointTrajectoryPduEventReader::Impl {
public:
    Impl(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name)
        : reader(
            endpoint,
            ::hakoniwa::pdu::PduKey {
                std::move(pdu_robot), std::move(pdu_name)})
    {
    }

    ::hako::robots::pdu::adapter::trajectory_msgs::JointTrajectoryEventReader reader;
};

JointTrajectoryPduEventReader::JointTrajectoryPduEventReader(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name)))
{
}

JointTrajectoryPduEventReader::~JointTrajectoryPduEventReader() = default;

void JointTrajectoryPduEventReader::subscribe()
{
    impl_->reader.subscribe();
}

std::optional<runtime::JointTrajectory>
JointTrajectoryPduEventReader::take_latest()
{
    ::hako::robots::actuator::JointTrajectoryTarget input;
    if (!impl_->reader.take(input)) {
        return std::nullopt;
    }
    runtime::JointTrajectory output;
    output.joint_names = std::move(input.joint_names);
    output.points.reserve(input.points.size());
    for (auto& point : input.points) {
        const double time_usec = point.time_from_start_sec * 1'000'000.0;
        if (!std::isfinite(time_usec) || time_usec < 0.0
            || time_usec > static_cast<double>(
                std::numeric_limits<std::int64_t>::max())) {
            // Preserve the receive event but make validation fail explicitly.
            output.points.clear();
            return output;
        }
        output.points.push_back({
            std::move(point.positions),
            std::move(point.velocities),
            std::move(point.effort),
            static_cast<std::uint64_t>(std::llround(time_usec)),
        });
    }
    return output;
}

void JointTrajectoryPduEventReader::reset() noexcept
{
    ::hako::robots::actuator::JointTrajectoryTarget discarded;
    (void)impl_->reader.take(discarded);
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
