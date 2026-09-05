#include "hakoniwa/robot_runtime/adapters/endpoint/joint_state_pdu_writer.hpp"

#include "hakoniwa/pdu/adapter/sensor_msgs/joint_state.hpp"
#include "hakoniwa/pdu/endpoint.hpp"

#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class JointStatePduWriter::Impl {
public:
    Impl(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name)
        : adapter(
            endpoint,
            ::hakoniwa::pdu::PduKey {
                std::move(pdu_robot), std::move(pdu_name)})
    {
    }

    ::hako::robots::pdu::adapter::sensor_msgs::JointStatePduAdapter adapter;
};

JointStatePduWriter::JointStatePduWriter(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name)))
{
}

JointStatePduWriter::~JointStatePduWriter() = default;

bool JointStatePduWriter::send(
    const runtime::JointStateOutput& output) noexcept
{
    ::hako::robots::sensor::JointStateFrame frame;
    frame.header.frame_id = "";
    frame.header.stamp_sec =
        static_cast<double>(output.simulation_time_usec) / 1'000'000.0;
    frame.names = output.names;
    frame.position = output.position;
    frame.velocity = output.velocity;
    frame.effort = output.effort;
    return impl_->adapter.send(frame);
}

void JointStatePduWriter::reset() noexcept
{
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
