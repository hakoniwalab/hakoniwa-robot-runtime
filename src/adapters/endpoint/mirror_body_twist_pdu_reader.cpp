#include "hakoniwa/robot_runtime/adapters/endpoint/mirror_body_twist_pdu_reader.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR

#include "geometry_msgs/pdu_cpptype_Twist.hpp"
#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"

#include <optional>
#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {
namespace {

using TwistEndpoint = ::hakoniwa::pdu::TypedEndpoint<
    HakoCpp_Twist,
    ::hako::pdu::msgs::geometry_msgs::Twist>;

} // namespace

class MirrorBodyTwistPduReader::Impl {
public:
    Impl(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pose_pdu_name,
        std::optional<std::string> velocity_pdu_name)
        : pose(endpoint, {pdu_robot, std::move(pose_pdu_name)})
    {
        if (velocity_pdu_name.has_value()) {
            velocity.emplace(
                endpoint,
                ::hakoniwa::pdu::PduKey {
                    std::move(pdu_robot), std::move(*velocity_pdu_name)});
        }
    }

    TwistEndpoint pose;
    std::optional<TwistEndpoint> velocity;
};

MirrorBodyTwistPduReader::MirrorBodyTwistPduReader(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pose_pdu_name,
    std::optional<std::string> velocity_pdu_name)
    : impl_(std::make_unique<Impl>(
        endpoint,
        std::move(pdu_robot),
        std::move(pose_pdu_name),
        std::move(velocity_pdu_name)))
{
}

MirrorBodyTwistPduReader::~MirrorBodyTwistPduReader() = default;

std::optional<runtime::MirrorBodySample>
MirrorBodyTwistPduReader::read_latest()
{
    HakoCpp_Twist pose {};
    if (impl_->pose.recv(pose) != HAKO_PDU_ERR_OK) {
        return std::nullopt;
    }
    runtime::MirrorBodySample sample {
        {pose.linear.x, pose.linear.y, pose.linear.z},
        {pose.angular.x, pose.angular.y, pose.angular.z},
        std::nullopt,
        std::nullopt,
    };
    if (impl_->velocity.has_value()) {
        HakoCpp_Twist velocity {};
        if (impl_->velocity->recv(velocity) == HAKO_PDU_ERR_OK) {
            sample.linear_velocity = runtime::Vector3State {
                velocity.linear.x, velocity.linear.y, velocity.linear.z};
            sample.angular_velocity = runtime::Vector3State {
                velocity.angular.x, velocity.angular.y, velocity.angular.z};
        }
    }
    return sample;
}

void MirrorBodyTwistPduReader::reset() noexcept
{
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
