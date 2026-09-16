#include "hakoniwa/robot_runtime/adapters/endpoint/multi_dof_joint_state_pdu_writer.hpp"

#define hako_convert_pdu2cpp_array_string_varray hako_convert_pdu2ros_array_string_varray
#define hako_convert_cpp2pdu_array_string_varray hako_convert_ros2pdu_array_string_varray
#include "sensor_msgs/pdu_cpptype_conv_MultiDOFJointState.hpp"
#undef hako_convert_pdu2cpp_array_string_varray
#undef hako_convert_cpp2pdu_array_string_varray

#include "hakoniwa/pdu/converter/common.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "sensor_msgs/pdu_cpptype_MultiDOFJointState.hpp"

#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class MultiDofJointStatePduWriter::Impl {
public:
    Impl(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name)
        : endpoint(
            endpoint,
            ::hakoniwa::pdu::PduKey {
                std::move(pdu_robot), std::move(pdu_name)})
    {
    }

    ::hakoniwa::pdu::TypedEndpoint<
        HakoCpp_MultiDOFJointState,
        ::hako::pdu::msgs::sensor_msgs::MultiDOFJointState> endpoint;
};

MultiDofJointStatePduWriter::MultiDofJointStatePduWriter(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name)))
{
}

MultiDofJointStatePduWriter::~MultiDofJointStatePduWriter() = default;

bool MultiDofJointStatePduWriter::send(
    const runtime::MultiDofJointStateOutput& output) noexcept
{
    HakoCpp_MultiDOFJointState pdu {};
    pdu.header.stamp = ::hako::robots::pdu::converter::ToHakoTime(
        static_cast<double>(output.simulation_time_usec) / 1'000'000.0);
    pdu.header.frame_id = output.frame_id;
    pdu.joint_names.reserve(output.bodies.size());
    pdu.transforms.reserve(output.bodies.size());
    pdu.twist.reserve(output.bodies.size());
    pdu.wrench.reserve(output.bodies.size());
    for (const auto& body : output.bodies) {
        pdu.joint_names.push_back(body.name);
        HakoCpp_Transform transform {};
        transform.translation.x = body.position.x;
        transform.translation.y = body.position.y;
        transform.translation.z = body.position.z;
        transform.rotation.x = body.orientation.x;
        transform.rotation.y = body.orientation.y;
        transform.rotation.z = body.orientation.z;
        transform.rotation.w = body.orientation.w;
        pdu.transforms.push_back(transform);
        HakoCpp_Twist twist {};
        twist.linear.x = body.linear_velocity.x;
        twist.linear.y = body.linear_velocity.y;
        twist.linear.z = body.linear_velocity.z;
        twist.angular.x = body.angular_velocity.x;
        twist.angular.y = body.angular_velocity.y;
        twist.angular.z = body.angular_velocity.z;
        pdu.twist.push_back(twist);
        pdu.wrench.emplace_back();
    }
    return impl_->endpoint.send(pdu) == HAKO_PDU_ERR_OK;
}

void MultiDofJointStatePduWriter::reset() noexcept
{
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
