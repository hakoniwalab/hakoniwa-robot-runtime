#include "hakoniwa/robot_runtime/adapters/endpoint/imu_pdu_writer.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#include "sensor_msgs/pdu_cpptype_Imu.hpp"
#include "sensor_msgs/pdu_cpptype_conv_Imu.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"

#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class ImuPduWriter::Impl {
public:
    Impl(
        ::hakoniwa::pdu::Endpoint& endpoint_value,
        std::string pdu_robot,
        std::string pdu_name)
        : endpoint(endpoint_value,
            ::hakoniwa::pdu::PduKey {
                std::move(pdu_robot), std::move(pdu_name)})
    {
    }

    ::hakoniwa::pdu::TypedEndpoint<
        HakoCpp_Imu,
        ::hako::pdu::msgs::sensor_msgs::Imu> endpoint;
};

ImuPduWriter::ImuPduWriter(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name)))
{
}

ImuPduWriter::~ImuPduWriter() = default;

bool ImuPduWriter::send(const runtime::ImuOutput& output) noexcept
{
    HakoCpp_Imu pdu {};
    pdu.header.stamp.sec = static_cast<Hako_int32>(output.simulation_time_usec / 1'000'000);
    pdu.header.stamp.nanosec = static_cast<Hako_uint32>((output.simulation_time_usec % 1'000'000) * 1000);
    pdu.header.frame_id = output.frame_id;
    // An IMU does not measure orientation (ROS: covariance[0] = -1).
    pdu.orientation.x = 0.0;
    pdu.orientation.y = 0.0;
    pdu.orientation.z = 0.0;
    pdu.orientation.w = 1.0;
    pdu.orientation_covariance.fill(0.0);
    pdu.orientation_covariance[0] = -1.0;
    pdu.angular_velocity.x = output.angular_velocity[0];
    pdu.angular_velocity.y = output.angular_velocity[1];
    pdu.angular_velocity.z = output.angular_velocity[2];
    pdu.angular_velocity_covariance.fill(0.0);
    pdu.linear_acceleration.x = output.linear_acceleration[0];
    pdu.linear_acceleration.y = output.linear_acceleration[1];
    pdu.linear_acceleration.z = output.linear_acceleration[2];
    pdu.linear_acceleration_covariance.fill(0.0);
    return impl_->endpoint.send(pdu) == HAKO_PDU_ERR_OK;
}

void ImuPduWriter::reset() noexcept
{
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
