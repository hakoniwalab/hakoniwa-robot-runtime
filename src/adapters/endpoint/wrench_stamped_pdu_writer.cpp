#include "hakoniwa/robot_runtime/adapters/endpoint/wrench_stamped_pdu_writer.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#include "geometry_msgs/pdu_cpptype_WrenchStamped.hpp"
#include "geometry_msgs/pdu_cpptype_conv_WrenchStamped.hpp"
#include "adapters/endpoint/encoding_pdu_endpoint.hpp"

#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class WrenchStampedPduWriter::Impl {
public:
    Impl(
        ::hakoniwa::pdu::Endpoint& endpoint_value,
        std::string pdu_robot,
        std::string pdu_name,
        std::shared_ptr<runtime::IEncodedPduSink> sink)
        : endpoint(endpoint_value,
            ::hakoniwa::pdu::PduKey {
                std::move(pdu_robot), std::move(pdu_name)},
            std::move(sink))
    {
    }

    EncodingPduEndpoint<
        HakoCpp_WrenchStamped,
        ::hako::pdu::msgs::geometry_msgs::WrenchStamped> endpoint;
};

WrenchStampedPduWriter::WrenchStampedPduWriter(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name,
    std::shared_ptr<runtime::IEncodedPduSink> sink)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name), std::move(sink)))
{
}

WrenchStampedPduWriter::~WrenchStampedPduWriter() = default;

bool WrenchStampedPduWriter::send(const runtime::WrenchOutput& output) noexcept
{
    HakoCpp_WrenchStamped pdu {};
    pdu.header.stamp.sec = static_cast<Hako_int32>(output.simulation_time_usec / 1'000'000);
    pdu.header.stamp.nanosec = static_cast<Hako_uint32>((output.simulation_time_usec % 1'000'000) * 1000);
    pdu.header.frame_id = output.frame_id;
    pdu.wrench.force.x = output.force[0];
    pdu.wrench.force.y = output.force[1];
    pdu.wrench.force.z = output.force[2];
    pdu.wrench.torque.x = output.torque[0];
    pdu.wrench.torque.y = output.torque[1];
    pdu.wrench.torque.z = output.torque[2];
    return impl_->endpoint.send(pdu, output.simulation_time_usec) == HAKO_PDU_ERR_OK;
}

void WrenchStampedPduWriter::reset() noexcept
{
    impl_->endpoint.reset();
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
