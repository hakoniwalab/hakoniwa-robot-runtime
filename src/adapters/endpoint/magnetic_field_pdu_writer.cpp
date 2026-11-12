#include "hakoniwa/robot_runtime/adapters/endpoint/magnetic_field_pdu_writer.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#include "sensor_msgs/pdu_cpptype_MagneticField.hpp"
#include "sensor_msgs/pdu_cpptype_conv_MagneticField.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"

#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class MagneticFieldPduWriter::Impl {
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
        HakoCpp_MagneticField,
        ::hako::pdu::msgs::sensor_msgs::MagneticField> endpoint;
};

MagneticFieldPduWriter::MagneticFieldPduWriter(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name)))
{
}

MagneticFieldPduWriter::~MagneticFieldPduWriter() = default;

bool MagneticFieldPduWriter::send(const runtime::MagneticFieldOutput& output) noexcept
{
    try {
        HakoCpp_MagneticField pdu {};
        pdu.header.stamp.sec = static_cast<Hako_int32>(output.simulation_time_usec / 1'000'000);
        pdu.header.stamp.nanosec = static_cast<Hako_uint32>((output.simulation_time_usec % 1'000'000) * 1000);
        pdu.header.frame_id = output.frame_id;
        // ROS: the field is in tesla; the sensor gives nanotesla.
        pdu.magnetic_field.x = output.magnetic_field_nT[0] * 1e-9;
        pdu.magnetic_field.y = output.magnetic_field_nT[1] * 1e-9;
        pdu.magnetic_field.z = output.magnetic_field_nT[2] * 1e-9;
        pdu.magnetic_field_covariance.fill(0.0);
        return impl_->endpoint.send(pdu) == HAKO_PDU_ERR_OK;
    } catch (...) {
        return false;
    }
}

void MagneticFieldPduWriter::reset() noexcept
{
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
