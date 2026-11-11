#include "hakoniwa/robot_runtime/adapters/endpoint/nav_sat_fix_pdu_writer.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#include "sensor_msgs/pdu_cpptype_NavSatFix.hpp"
#include "sensor_msgs/pdu_cpptype_conv_NavSatFix.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"

#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class NavSatFixPduWriter::Impl {
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
        HakoCpp_NavSatFix,
        ::hako::pdu::msgs::sensor_msgs::NavSatFix> endpoint;
};

NavSatFixPduWriter::NavSatFixPduWriter(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name)))
{
}

NavSatFixPduWriter::~NavSatFixPduWriter() = default;

bool NavSatFixPduWriter::send(const runtime::NavSatFixOutput& output) noexcept
{
    try {
        HakoCpp_NavSatFix pdu {};
        pdu.header.stamp.sec = static_cast<Hako_int32>(output.simulation_time_usec / 1'000'000);
        pdu.header.stamp.nanosec = static_cast<Hako_uint32>((output.simulation_time_usec % 1'000'000) * 1000);
        pdu.header.frame_id = output.frame_id;
        const auto& fix = output.fix;
        pdu.status.status = static_cast<Hako_int8>(fix.fix_type >= 2 ? 0 : -1);  // FIX / NO_FIX
        pdu.status.service = 1;  // SERVICE_GPS
        pdu.latitude = fix.latitude_deg;
        pdu.longitude = fix.longitude_deg;
        pdu.altitude = fix.altitude_m;
        pdu.position_covariance.fill(0.0);
        if (fix.horizontal_accuracy_m > 0.0 || fix.vertical_accuracy_m > 0.0) {
            const double h = fix.horizontal_accuracy_m * fix.horizontal_accuracy_m;
            pdu.position_covariance[0] = h;
            pdu.position_covariance[4] = h;
            pdu.position_covariance[8] = fix.vertical_accuracy_m * fix.vertical_accuracy_m;
            pdu.position_covariance_type = 2;  // COVARIANCE_TYPE_DIAGONAL_KNOWN
        } else {
            pdu.position_covariance_type = 0;  // COVARIANCE_TYPE_UNKNOWN
        }
        return impl_->endpoint.send(pdu) == HAKO_PDU_ERR_OK;
    } catch (...) {
        return false;
    }
}

void NavSatFixPduWriter::reset() noexcept
{
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
