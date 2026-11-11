#include "hakoniwa/robot_runtime/adapters/endpoint/laser_scan_pdu_writer.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#include "sensor_msgs/pdu_cpptype_LaserScan.hpp"
#include "sensor_msgs/pdu_cpptype_conv_LaserScan.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"

#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class LaserScanPduWriter::Impl {
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
        HakoCpp_LaserScan,
        ::hako::pdu::msgs::sensor_msgs::LaserScan> endpoint;
};

LaserScanPduWriter::LaserScanPduWriter(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name)))
{
}

LaserScanPduWriter::~LaserScanPduWriter() = default;

bool LaserScanPduWriter::send(const runtime::LaserScanOutput& output) noexcept
{
    if (output.scan == nullptr) {
        return false;
    }
    try {
        const auto& scan = *output.scan;
        HakoCpp_LaserScan pdu {};
        pdu.header.stamp.sec = static_cast<Hako_int32>(output.simulation_time_usec / 1'000'000);
        pdu.header.stamp.nanosec = static_cast<Hako_uint32>((output.simulation_time_usec % 1'000'000) * 1000);
        pdu.header.frame_id = output.frame_id;
        pdu.angle_min = static_cast<Hako_float32>(scan.angle_min);
        pdu.angle_max = static_cast<Hako_float32>(scan.angle_max);
        pdu.angle_increment = static_cast<Hako_float32>(scan.angle_increment);
        pdu.time_increment = static_cast<Hako_float32>(scan.time_increment);
        pdu.scan_time = static_cast<Hako_float32>(scan.scan_time);
        pdu.range_min = static_cast<Hako_float32>(scan.range_min);
        pdu.range_max = static_cast<Hako_float32>(scan.range_max);
        pdu.ranges.assign(scan.ranges.begin(), scan.ranges.end());
        pdu.intensities.assign(scan.intensities.begin(), scan.intensities.end());
        return impl_->endpoint.send(pdu) == HAKO_PDU_ERR_OK;
    } catch (...) {
        return false;
    }
}

void LaserScanPduWriter::reset() noexcept
{
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
