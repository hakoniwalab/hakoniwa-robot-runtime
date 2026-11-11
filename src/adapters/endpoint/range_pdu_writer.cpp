#include "hakoniwa/robot_runtime/adapters/endpoint/range_pdu_writer.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#include "sensor_msgs/pdu_cpptype_Range.hpp"
#include "sensor_msgs/pdu_cpptype_conv_Range.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"

#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class RangePduWriter::Impl {
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
        HakoCpp_Range,
        ::hako::pdu::msgs::sensor_msgs::Range> endpoint;
};

RangePduWriter::RangePduWriter(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name)))
{
}

RangePduWriter::~RangePduWriter() = default;

bool RangePduWriter::send(const runtime::RangeOutput& output) noexcept
{
    try {
        HakoCpp_Range pdu {};
        pdu.header.stamp.sec = static_cast<Hako_int32>(output.simulation_time_usec / 1'000'000);
        pdu.header.stamp.nanosec = static_cast<Hako_uint32>((output.simulation_time_usec % 1'000'000) * 1000);
        pdu.header.frame_id = output.frame_id;
        pdu.radiation_type = output.range.radiation_type;
        pdu.field_of_view = static_cast<Hako_float32>(output.range.field_of_view);
        pdu.min_range = static_cast<Hako_float32>(output.range.min_range);
        pdu.max_range = static_cast<Hako_float32>(output.range.max_range);
        pdu.range = static_cast<Hako_float32>(output.range.range);
        pdu.variance = static_cast<Hako_float32>(output.range.variance);
        return impl_->endpoint.send(pdu) == HAKO_PDU_ERR_OK;
    } catch (...) {
        return false;
    }
}

void RangePduWriter::reset() noexcept
{
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
