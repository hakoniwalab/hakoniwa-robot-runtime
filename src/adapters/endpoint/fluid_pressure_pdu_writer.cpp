#include "hakoniwa/robot_runtime/adapters/endpoint/fluid_pressure_pdu_writer.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#include "sensor_msgs/pdu_cpptype_FluidPressure.hpp"
#include "sensor_msgs/pdu_cpptype_conv_FluidPressure.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"

#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class FluidPressurePduWriter::Impl {
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
        HakoCpp_FluidPressure,
        ::hako::pdu::msgs::sensor_msgs::FluidPressure> endpoint;
};

FluidPressurePduWriter::FluidPressurePduWriter(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name)))
{
}

FluidPressurePduWriter::~FluidPressurePduWriter() = default;

bool FluidPressurePduWriter::send(const runtime::FluidPressureOutput& output) noexcept
{
    try {
        HakoCpp_FluidPressure pdu {};
        pdu.header.stamp.sec = static_cast<Hako_int32>(output.simulation_time_usec / 1'000'000);
        pdu.header.stamp.nanosec = static_cast<Hako_uint32>((output.simulation_time_usec % 1'000'000) * 1000);
        pdu.header.frame_id = output.frame_id;
        pdu.fluid_pressure = output.pressure_pa;
        pdu.variance = 0.0;
        return impl_->endpoint.send(pdu) == HAKO_PDU_ERR_OK;
    } catch (...) {
        return false;
    }
}

void FluidPressurePduWriter::reset() noexcept
{
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
