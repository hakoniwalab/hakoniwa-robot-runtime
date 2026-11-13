#include "hakoniwa/robot_runtime/adapters/endpoint/point_cloud2_pdu_writer.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#include "sensor_msgs/pdu_cpptype_PointCloud2.hpp"
#include "sensor_msgs/pdu_cpptype_conv_PointCloud2.hpp"
#include "adapters/endpoint/encoding_pdu_endpoint.hpp"

#include <cstring>
#include <utility>
#include <vector>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class PointCloud2PduWriter::Impl {
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
        HakoCpp_PointCloud2,
        ::hako::pdu::msgs::sensor_msgs::PointCloud2> endpoint;
};

PointCloud2PduWriter::PointCloud2PduWriter(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name,
    std::shared_ptr<runtime::IEncodedPduSink> sink)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name), std::move(sink)))
{
}

PointCloud2PduWriter::~PointCloud2PduWriter() = default;

namespace {
constexpr Hako_uint8 kFloat32 = 7;  // sensor_msgs/PointField
constexpr Hako_uint8 kUint16 = 4;
constexpr Hako_uint32 kPointStep = 22;

template <typename T>
void put(std::vector<Hako_uint8>& data, std::size_t offset, T value)
{
    std::memcpy(data.data() + offset, &value, sizeof(T));
}
} // namespace

bool PointCloud2PduWriter::send(const runtime::PointCloudOutput& output) noexcept
{
    if (output.scan == nullptr) {
        return false;
    }
    try {
        const auto& points = output.scan->points;
        HakoCpp_PointCloud2 pdu {};
        pdu.header.stamp.sec = static_cast<Hako_int32>(output.simulation_time_usec / 1'000'000);
        pdu.header.stamp.nanosec = static_cast<Hako_uint32>((output.simulation_time_usec % 1'000'000) * 1000);
        pdu.header.frame_id = output.frame_id;
        pdu.height = 1;
        pdu.width = static_cast<Hako_uint32>(points.size());
        pdu.fields = {
            {"x", 0, kFloat32, 1}, {"y", 4, kFloat32, 1}, {"z", 8, kFloat32, 1},
            {"intensity", 12, kFloat32, 1}, {"ring", 16, kUint16, 1}, {"time", 18, kFloat32, 1}};
        pdu.is_bigendian = false;
        pdu.point_step = kPointStep;
        pdu.row_step = kPointStep * pdu.width;
        pdu.data.resize(static_cast<std::size_t>(pdu.row_step));
        for (std::size_t i = 0; i < points.size(); ++i) {
            const std::size_t base = i * kPointStep;
            put(pdu.data, base + 0, points[i].x);
            put(pdu.data, base + 4, points[i].y);
            put(pdu.data, base + 8, points[i].z);
            put(pdu.data, base + 12, points[i].intensity);
            put(pdu.data, base + 16, points[i].ring);
            put(pdu.data, base + 18, points[i].time_offset);
        }
        pdu.is_dense = true;  // only returned (finite) points
        return impl_->endpoint.send(pdu, output.simulation_time_usec) == HAKO_PDU_ERR_OK;
    } catch (...) {
        return false;
    }
}

void PointCloud2PduWriter::reset() noexcept
{
    impl_->endpoint.reset();
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
