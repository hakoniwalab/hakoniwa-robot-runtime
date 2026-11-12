#include "hakoniwa/robot_runtime/adapters/endpoint/depth_image_pdu_writer.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#include "sensor_msgs/pdu_cpptype_CameraInfo.hpp"
#include "sensor_msgs/pdu_cpptype_conv_CameraInfo.hpp"
#include "sensor_msgs/pdu_cpptype_Image.hpp"
#include "sensor_msgs/pdu_cpptype_conv_Image.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"

#include <cstring>
#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class DepthImagePduWriter::Impl {
public:
    Impl(
        ::hakoniwa::pdu::Endpoint& endpoint_value,
        const std::string& pdu_robot,
        std::string image_pdu_name,
        std::string camera_info_pdu_name)
        : image(endpoint_value,
            ::hakoniwa::pdu::PduKey {pdu_robot, std::move(image_pdu_name)})
        , info(endpoint_value,
            ::hakoniwa::pdu::PduKey {pdu_robot, std::move(camera_info_pdu_name)})
    {
    }

    ::hakoniwa::pdu::TypedEndpoint<
        HakoCpp_Image,
        ::hako::pdu::msgs::sensor_msgs::Image> image;
    ::hakoniwa::pdu::TypedEndpoint<
        HakoCpp_CameraInfo,
        ::hako::pdu::msgs::sensor_msgs::CameraInfo> info;
};

DepthImagePduWriter::DepthImagePduWriter(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string image_pdu_name,
    std::string camera_info_pdu_name)
    : impl_(std::make_unique<Impl>(
        endpoint, pdu_robot, std::move(image_pdu_name), std::move(camera_info_pdu_name)))
{
}

DepthImagePduWriter::~DepthImagePduWriter() = default;

bool DepthImagePduWriter::send(const runtime::DepthImageOutput& output) noexcept
{
    if (output.image == nullptr) {
        return false;
    }
    try {
        const auto& image = *output.image;
        const auto sec = static_cast<Hako_int32>(output.simulation_time_usec / 1'000'000);
        const auto nanosec = static_cast<Hako_uint32>((output.simulation_time_usec % 1'000'000) * 1000);

        HakoCpp_Image pdu {};
        pdu.header.stamp.sec = sec;
        pdu.header.stamp.nanosec = nanosec;
        pdu.header.frame_id = output.frame_id;
        pdu.height = image.height;
        pdu.width = image.width;
        pdu.encoding = "32FC1";
        pdu.is_bigendian = 0;
        pdu.step = image.width * static_cast<Hako_uint32>(sizeof(float));
        pdu.data.resize(image.depths.size() * sizeof(float));
        if (!image.depths.empty()) {
            std::memcpy(pdu.data.data(), image.depths.data(), pdu.data.size());
        }

        HakoCpp_CameraInfo info {};
        info.header.stamp.sec = sec;
        info.header.stamp.nanosec = nanosec;
        info.header.frame_id = output.frame_id;
        info.height = image.info.height;
        info.width = image.info.width;
        info.distortion_model = image.info.distortion_model;
        info.d = image.info.d;
        info.k = image.info.k;
        info.r = image.info.r;
        info.p = image.info.p;
        info.binning_x = 0;
        info.binning_y = 0;

        const bool image_sent = impl_->image.send(pdu) == HAKO_PDU_ERR_OK;
        const bool info_sent = impl_->info.send(info) == HAKO_PDU_ERR_OK;
        return image_sent && info_sent;
    } catch (...) {
        return false;
    }
}

void DepthImagePduWriter::reset() noexcept
{
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
