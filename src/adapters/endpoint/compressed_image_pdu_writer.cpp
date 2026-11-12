#include "hakoniwa/robot_runtime/adapters/endpoint/compressed_image_pdu_writer.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#include "sensor_msgs/pdu_cpptype_CameraInfo.hpp"
#include "sensor_msgs/pdu_cpptype_conv_CameraInfo.hpp"
#include "sensor_msgs/pdu_cpptype_CompressedImage.hpp"
#include "sensor_msgs/pdu_cpptype_conv_CompressedImage.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC
#include "stb/stb_image_write.h"

#include <cstdint>
#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

namespace {

void append_bytes(void* context, void* data, int size)
{
    auto* out = static_cast<std::vector<Hako_uint8>*>(context);
    const auto* bytes = static_cast<const Hako_uint8*>(data);
    out->insert(out->end(), bytes, bytes + size);
}

} // namespace

class CompressedImagePduWriter::Impl {
public:
    struct Camera {
        std::string name;
        ::hakoniwa::pdu::TypedEndpoint<
            HakoCpp_CompressedImage,
            ::hako::pdu::msgs::sensor_msgs::CompressedImage> image;
        ::hakoniwa::pdu::TypedEndpoint<
            HakoCpp_CameraInfo,
            ::hako::pdu::msgs::sensor_msgs::CameraInfo> info;
    };

    Impl(
        ::hakoniwa::pdu::Endpoint& endpoint,
        const std::string& pdu_robot,
        const std::vector<runtime::RuntimeSensorOutputConfig::CameraChannels>& channels,
        int quality)
        : jpeg_quality(quality)
    {
        for (const auto& channel : channels) {
            cameras.push_back(std::make_unique<Camera>(Camera {
                channel.camera,
                {endpoint, ::hakoniwa::pdu::PduKey {pdu_robot, channel.pdu_name}},
                {endpoint, ::hakoniwa::pdu::PduKey {pdu_robot, channel.camera_info_pdu_name}},
            }));
        }
    }

    std::vector<std::unique_ptr<Camera>> cameras;
    int jpeg_quality;
};

CompressedImagePduWriter::CompressedImagePduWriter(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::vector<runtime::RuntimeSensorOutputConfig::CameraChannels> cameras,
    const int jpeg_quality)
    : impl_(std::make_unique<Impl>(endpoint, pdu_robot, cameras, jpeg_quality))
{
}

CompressedImagePduWriter::~CompressedImagePduWriter() = default;

bool CompressedImagePduWriter::send(const runtime::CompressedImageOutput& output) noexcept
{
    if (output.images == nullptr) {
        return false;
    }
    try {
        const auto sec = static_cast<Hako_int32>(output.simulation_time_usec / 1'000'000);
        const auto nanosec = static_cast<Hako_uint32>((output.simulation_time_usec % 1'000'000) * 1000);
        bool all_sent = true;
        for (const auto& camera : impl_->cameras) {
            const runtime::RgbCameraImageReading* image = nullptr;
            for (const auto& candidate : output.images->cameras) {
                if (candidate.camera_name == camera->name) {
                    image = &candidate;
                    break;
                }
            }
            if (image == nullptr) {
                all_sent = false;
                continue;
            }
            const std::string frame_id = camera->name.empty()
                ? output.frame_id
                : output.frame_id + "_" + camera->name;

            HakoCpp_CompressedImage compressed {};
            compressed.header.stamp.sec = sec;
            compressed.header.stamp.nanosec = nanosec;
            compressed.header.frame_id = frame_id;
            compressed.format = "rgb8; jpeg compressed rgb8";
            const int written = stbi_write_jpg_to_func(
                append_bytes, &compressed.data,
                static_cast<int>(image->width), static_cast<int>(image->height), 3,
                image->rgb.data(), impl_->jpeg_quality);
            if (written == 0) {
                all_sent = false;
                continue;
            }

            HakoCpp_CameraInfo info {};
            info.header.stamp.sec = sec;
            info.header.stamp.nanosec = nanosec;
            info.header.frame_id = frame_id;
            info.height = image->info.height;
            info.width = image->info.width;
            info.distortion_model = image->info.distortion_model;
            info.d = image->info.d;
            info.k = image->info.k;
            info.r = image->info.r;
            info.p = image->info.p;
            info.binning_x = 0;
            info.binning_y = 0;

            const bool image_sent = camera->image.send(compressed) == HAKO_PDU_ERR_OK;
            const bool info_sent = camera->info.send(info) == HAKO_PDU_ERR_OK;
            all_sent = all_sent && image_sent && info_sent;
        }
        return all_sent;
    } catch (...) {
        return false;
    }
}

void CompressedImagePduWriter::reset() noexcept
{
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
