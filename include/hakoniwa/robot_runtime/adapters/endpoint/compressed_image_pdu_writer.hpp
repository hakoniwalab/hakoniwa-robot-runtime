#pragma once

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#include "runtime/publisher/sensor_compressed_image_publisher.hpp"
#include "runtime/runtime_definition.hpp"

#include <memory>
#include <string>
#include <vector>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

/**
 * Writes the images of an RGB camera rig: for each listed camera, the colour
 * image JPEG-compressed as sensor_msgs/CompressedImage (format
 * "rgb8; jpeg compressed rgb8") on its channel and its sensor_msgs/CameraInfo
 * on another, with the same stamp. frame_id is the sensor name, followed by
 * "_" and the camera name for a named camera of a rig.
 */
class CompressedImagePduWriter final : public runtime::ICompressedImageWriter {
public:
    CompressedImagePduWriter(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::vector<runtime::RuntimeSensorOutputConfig::CameraChannels> cameras,
        int jpeg_quality);
    ~CompressedImagePduWriter() override;

    [[nodiscard]] bool send(const runtime::CompressedImageOutput& output) noexcept override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
