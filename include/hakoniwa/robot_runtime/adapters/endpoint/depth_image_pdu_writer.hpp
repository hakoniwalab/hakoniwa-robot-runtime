#pragma once

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#include "runtime/publisher/sensor_depth_image_publisher.hpp"

#include <memory>
#include <string>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

/**
 * Writes a DepthImageOutput as sensor_msgs/Image (32FC1, little endian,
 * metres) on one channel and its sensor_msgs/CameraInfo on another, with
 * the same stamp and frame_id, as ROS cameras publish them.
 */
class DepthImagePduWriter final : public runtime::IDepthImageWriter {
public:
    DepthImagePduWriter(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string image_pdu_name,
        std::string camera_info_pdu_name);
    ~DepthImagePduWriter() override;

    [[nodiscard]] bool send(const runtime::DepthImageOutput& output) noexcept override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
