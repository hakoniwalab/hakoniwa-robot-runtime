#pragma once

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#include "runtime/publisher/sensor_point_cloud_publisher.hpp"

#include <memory>
#include <string>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

/**
 * Writes a PointCloudOutput as sensor_msgs/PointCloud2: one row of the
 * returned points, little endian, fields x, y, z, intensity (float32),
 * ring (uint16) and time (float32, since the start of the scan), as common
 * 3D LiDAR drivers publish them.
 */
class PointCloud2PduWriter final : public runtime::IPointCloudWriter {
public:
    PointCloud2PduWriter(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name);
    ~PointCloud2PduWriter() override;

    [[nodiscard]] bool send(const runtime::PointCloudOutput& output) noexcept override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
