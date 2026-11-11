#pragma once

#include "runtime/publisher/state_publisher.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace hakoniwa::robot_runtime::runtime {

struct PointCloudOutput {
    std::uint64_t simulation_time_usec {0};  ///< time of the first ray
    std::string frame_id;
    const LidarScanReading* scan {nullptr};  ///< valid during send()
};

class IPointCloudWriter {
public:
    virtual ~IPointCloudWriter() = default;
    [[nodiscard]] virtual bool send(const PointCloudOutput& output) noexcept = 0;
    virtual void reset() noexcept = 0;
};

/**
 * Publishes the returned points of the latest complete scan of one LiDAR
 * (2D or 3D) as a PointCloud2,
 * rate-limited on simulation time like JointStatePublisher. frame_id is the
 * sensor name. Nothing is sent before the first scan.
 */
class SensorPointCloudPublisher final : public IStatePublisher {
public:
    SensorPointCloudPublisher(
        std::string component_id,
        std::string sensor_name,
        double update_rate_hz,
        std::shared_ptr<IPointCloudWriter> writer);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] ComponentStatus publish(
        const RobotState& state,
        const RuntimeStepContext& context) noexcept override;
    void reset() noexcept override;

private:
    std::string component_id_;
    std::string sensor_name_;
    std::uint64_t update_period_usec_ {0};
    std::shared_ptr<IPointCloudWriter> writer_;
    std::uint64_t next_publish_time_usec_ {0};
    bool publish_ready_ {true};
};

} // namespace hakoniwa::robot_runtime::runtime
