#pragma once

#include "runtime/publisher/state_publisher.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace hakoniwa::robot_runtime::runtime {

struct DepthImageOutput {
    std::uint64_t simulation_time_usec {0};
    std::string frame_id;
    const DepthImageReading* image {nullptr};  ///< valid during send()
};

class IDepthImageWriter {
public:
    virtual ~IDepthImageWriter() = default;
    [[nodiscard]] virtual bool send(const DepthImageOutput& output) noexcept = 0;
    virtual void reset() noexcept = 0;
};

/**
 * Publishes the latest image of one depth camera as an Image (32FC1) with
 * its CameraInfo (same stamp),
 * rate-limited on simulation time like JointStatePublisher. frame_id is the
 * sensor name. Nothing is sent before the first image.
 */
class SensorDepthImagePublisher final : public IStatePublisher {
public:
    SensorDepthImagePublisher(
        std::string component_id,
        std::string sensor_name,
        double update_rate_hz,
        std::shared_ptr<IDepthImageWriter> writer);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] ComponentStatus publish(
        const RobotState& state,
        const RuntimeStepContext& context) noexcept override;
    void reset() noexcept override;

private:
    std::string component_id_;
    std::string sensor_name_;
    std::uint64_t update_period_usec_ {0};
    std::shared_ptr<IDepthImageWriter> writer_;
    std::uint64_t next_publish_time_usec_ {0};
    bool publish_ready_ {true};
};

} // namespace hakoniwa::robot_runtime::runtime
