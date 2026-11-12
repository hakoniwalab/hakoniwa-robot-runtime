#pragma once

#include "runtime/publisher/state_publisher.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace hakoniwa::robot_runtime::runtime {

struct CompressedImageOutput {
    std::uint64_t simulation_time_usec {0};
    std::string frame_id;
    const RgbImagesReading* images {nullptr};  ///< every camera of the rig; valid during send()
};

class ICompressedImageWriter {
public:
    virtual ~ICompressedImageWriter() = default;
    [[nodiscard]] virtual bool send(const CompressedImageOutput& output) noexcept = 0;
    virtual void reset() noexcept = 0;
};

/**
 * Publishes the latest images of one RGB camera rig: for each listed camera,
 * a JPEG CompressedImage with its CameraInfo (same stamp),
 * rate-limited on simulation time like JointStatePublisher. frame_id is the
 * sensor name. Nothing is sent before the first images.
 */
class SensorCompressedImagePublisher final : public IStatePublisher {
public:
    SensorCompressedImagePublisher(
        std::string component_id,
        std::string sensor_name,
        double update_rate_hz,
        std::shared_ptr<ICompressedImageWriter> writer);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] ComponentStatus publish(
        const RobotState& state,
        const RuntimeStepContext& context) noexcept override;
    void reset() noexcept override;

private:
    std::string component_id_;
    std::string sensor_name_;
    std::uint64_t update_period_usec_ {0};
    std::shared_ptr<ICompressedImageWriter> writer_;
    std::uint64_t next_publish_time_usec_ {0};
    bool publish_ready_ {true};
};

} // namespace hakoniwa::robot_runtime::runtime
