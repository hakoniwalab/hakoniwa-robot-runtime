#pragma once

#include "runtime/publisher/state_publisher.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace hakoniwa::robot_runtime::runtime {

struct RangeOutput {
    std::uint64_t simulation_time_usec {0};
    std::string frame_id;
    RangeReading range;
};

class IRangeWriter {
public:
    virtual ~IRangeWriter() = default;
    [[nodiscard]] virtual bool send(const RangeOutput& output) noexcept = 0;
    virtual void reset() noexcept = 0;
};

/**
 * Publishes the latest measured range of one rangefinder as a Range,
 * rate-limited on simulation time like JointStatePublisher. frame_id is the
 * sensor name. Nothing is sent before the first scan.
 */
class SensorRangePublisher final : public IStatePublisher {
public:
    SensorRangePublisher(
        std::string component_id,
        std::string sensor_name,
        double update_rate_hz,
        std::shared_ptr<IRangeWriter> writer);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] ComponentStatus publish(
        const RobotState& state,
        const RuntimeStepContext& context) noexcept override;
    void reset() noexcept override;

private:
    std::string component_id_;
    std::string sensor_name_;
    std::uint64_t update_period_usec_ {0};
    std::shared_ptr<IRangeWriter> writer_;
    std::uint64_t next_publish_time_usec_ {0};
    bool publish_ready_ {true};
};

} // namespace hakoniwa::robot_runtime::runtime
