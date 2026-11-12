#pragma once

#include "runtime/publisher/joint_state_publisher.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

/**
 * Publishes the measured values of named joint sensors as one JointState,
 * stamped with the measurement time (ROS), rate-limited on simulation time like JointStatePublisher.
 *
 * Each entry is named by the MJCF joint the sensor measures and carries the
 * sensor's latest position and velocity, and its joint torque as effort when
 * the sensors measure it (all of them or none). Nothing is
 * sent until every listed sensor has taken its first sample.
 */
class SensorJointStatePublisher final : public IStatePublisher {
public:
    SensorJointStatePublisher(
        std::string component_id,
        std::vector<std::string> sensor_names,
        double update_rate_hz,
        std::shared_ptr<IJointStateWriter> writer);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] ComponentStatus publish(
        const RobotState& state,
        const RuntimeStepContext& context) noexcept override;
    void reset() noexcept override;

private:
    std::string component_id_;
    std::vector<std::string> sensor_names_;
    std::uint64_t update_period_usec_ {0};
    std::shared_ptr<IJointStateWriter> writer_;
    std::uint64_t next_publish_time_usec_ {0};
    bool publish_ready_ {true};
};

} // namespace hakoniwa::robot_runtime::runtime
