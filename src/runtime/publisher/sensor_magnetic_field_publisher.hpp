#pragma once

#include "runtime/publisher/state_publisher.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace hakoniwa::robot_runtime::runtime {

struct MagneticFieldOutput {
    std::uint64_t simulation_time_usec {0};
    std::string frame_id;
    std::array<double, 3> magnetic_field_nT {0.0, 0.0, 0.0};  ///< sensor axes
};

class IMagneticFieldWriter {
public:
    virtual ~IMagneticFieldWriter() = default;
    [[nodiscard]] virtual bool send(const MagneticFieldOutput& output) noexcept = 0;
    virtual void reset() noexcept = 0;
};

/**
 * Publishes the latest measurement of one magnetometer as a MagneticField,
 * rate-limited on simulation time like JointStatePublisher. frame_id is the
 * sensor name. Nothing is sent before the first scan.
 */
class SensorMagneticFieldPublisher final : public IStatePublisher {
public:
    SensorMagneticFieldPublisher(
        std::string component_id,
        std::string sensor_name,
        double update_rate_hz,
        std::shared_ptr<IMagneticFieldWriter> writer);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] ComponentStatus publish(
        const RobotState& state,
        const RuntimeStepContext& context) noexcept override;
    void reset() noexcept override;

private:
    std::string component_id_;
    std::string sensor_name_;
    std::uint64_t update_period_usec_ {0};
    std::shared_ptr<IMagneticFieldWriter> writer_;
    std::uint64_t next_publish_time_usec_ {0};
    bool publish_ready_ {true};
};

} // namespace hakoniwa::robot_runtime::runtime
