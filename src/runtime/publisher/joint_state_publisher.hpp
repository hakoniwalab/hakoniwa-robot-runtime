#pragma once

#include "runtime/publisher/state_publisher.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

struct JointStateOutputBinding {
    std::string joint_name;
    std::string actuator_id;
};

struct JointStateOutput {
    std::uint64_t simulation_time_usec {0};
    std::vector<std::string> names;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
};

class IJointStateWriter {
public:
    virtual ~IJointStateWriter() = default;
    [[nodiscard]] virtual bool send(const JointStateOutput& output) noexcept = 0;
    virtual void reset() noexcept = 0;
};

/** Rate-limited RobotState to JointState boundary using simulation time only. */
class JointStatePublisher final : public IStatePublisher {
public:
    JointStatePublisher(
        std::string component_id,
        std::vector<JointStateOutputBinding> bindings,
        double update_rate_hz,
        std::shared_ptr<IJointStateWriter> writer);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] ComponentStatus publish(
        const RobotState& state,
        const RuntimeStepContext& context) noexcept override;
    void reset() noexcept override;

private:
    std::string component_id_;
    std::vector<JointStateOutputBinding> bindings_;
    std::uint64_t update_period_usec_ {0};
    std::shared_ptr<IJointStateWriter> writer_;
    std::uint64_t next_publish_time_usec_ {0};
    bool publish_ready_ {true};
};

} // namespace hakoniwa::robot_runtime::runtime
