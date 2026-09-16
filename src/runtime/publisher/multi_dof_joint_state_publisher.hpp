#pragma once

#include "runtime/publisher/state_publisher.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

struct MultiDofOutputBinding {
    std::string name;
    std::string body_id;
};

struct MultiDofBodyOutput {
    std::string name;
    Vector3State position;
    QuaternionState orientation;
    Vector3State linear_velocity;
    Vector3State angular_velocity;
};

struct MultiDofJointStateOutput {
    std::uint64_t simulation_time_usec {0};
    std::string frame_id;
    std::vector<MultiDofBodyOutput> bodies;
};

class IMultiDofJointStateWriter {
public:
    virtual ~IMultiDofJointStateWriter() = default;
    [[nodiscard]] virtual bool send(
        const MultiDofJointStateOutput& output) noexcept = 0;
    virtual void reset() noexcept = 0;
};

class MultiDofJointStatePublisher final : public IStatePublisher {
public:
    MultiDofJointStatePublisher(
        std::string component_id,
        std::string frame_id,
        std::vector<MultiDofOutputBinding> bindings,
        double update_rate_hz,
        std::shared_ptr<IMultiDofJointStateWriter> writer);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] ComponentStatus publish(
        const RobotState& state,
        const RuntimeStepContext& context) noexcept override;
    void reset() noexcept override;

private:
    std::string component_id_;
    std::string frame_id_;
    std::vector<MultiDofOutputBinding> bindings_;
    std::uint64_t update_period_usec_ {0};
    std::shared_ptr<IMultiDofJointStateWriter> writer_;
    std::uint64_t next_publish_time_usec_ {0};
    bool publish_ready_ {true};
};

} // namespace hakoniwa::robot_runtime::runtime
