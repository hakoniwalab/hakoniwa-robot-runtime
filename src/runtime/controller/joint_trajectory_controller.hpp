#pragma once

#include "runtime/controller/controller.hpp"
#include "runtime/source/joint_trajectory_command_source.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

struct TrajectoryActuatorBinding {
    std::string joint_name;
    std::string actuator_id;
    ActuatorCommandType command_type {ActuatorCommandType::Position};
};

/**
 * Linear interpolation of validated joint targets in simulation time.
 *
 * A trajectory remains active only while this logical Controller continues to
 * be selected. If another control preempts it, the old time-based trajectory is
 * cancelled so it cannot advance invisibly and resume later from a stale time.
 * A fresh trajectory input may always start a new AUTO request.
 */
class JointTrajectoryController final : public IController {
public:
    JointTrajectoryController(
        std::string controller_id,
        std::string source_id,
        std::vector<TrajectoryActuatorBinding> bindings);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] std::string_view source_id() const noexcept override;
    [[nodiscard]] bool accepts(const IControllerInput& input) const noexcept override;
    [[nodiscard]] ControllerOutput update(
        std::shared_ptr<const IControllerInput> input,
        const RobotState& current_state,
        const RuntimeStepContext& context) override;
    void reset(const RobotState& current_state) override;

private:
    struct ActiveBinding {
        TrajectoryActuatorBinding binding;
        std::size_t trajectory_index {0};
    };

    [[nodiscard]] std::string validate_and_activate(
        const JointTrajectory& trajectory,
        const RuntimeStepContext& context);
    [[nodiscard]] std::vector<ActuatorCommand> interpolate(
        const RuntimeStepContext& context) const;
    void cancel() noexcept;

    std::string controller_id_;
    std::string source_id_;
    std::vector<TrajectoryActuatorBinding> configured_bindings_;
    std::vector<ActiveBinding> active_bindings_;
    JointTrajectory active_trajectory_;
    std::uint64_t start_time_usec_ {0};
    bool active_ {false};
};

} // namespace hakoniwa::robot_runtime::runtime
