#pragma once

#include "runtime/arbiter/command_arbiter.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

/**
 * Priority of one concrete Controller.
 *
 * `control_group_id` is optional and is used only when multiple concrete
 * Controllers intentionally represent one logical control strategy. The
 * current scalar direct path uses this to combine its per-actuator Controller
 * instances before selection.
 */
struct ControllerPriority {
    std::string controller_id;
    std::int32_t priority {0};
    std::string control_group_id;
};

/** Highest numeric Controller priority wins for the current Runtime step. */
class PriorityCommandArbiter final : public ICommandArbiter {
public:
    /** @throws std::invalid_argument for invalid or inconsistent policies. */
    explicit PriorityCommandArbiter(std::vector<ControllerPriority> priorities);

    [[nodiscard]] ArbitrationResult arbitrate(
        const std::vector<ControllerOutput>& controller_outputs,
        const RobotState& current_state,
        const RuntimeStepContext& context) const override;

private:
    struct Policy {
        std::string control_group_id;
        std::int32_t priority {0};
    };

    std::unordered_map<std::string, Policy> policies_;
};

} // namespace hakoniwa::robot_runtime::runtime
