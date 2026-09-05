#pragma once

#include "runtime/controller/controller.hpp"

#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

struct HoldActuatorBinding {
    std::string actuator_id;
};

/**
 * Lowest-priority position-hold Controller.
 *
 * Hold has no external CommandSource. While another control is selected, Hold
 * prepares takeover targets from that control's previous selected position
 * commands, falling back to the current RobotState for actuators without such
 * a command. Once Hold was selected in the previous completed step, it keeps
 * those captured targets fixed until another control is selected.
 */
class HoldController final : public IController {
public:
    HoldController(
        std::string controller_id,
        std::vector<HoldActuatorBinding> actuators);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] std::string_view source_id() const noexcept override;
    [[nodiscard]] bool accepts(const IControllerInput& input) const noexcept override;
    [[nodiscard]] ControllerOutput update(
        std::shared_ptr<const IControllerInput> input,
        const RobotState& current_state,
        const RuntimeStepContext& context) override;
    void reset(const RobotState& current_state) override;

private:
    void capture_targets(
        const RobotState& current_state,
        const std::vector<ActuatorCommand>& previous_commands);

    std::string controller_id_;
    std::vector<HoldActuatorBinding> actuators_;
    std::unordered_map<std::string, double> targets_;
};

} // namespace hakoniwa::robot_runtime::runtime
