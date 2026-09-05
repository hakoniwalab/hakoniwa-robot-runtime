#pragma once

#include "runtime/controller/controller.hpp"
#include "runtime/source/scalar_pdu_command_source.hpp"

#include <optional>
#include <string>

namespace hakoniwa::robot_runtime::runtime {

/** One-to-one controller from a scalar PDU sample to one actuator candidate. */
class ScalarPduCommandController final : public IController {
public:
    ScalarPduCommandController(
        std::string controller_id,
        std::string source_id,
        std::string actuator_id,
        ActuatorCommandType command_type);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] std::string_view source_id() const noexcept override;
    [[nodiscard]] bool accepts(const IControllerInput& input) const noexcept override;
    [[nodiscard]] ControllerOutput update(
        std::shared_ptr<const IControllerInput> input,
        const RobotState& current_state,
        const RuntimeStepContext& context) override;
    void reset(const RobotState& current_state) override;

private:
    std::string controller_id_;
    std::string source_id_;
    std::string actuator_id_;
    ActuatorCommandType command_type_ {ActuatorCommandType::Position};
    std::optional<ControllerInputMetadata> active_metadata_;
    double active_value_ {0.0};
};

} // namespace hakoniwa::robot_runtime::runtime
