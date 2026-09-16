#pragma once

#include "runtime/controller/controller.hpp"
#include "runtime/runtime_definition.hpp"
#include "runtime/source/ackermann_drive_command_source.hpp"

#include <optional>
#include <string>

namespace hakoniwa::robot_runtime::runtime {

class AckermannController final : public IController {
public:
    explicit AckermannController(RuntimeAckermannControllerConfig config);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] std::string_view source_id() const noexcept override;
    [[nodiscard]] bool accepts(const IControllerInput& input) const noexcept override;
    [[nodiscard]] ControllerOutput update(
        std::shared_ptr<const IControllerInput> input,
        const RobotState& current_state,
        const RuntimeStepContext& context) override;
    void reset(const RobotState& current_state) override;

private:
    RuntimeAckermannControllerConfig config_;
    std::string source_id_;
    std::optional<ControllerInputMetadata> active_metadata_;
    AckermannDriveCommand active_command_;
};

} // namespace hakoniwa::robot_runtime::runtime
