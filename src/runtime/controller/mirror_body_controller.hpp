#pragma once

#include "runtime/controller/directive_controller.hpp"
#include "runtime/source/mirror_body_state_source.hpp"

#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace hakoniwa::robot_runtime::runtime {

/**
 * Converts Mirror input into a BodyMotionOperation. As a directive
 * controller it never produces an arbitration candidate.
 */
class MirrorBodyController final : public IDirectiveController {
public:
    MirrorBodyController(
        std::string controller_id,
        std::string source_id,
        std::string mirror_id,
        MirrorVelocityFrame velocity_frame = MirrorVelocityFrame::World);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] std::string_view source_id() const noexcept override;
    [[nodiscard]] bool accepts(const IControllerInput& input) const noexcept override;
    [[nodiscard]] DirectiveControllerOutput update(
        std::shared_ptr<const IControllerInput> input,
        const RuntimeStepContext& context) override;
    void reset() noexcept override;

private:
    std::string controller_id_;
    std::string source_id_;
    std::string mirror_id_;
    MirrorVelocityFrame velocity_frame_ {MirrorVelocityFrame::World};
    std::optional<Vector3State> previous_position_;
    std::optional<std::uint64_t> previous_position_time_usec_;
    std::optional<EulerState> previous_orientation_;
    std::optional<std::uint64_t> previous_orientation_time_usec_;
};

} // namespace hakoniwa::robot_runtime::runtime
