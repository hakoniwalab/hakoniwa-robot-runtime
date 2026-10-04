#pragma once

#include "runtime/controller/directive_controller.hpp"

#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

/**
 * Converts a std_msgs/Float64 sliding-friction value into one
 * GeomFrictionDirective for a fixed set of geoms.
 *
 * A directive is emitted only when a new value arrives. The Plant keeps the
 * applied friction until the next directive, so a missing sample is not a
 * fallback trigger. Time synchronization comes from the Runtime reading the
 * latest staged value at the step start: a value written during one Hakoniwa
 * step takes effect from the next physics step.
 */
class GeomFrictionController final : public IDirectiveController {
public:
    GeomFrictionController(
        std::string controller_id,
        std::string source_id,
        std::vector<std::string> geom_names);

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
    std::vector<std::string> geom_names_;
    bool applied_ {false};
};

} // namespace hakoniwa::robot_runtime::runtime
