#pragma once

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE

#include "runtime/plant/plant_directive.hpp"
#include "runtime/source/command_source.hpp"

#include <memory>
#include <string>
#include <string_view>

namespace hakoniwa::robot_runtime::runtime {

struct DirectiveControllerOutput {
    std::string controller_id;
    ComponentStatus status;
    PlantDirectiveList directives;
};

/**
 * Non-arbitrated Controller on the Plant Directive path.
 *
 * It turns the input of its bound CommandSource into backend-independent
 * IPlantDirectives. Its output type differs from ControllerOutput, so the
 * directives structurally cannot enter the Arbiter. A null input means that
 * the bound source had no new sample this step.
 *
 * Calls are serialized by ActuatorRuntime; implementations need not be
 * thread-safe.
 */
class IDirectiveController {
public:
    virtual ~IDirectiveController() = default;
    [[nodiscard]] virtual std::string_view id() const noexcept = 0;
    [[nodiscard]] virtual std::string_view source_id() const noexcept = 0;
    [[nodiscard]] virtual bool accepts(const IControllerInput& input) const noexcept = 0;
    [[nodiscard]] virtual DirectiveControllerOutput update(
        std::shared_ptr<const IControllerInput> input,
        const RuntimeStepContext& context) = 0;
    virtual void reset() noexcept = 0;
};

} // namespace hakoniwa::robot_runtime::runtime

#endif
