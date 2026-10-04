#include "runtime/controller/geom_friction_controller.hpp"

#include "runtime/source/scalar_pdu_command_source.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

GeomFrictionController::GeomFrictionController(
    std::string controller_id,
    std::string source_id,
    std::vector<std::string> geom_names)
    : controller_id_(std::move(controller_id))
    , source_id_(std::move(source_id))
    , geom_names_(std::move(geom_names))
{
    if (controller_id_.empty() || source_id_.empty()) {
        throw std::invalid_argument(
            "geom friction Controller requires controller and source IDs");
    }
    if (geom_names_.empty()) {
        throw std::invalid_argument(
            "geom friction Controller requires at least one geom");
    }
}

std::string_view GeomFrictionController::id() const noexcept
{
    return controller_id_;
}

std::string_view GeomFrictionController::source_id() const noexcept
{
    return source_id_;
}

bool GeomFrictionController::accepts(
    const IControllerInput& input) const noexcept
{
    return input.type_name() == "std_msgs/Float64";
}

DirectiveControllerOutput GeomFrictionController::update(
    std::shared_ptr<const IControllerInput> input,
    const RuntimeStepContext& context)
{
    if (input == nullptr) {
        // The Plant retains the last applied friction, so no new sample keeps
        // the controller Ready once a value has been applied.
        return {controller_id_,
            {controller_id_,
                applied_ ? ComponentState::Ready : ComponentState::WaitingForInput,
                {}},
            {}};
    }
    const auto scalar = std::dynamic_pointer_cast<const ScalarPduInput>(input);
    if (scalar == nullptr || scalar->metadata().source_id != source_id_) {
        return {controller_id_,
            {controller_id_, ComponentState::Error,
                "invalid geom friction input"},
            {}};
    }
    const double friction = scalar->value();
    if (!std::isfinite(friction) || friction < 0.0) {
        return {controller_id_,
            {controller_id_, ComponentState::Degraded,
                "geom friction must be a finite non-negative value"},
            {}};
    }
    applied_ = true;
    return {
        controller_id_,
        {controller_id_, ComponentState::Ready, {}},
        {std::make_shared<const GeomFrictionDirective>(
            geom_names_, friction, context.simulation_time_usec)},
    };
}

void GeomFrictionController::reset() noexcept
{
    applied_ = false;
}

} // namespace hakoniwa::robot_runtime::runtime
