#include "runtime/controller/ackermann_controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {
namespace {

struct Targets {
    double steering_left {0.0};
    double steering_right {0.0};
    double drive_left {0.0};
    double drive_right {0.0};
};

Targets resolve_targets(
    const AckermannDriveCommand& command,
    const RuntimeAckermannGeometry& geometry)
{
    const double delta = std::clamp(
        command.steering_angle_rad,
        -geometry.max_steering_angle_rad,
        geometry.max_steering_angle_rad);
    const double base_wheel_velocity = std::clamp(
        command.speed_m_s / geometry.wheel_radius_m,
        -geometry.max_wheel_angular_velocity_rad_s,
        geometry.max_wheel_angular_velocity_rad_s);

    Targets targets {};
    targets.drive_left = base_wheel_velocity;
    targets.drive_right = base_wheel_velocity;
    if (std::abs(delta) < 1.0e-9) {
        return targets;
    }

    const double radius = geometry.wheelbase_m / std::tan(delta);
    targets.steering_left = std::atan2(
        geometry.wheelbase_m,
        radius - geometry.track_width_m / 2.0);
    targets.steering_right = std::atan2(
        geometry.wheelbase_m,
        radius + geometry.track_width_m / 2.0);
    if (delta < 0.0) {
        if (targets.steering_left > 0.0) {
            targets.steering_left -= std::numbers::pi;
        }
        if (targets.steering_right > 0.0) {
            targets.steering_right -= std::numbers::pi;
        }
    }
    targets.steering_left = std::clamp(
        targets.steering_left,
        -geometry.max_steering_angle_rad,
        geometry.max_steering_angle_rad);
    targets.steering_right = std::clamp(
        targets.steering_right,
        -geometry.max_steering_angle_rad,
        geometry.max_steering_angle_rad);

    const double left_scale = 1.0 - geometry.track_width_m / (2.0 * radius);
    const double right_scale = 1.0 + geometry.track_width_m / (2.0 * radius);
    targets.drive_left = std::clamp(
        base_wheel_velocity * left_scale,
        -geometry.max_wheel_angular_velocity_rad_s,
        geometry.max_wheel_angular_velocity_rad_s);
    targets.drive_right = std::clamp(
        base_wheel_velocity * right_scale,
        -geometry.max_wheel_angular_velocity_rad_s,
        geometry.max_wheel_angular_velocity_rad_s);
    return targets;
}

} // namespace

AckermannController::AckermannController(RuntimeAckermannControllerConfig config)
    : config_(std::move(config))
    , source_id_("ackermann-pdu:" + config_.component_id)
{
    const auto& geometry = config_.geometry;
    const auto& actuators = config_.actuators;
    if (config_.component_id.empty()
        || geometry.wheelbase_m <= 0.0
        || geometry.track_width_m <= 0.0
        || geometry.wheel_radius_m <= 0.0
        || geometry.max_steering_angle_rad <= 0.0
        || geometry.max_wheel_angular_velocity_rad_s <= 0.0
        || actuators.steering_left.empty()
        || actuators.steering_right.empty()
        || actuators.drive_left.empty()
        || actuators.drive_right.empty()) {
        throw std::invalid_argument("invalid Ackermann controller configuration");
    }
}

std::string_view AckermannController::id() const noexcept
{
    return config_.component_id;
}

std::string_view AckermannController::source_id() const noexcept
{
    return source_id_;
}

bool AckermannController::accepts(const IControllerInput& input) const noexcept
{
    return input.type_name() == "ackermann_msgs/AckermannDrive";
}

ControllerOutput AckermannController::update(
    std::shared_ptr<const IControllerInput> input,
    const RobotState& current_state,
    const RuntimeStepContext& context)
{
    (void)current_state;
    ControllerOutput output {
        config_.component_id,
        {config_.component_id, ComponentState::WaitingForInput, {}},
        {},
    };
    if (input != nullptr) {
        const auto drive = std::dynamic_pointer_cast<const AckermannDriveInput>(input);
        if (drive == nullptr || drive->metadata().source_id != source_id_) {
            output.status = {config_.component_id, ComponentState::Error,
                "invalid Ackermann controller input"};
            return output;
        }
        const auto& command = drive->command();
        if (!std::isfinite(command.steering_angle_rad)
            || !std::isfinite(command.speed_m_s)) {
            output.status = {config_.component_id, ComponentState::Degraded,
                "Ackermann command contains non-finite primary values"};
            return output;
        }
        active_metadata_ = drive->metadata();
        active_command_ = command;
    }
    if (!active_metadata_.has_value()
        || active_metadata_->created_at_usec > context.simulation_time_usec
        || (active_metadata_->expires_at_usec.has_value()
            && context.simulation_time_usec >= *active_metadata_->expires_at_usec)) {
        active_metadata_.reset();
        return output;
    }

    const auto targets = resolve_targets(active_command_, config_.geometry);
    const auto& metadata = *active_metadata_;
    const auto make_command = [&](
        const std::string& actuator,
        const ActuatorCommandType type,
        const double value) {
        return ActuatorCommand {
            actuator,
            type,
            value,
            source_id_,
            metadata.created_at_usec,
            metadata.expires_at_usec,
        };
    };
    output.status.state = ComponentState::Ready;
    output.commands = {
        make_command(config_.actuators.steering_left,
            ActuatorCommandType::Position, targets.steering_left),
        make_command(config_.actuators.steering_right,
            ActuatorCommandType::Position, targets.steering_right),
        make_command(config_.actuators.drive_left,
            ActuatorCommandType::Velocity, targets.drive_left),
        make_command(config_.actuators.drive_right,
            ActuatorCommandType::Velocity, targets.drive_right),
    };
    return output;
}

void AckermannController::reset(const RobotState& current_state)
{
    (void)current_state;
    active_metadata_.reset();
    active_command_ = {};
}

} // namespace hakoniwa::robot_runtime::runtime
