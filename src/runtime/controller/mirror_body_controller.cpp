#include "runtime/controller/mirror_body_controller.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {
namespace {

bool finite(const Vector3State& value) noexcept
{
    return std::isfinite(value.x)
        && std::isfinite(value.y)
        && std::isfinite(value.z);
}

bool finite(const EulerState& value) noexcept
{
    return std::isfinite(value.roll)
        && std::isfinite(value.pitch)
        && std::isfinite(value.yaw);
}

double wrapped_delta(const double current, const double previous) noexcept
{
    return std::remainder(current - previous, 2.0 * std::numbers::pi);
}

Vector3State difference(
    const Vector3State& current,
    const Vector3State& previous,
    const double seconds) noexcept
{
    return {
        (current.x - previous.x) / seconds,
        (current.y - previous.y) / seconds,
        (current.z - previous.z) / seconds,
    };
}

Vector3State angular_difference(
    const EulerState& current,
    const EulerState& previous,
    const double seconds) noexcept
{
    return {
        wrapped_delta(current.roll, previous.roll) / seconds,
        wrapped_delta(current.pitch, previous.pitch) / seconds,
        wrapped_delta(current.yaw, previous.yaw) / seconds,
    };
}

bool same(const Vector3State& lhs, const Vector3State& rhs) noexcept
{
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool same(const EulerState& lhs, const EulerState& rhs) noexcept
{
    return lhs.roll == rhs.roll
        && lhs.pitch == rhs.pitch
        && lhs.yaw == rhs.yaw;
}

Vector3State rotate_body_to_world(
    const Vector3State& value,
    const EulerState& euler) noexcept
{
    const double cr = std::cos(euler.roll);
    const double sr = std::sin(euler.roll);
    const double cp = std::cos(euler.pitch);
    const double sp = std::sin(euler.pitch);
    const double cy = std::cos(euler.yaw);
    const double sy = std::sin(euler.yaw);
    return {
        (cy * cp) * value.x
            + (cy * sp * sr - sy * cr) * value.y
            + (cy * sp * cr + sy * sr) * value.z,
        (sy * cp) * value.x
            + (sy * sp * sr + cy * cr) * value.y
            + (sy * sp * cr - cy * sr) * value.z,
        (-sp) * value.x + (cp * sr) * value.y + (cp * cr) * value.z,
    };
}

Vector3State rotate_world_to_body(
    const Vector3State& value,
    const EulerState& euler) noexcept
{
    const double cr = std::cos(euler.roll);
    const double sr = std::sin(euler.roll);
    const double cp = std::cos(euler.pitch);
    const double sp = std::sin(euler.pitch);
    const double cy = std::cos(euler.yaw);
    const double sy = std::sin(euler.yaw);
    return {
        (cy * cp) * value.x + (sy * cp) * value.y + (-sp) * value.z,
        (cy * sp * sr - sy * cr) * value.x
            + (sy * sp * sr + cy * cr) * value.y
            + (cp * sr) * value.z,
        (cy * sp * cr + sy * sr) * value.x
            + (sy * sp * cr - cy * sr) * value.y
            + (cp * cr) * value.z,
    };
}

} // namespace

MirrorBodyController::MirrorBodyController(
    std::string controller_id,
    std::string source_id,
    std::string mirror_id,
    const MirrorVelocityFrame velocity_frame)
    : controller_id_(std::move(controller_id))
    , source_id_(std::move(source_id))
    , mirror_id_(std::move(mirror_id))
    , velocity_frame_(velocity_frame)
{
    if (controller_id_.empty() || source_id_.empty() || mirror_id_.empty()) {
        throw std::invalid_argument(
            "Mirror Controller requires controller, source, and Mirror IDs");
    }
}

std::string_view MirrorBodyController::id() const noexcept
{
    return controller_id_;
}

std::string_view MirrorBodyController::source_id() const noexcept
{
    return source_id_;
}

bool MirrorBodyController::accepts(
    const IControllerInput& input) const noexcept
{
    return input.type_name() == "hakoniwa/MirrorBodyState";
}

MirrorControllerOutput MirrorBodyController::update(
    std::shared_ptr<const IControllerInput> input,
    const RuntimeStepContext& context)
{
    if (input == nullptr) {
        return {controller_id_,
            {controller_id_, ComponentState::WaitingForInput, {}},
            std::nullopt};
    }
    const auto typed = std::dynamic_pointer_cast<const MirrorBodyStateInput>(input);
    if (typed == nullptr) {
        return {controller_id_,
            {controller_id_, ComponentState::Error, "invalid Mirror input"},
            std::nullopt};
    }
    const auto& sample = typed->sample();
    if (!finite(sample.position) || !finite(sample.orientation)
        || (sample.linear_velocity.has_value()
            && !finite(*sample.linear_velocity))
        || (sample.angular_velocity.has_value()
            && !finite(*sample.angular_velocity))) {
        return {controller_id_,
            {controller_id_, ComponentState::Degraded,
                "Mirror input contains a non-finite value"},
            std::nullopt};
    }

    Vector3State linear {};
    Vector3State angular {};
    if (sample.linear_velocity.has_value()) {
        linear = velocity_frame_ == MirrorVelocityFrame::Body
            ? rotate_body_to_world(*sample.linear_velocity, sample.orientation)
            : *sample.linear_velocity;
        previous_position_ = sample.position;
        previous_position_time_usec_ = context.simulation_time_usec;
    } else if (previous_position_.has_value()
        && previous_position_time_usec_.has_value()
        && !same(sample.position, *previous_position_)
        && context.simulation_time_usec > *previous_position_time_usec_) {
        const double seconds = static_cast<double>(
            context.simulation_time_usec - *previous_position_time_usec_)
            / 1'000'000.0;
        linear = difference(
            sample.position, *previous_position_, seconds);
        previous_position_ = sample.position;
        previous_position_time_usec_ = context.simulation_time_usec;
    } else if (!previous_position_.has_value()) {
        previous_position_ = sample.position;
        previous_position_time_usec_ = context.simulation_time_usec;
    }
    if (sample.angular_velocity.has_value()) {
        // MuJoCo freejoint translation is world-frame while its rotational
        // velocity is body-frame. Normalize both parts to that convention.
        angular = velocity_frame_ == MirrorVelocityFrame::World
            ? rotate_world_to_body(*sample.angular_velocity, sample.orientation)
            : *sample.angular_velocity;
        previous_orientation_ = sample.orientation;
        previous_orientation_time_usec_ = context.simulation_time_usec;
    } else if (previous_orientation_.has_value()
        && previous_orientation_time_usec_.has_value()
        && !same(sample.orientation, *previous_orientation_)
        && context.simulation_time_usec > *previous_orientation_time_usec_) {
        const double seconds = static_cast<double>(
            context.simulation_time_usec - *previous_orientation_time_usec_)
            / 1'000'000.0;
        angular = angular_difference(
            sample.orientation, *previous_orientation_, seconds);
        previous_orientation_ = sample.orientation;
        previous_orientation_time_usec_ = context.simulation_time_usec;
    } else if (!previous_orientation_.has_value()) {
        previous_orientation_ = sample.orientation;
        previous_orientation_time_usec_ = context.simulation_time_usec;
    }

    return {
        controller_id_,
        {controller_id_, ComponentState::Ready, {}},
        MirrorBodyCommand {
            mirror_id_,
            sample.position,
            sample.orientation,
            linear,
            angular,
            context.simulation_time_usec,
        },
    };
}

void MirrorBodyController::reset() noexcept
{
    previous_position_.reset();
    previous_position_time_usec_.reset();
    previous_orientation_.reset();
    previous_orientation_time_usec_.reset();
}

} // namespace hakoniwa::robot_runtime::runtime
