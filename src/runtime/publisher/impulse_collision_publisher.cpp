#include "runtime/publisher/impulse_collision_publisher.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

ImpulseCollisionPublisher::ImpulseCollisionPublisher(
    std::string component_id,
    std::string mirror_id,
    ImpulseCollisionPolicy policy,
    std::shared_ptr<IImpulseCollisionWriter> writer)
    : component_id_(std::move(component_id))
    , mirror_id_(std::move(mirror_id))
    , policy_(policy)
    , writer_(std::move(writer))
{
    if (component_id_.empty() || mirror_id_.empty() || writer_ == nullptr) {
        throw std::invalid_argument(
            "Impulse publisher requires component, Mirror, and writer");
    }
    if (!std::isfinite(policy_.restitution_coefficient)
        || policy_.restitution_coefficient < 0.0
        || policy_.restitution_coefficient > 1.0
        || !std::isfinite(policy_.relative_normal_speed_threshold_mps)
        || policy_.relative_normal_speed_threshold_mps < 0.0
        || policy_.cooldown_usec == 0) {
        throw std::invalid_argument("invalid Impulse collision policy");
    }
}

std::string_view ImpulseCollisionPublisher::id() const noexcept
{
    return component_id_;
}

ComponentStatus ImpulseCollisionPublisher::publish(
    const RobotState& state,
    const RuntimeStepContext& context) noexcept
{
    try {
        if (state.sample_time_usec != context.simulation_time_usec) {
            return {component_id_, ComponentState::Error,
                "RobotState sample time does not match runtime context"};
        }

        std::unordered_map<std::string_view, const MirrorContactState*> deepest;
        for (const auto& contact : state.mirror_contacts) {
            if (contact.mirror_id != mirror_id_
                || contact.local_body_id.empty()) {
                continue;
            }
            const auto iterator = deepest.find(contact.local_body_id);
            if (iterator == deepest.end()
                || contact.distance_m < iterator->second->distance_m) {
                deepest[contact.local_body_id] = &contact;
            }
        }

        std::unordered_set<std::string_view> active_pairs;
        active_pairs.reserve(deepest.size());
        for (const auto& [local_body_id, contact] : deepest) {
            (void)contact;
            active_pairs.insert(local_body_id);
        }
        for (auto& [local_body_id, pair] : pair_states_) {
            if (!active_pairs.contains(local_body_id)) {
                pair.contact_active = false;
            }
        }

        const MirrorContactState* selected = nullptr;
        for (const auto& [local_body_id, contact] : deepest) {
            auto& pair = pair_states_[std::string(local_body_id)];
            const bool was_active = pair.contact_active;
            pair.contact_active = true;
            if (was_active
                || contact->relative_normal_speed_mps
                    < policy_.relative_normal_speed_threshold_mps) {
                continue;
            }
            if (pair.last_emit_time_usec.has_value()) {
                const auto elapsed = context.simulation_time_usec
                    >= *pair.last_emit_time_usec
                    ? context.simulation_time_usec - *pair.last_emit_time_usec
                    : 0;
                if (elapsed < policy_.cooldown_usec) {
                    continue;
                }
            }
            if (selected == nullptr
                || contact->distance_m < selected->distance_m) {
                selected = contact;
            }
        }

        if (selected == nullptr) {
            return {component_id_, ComponentState::Ready, {}};
        }

        const ImpulseCollisionOutput output {
            selected->mirror_id,
            selected->local_body_id,
            true,
            false,
            policy_.restitution_coefficient,
            selected->self_contact_vector,
            selected->normal,
            selected->target_contact_vector,
            selected->target_velocity,
            selected->target_angular_velocity,
            selected->target_euler,
            selected->target_inertia,
            selected->target_mass,
        };
        const bool sent = writer_->send(output);
        pair_states_.at(selected->local_body_id).last_emit_time_usec =
            context.simulation_time_usec;
        return sent
            ? ComponentStatus {component_id_, ComponentState::Ready, {}}
            : ComponentStatus {component_id_, ComponentState::Degraded,
                "failed to send Impulse collision output"};
    } catch (const std::exception& exception) {
        return {component_id_, ComponentState::Error, exception.what()};
    } catch (...) {
        return {component_id_, ComponentState::Error,
            "unknown Impulse publisher failure"};
    }
}

void ImpulseCollisionPublisher::reset() noexcept
{
    pair_states_.clear();
    writer_->reset();
}

} // namespace hakoniwa::robot_runtime::runtime
