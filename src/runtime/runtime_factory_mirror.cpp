#include "runtime/runtime_factory_internal.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR

#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime::detail {
namespace {

bool pdu_entry(
    const json& owner,
    const char* field,
    const bool required,
    std::optional<std::string>& pdu_name,
    std::string* error)
{
    if (!owner.contains(field)) {
        if (!required) {
            return true;
        }
        return fail(error, std::string("Mirror input is missing: ") + field);
    }
    const auto& entry = owner.at(field);
    std::string name;
    std::string type;
    if (!required_string(entry, "pdu_name", name, error, "Mirror input")
        || !required_string(entry, "message_type", type, error, "Mirror input")
        || type != "geometry_msgs/Twist") {
        return fail(error,
            std::string("Mirror ") + field
            + " input must use geometry_msgs/Twist");
    }
    pdu_name = std::move(name);
    return true;
}

bool finite_number(
    const json& object,
    const char* field,
    double& output,
    std::string* error)
{
    if (!object.contains(field) || !object.at(field).is_number()) {
        return fail(error,
            std::string("Impulse policy field is missing or invalid: ") + field);
    }
    output = object.at(field).get<double>();
    return std::isfinite(output);
}

} // namespace

bool load_mirror_definitions(
    RuntimeParseContext& context,
    RuntimeDefinition& definition,
    std::string* error)
{
    std::unordered_set<std::string> mirror_ids;
    for (const auto& component : context.components) {
        if (component.value("kind", "") != "controller"
            || component.value("type", "") != "mirror_body") {
            continue;
        }

        RuntimeMirrorBodyConfig mirror;
        std::string config_value;
        if (!required_string(component, "id", mirror.component_id,
                error, "Mirror component")
            || !required_string(component, "config", config_value,
                error, "Mirror component")
            || !required_string(component, "pdu_robot", mirror.pdu_robot,
                error, "Mirror component")) {
            return false;
        }
        const fs::path config_path = resolve(context.base_path, config_value);
        if (!regular_file(config_path, error)) {
            return false;
        }
        mirror.config_path = config_path.string();

        json root;
        if (!read_json(config_path, root, error)
            || !root.contains("spec") || !root.at("spec").is_object()
            || !root.contains("input") || !root.at("input").is_object()
            || !root.contains("mjcf_binding")
            || !root.at("mjcf_binding").is_object()) {
            return fail(error,
                "invalid Mirror body config: " + config_path.string());
        }
        if (!required_string(root.at("spec"), "mirror_id", mirror.mirror_id,
                error, "Mirror spec")
            || !required_string(root.at("mjcf_binding"), "freejoint",
                mirror.mjcf_freejoint, error, "Mirror MJCF binding")) {
            return false;
        }
        if (!mirror_ids.insert(mirror.mirror_id).second) {
            return fail(error, "duplicate Mirror ID: " + mirror.mirror_id);
        }

        std::optional<std::string> pose;
        std::optional<std::string> velocity;
        if (!pdu_entry(root.at("input"), "pose", true, pose, error)
            || !pdu_entry(root.at("input"), "velocity", false, velocity, error)) {
            return false;
        }
        mirror.pose_pdu_name = *pose;
        mirror.velocity_pdu_name = velocity;
        if (mirror.velocity_pdu_name.has_value()) {
            std::string velocity_frame;
            if (!required_string(root.at("input").at("velocity"), "frame",
                    velocity_frame, error, "Mirror velocity input")) {
                return false;
            }
            if (velocity_frame == "body") {
                mirror.velocity_frame = MirrorVelocityFrame::Body;
            } else if (velocity_frame == "world") {
                mirror.velocity_frame = MirrorVelocityFrame::World;
            } else {
                return fail(error,
                    "Mirror velocity frame must be body or world");
            }
        }
        if (!verify_pdu_binding(context, mirror.pdu_robot,
                mirror.pose_pdu_name, "geometry_msgs/Twist", 72, error)
            || (mirror.velocity_pdu_name.has_value()
                && !verify_pdu_binding(context, mirror.pdu_robot,
                    *mirror.velocity_pdu_name,
                    "geometry_msgs/Twist", 72, error))) {
            return false;
        }

        const auto& binding = root.at("mjcf_binding");
        if (!binding.contains("contact_bodies")
            || !binding.at("contact_bodies").is_array()
            || binding.at("contact_bodies").empty()) {
            return fail(error,
                "Mirror MJCF binding requires non-empty contact_bodies");
        }
        std::unordered_set<std::string> body_ids;
        std::unordered_set<std::string> freejoints;
        for (const auto& body : binding.at("contact_bodies")) {
            RuntimeMirrorContactBodyBinding parsed;
            if (!required_string(body, "body_id", parsed.body_id,
                    error, "Mirror contact body")
                || !required_string(body, "mjcf_freejoint", parsed.mjcf_freejoint,
                    error, "Mirror contact body")
                || !body_ids.insert(parsed.body_id).second
                || !freejoints.insert(parsed.mjcf_freejoint).second) {
                return fail(error,
                    "Mirror contact bodies must have unique IDs and freejoints");
            }
            mirror.contact_bodies.push_back(std::move(parsed));
        }
        definition.mirror_bodies.push_back(std::move(mirror));
    }

    std::unordered_map<std::string, const RuntimeMirrorBodyConfig*> mirrors;
    for (const auto& mirror : definition.mirror_bodies) {
        mirrors.emplace(mirror.component_id, &mirror);
    }
    for (const auto& component : context.components) {
        if (component.value("kind", "") != "state_output"
            || component.value("type", "") != "impulse_collision") {
            continue;
        }
        RuntimeImpulseCollisionOutputConfig output;
        std::string config_value;
        if (!required_string(component, "id", output.component_id,
                error, "Impulse output")
            || !required_string(component, "config", config_value,
                error, "Impulse output")
            || !required_string(component, "pdu_robot", output.pdu_robot,
                error, "Impulse output")) {
            return false;
        }
        const fs::path config_path = resolve(context.base_path, config_value);
        if (!regular_file(config_path, error)) {
            return false;
        }
        output.config_path = config_path.string();
        json root;
        if (!read_json(config_path, root, error)
            || !root.contains("spec") || !root.at("spec").is_object()
            || !root.contains("pdu_config")
            || !root.at("pdu_config").is_object()
            || !root.contains("policy") || !root.at("policy").is_object()) {
            return fail(error,
                "invalid Impulse collision output config: "
                + config_path.string());
        }
        if (!required_string(root.at("spec"), "mirror_component",
                output.mirror_component_id, error, "Impulse output spec")) {
            return false;
        }
        const auto mirror = mirrors.find(output.mirror_component_id);
        if (mirror == mirrors.end()) {
            return fail(error,
                "Impulse output references unknown Mirror component: "
                + output.mirror_component_id);
        }
        output.mirror_id = mirror->second->mirror_id;

        std::string message_type;
        if (!required_string(root.at("pdu_config"), "pdu_name",
                output.pdu_name, error, "Impulse pdu_config")
            || !required_string(root.at("pdu_config"), "message_type",
                message_type, error, "Impulse pdu_config")
            || message_type != "hako_msgs/ImpulseCollision") {
            return fail(error,
                "Impulse output must use hako_msgs/ImpulseCollision");
        }
        if (!verify_pdu_binding(context, output.pdu_robot,
                output.pdu_name, "hako_msgs/ImpulseCollision", 216, error)) {
            return false;
        }

        double cooldown_sec = 0.0;
        const auto& policy = root.at("policy");
        if (!finite_number(policy, "restitution_coefficient",
                output.restitution_coefficient, error)
            || !finite_number(policy, "relative_normal_speed_threshold_mps",
                output.relative_normal_speed_threshold_mps, error)
            || !finite_number(policy, "cooldown_sec", cooldown_sec, error)
            || output.restitution_coefficient < 0.0
            || output.restitution_coefficient > 1.0
            || output.relative_normal_speed_threshold_mps < 0.0
            || cooldown_sec <= 0.0
            || cooldown_sec > static_cast<double>(
                std::numeric_limits<std::uint64_t>::max()) / 1'000'000.0) {
            return fail(error, "Impulse collision policy is outside its valid range");
        }
        const double cooldown_usec = cooldown_sec * 1'000'000.0;
        if (cooldown_usec < 1.0) {
            return fail(error, "Impulse cooldown_sec must be at least one microsecond");
        }
        output.cooldown_usec = static_cast<std::uint64_t>(
            std::llround(cooldown_usec));
        definition.impulse_collision_outputs.push_back(std::move(output));
    }
    return true;
}

} // namespace hakoniwa::robot_runtime::runtime::detail

#endif
