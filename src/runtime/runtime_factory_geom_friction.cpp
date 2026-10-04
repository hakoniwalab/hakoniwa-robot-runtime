#include "runtime/runtime_factory_internal.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE

#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime::detail {

bool load_geom_friction_definitions(
    RuntimeParseContext& context,
    RuntimeDefinition& definition,
    std::string* error)
{
    // A geom can be owned by only one friction component; otherwise two PDU
    // streams would silently overwrite each other inside the Plant.
    std::unordered_set<std::string> owned_geoms;
    for (const auto& component : context.components) {
        if (component.value("kind", "") != "controller"
            || component.value("type", "") != "geom_friction") {
            continue;
        }

        RuntimeGeomFrictionConfig friction;
        std::string config_value;
        if (!required_string(component, "id", friction.component_id,
                error, "geom friction component")
            || !required_string(component, "config", config_value,
                error, "geom friction component")
            || !required_string(component, "pdu_robot", friction.pdu_robot,
                error, "geom friction component")) {
            return false;
        }
        const fs::path config_path = resolve(context.base_path, config_value);
        if (!regular_file(config_path, error)) {
            return false;
        }
        friction.config_path = config_path.string();

        json root;
        if (!read_json(config_path, root, error)
            || !root.contains("spec") || !root.at("spec").is_object()
            || !root.contains("input") || !root.at("input").is_object()) {
            return fail(error,
                "invalid geom friction config: " + config_path.string());
        }
        if (root.contains("schema_version")
            && root.at("schema_version") != 1) {
            return fail(error,
                "unsupported geom friction schema_version: "
                + config_path.string());
        }

        const auto& spec = root.at("spec");
        if (!spec.contains("geoms") || !spec.at("geoms").is_array()
            || spec.at("geoms").empty()) {
            return fail(error,
                "geom friction spec.geoms must be a non-empty array: "
                + friction.component_id);
        }
        for (const auto& geom : spec.at("geoms")) {
            if (!geom.is_string() || geom.get<std::string>().empty()) {
                return fail(error,
                    "geom friction spec.geoms entries must be non-empty strings: "
                    + friction.component_id);
            }
            auto name = geom.get<std::string>();
            if (!owned_geoms.insert(name).second) {
                return fail(error,
                    "geom is referenced by more than one geom friction entry: "
                    + name);
            }
            friction.geoms.push_back(std::move(name));
        }

        std::string message_type;
        if (!required_string(root.at("input"), "pdu_name", friction.pdu_name,
                error, "geom friction input")
            || !required_string(root.at("input"), "message_type",
                message_type, error, "geom friction input")) {
            return false;
        }
        if (message_type != "std_msgs/Float64") {
            return fail(error,
                "geom friction input must use std_msgs/Float64");
        }
        if (!verify_pdu_binding(context, friction.pdu_robot,
                friction.pdu_name, "std_msgs/Float64", 32, error)) {
            return false;
        }
        definition.geom_frictions.push_back(std::move(friction));
    }
    return true;
}

} // namespace hakoniwa::robot_runtime::runtime::detail

#endif
