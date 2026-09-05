#include "runtime/runtime_factory_internal.hpp"

#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::runtime::detail {

bool load_manual_definitions(
    RuntimeParseContext& context,
    RuntimeDefinition& definition,
    std::string* error)
{
    std::unordered_map<std::string, std::string> actuator_by_control_joint;
    for (const auto& controller : definition.trajectory_controllers) {
        for (const auto& binding : controller.joints) {
            const auto [iterator, inserted] = actuator_by_control_joint.emplace(
                binding.joint_name, binding.actuator_id);
            if (!inserted && iterator->second != binding.actuator_id) {
                return fail(error,
                    "controller joint maps to multiple actuators: "
                    + binding.joint_name);
            }
        }
    }

    for (const auto& component : context.components) {
        if (component.value("kind", "") != "controller"
            || component.value("type", "") != "joy_manual_controller") {
            continue;
        }

        RuntimeManualControllerConfig controller;
        std::string config_value;
        if (!required_string(component, "id", controller.component_id,
                error, "Manual Controller")
            || !required_string(component, "config", config_value,
                error, "Manual Controller")
            || !required_string(component, "pdu_robot", controller.pdu_robot,
                error, "Manual Controller")) {
            return false;
        }

        const fs::path config_path = resolve(context.base_path, config_value);
        if (!regular_file(config_path, error)) {
            return false;
        }
        controller.config_path = config_path.string();

        json controller_root;
        if (!read_json(config_path, controller_root, error)
            || !controller_root.contains("input")
            || !controller_root.at("input").is_object()
            || !controller_root.contains("spec")
            || !controller_root.at("spec").is_object()) {
            return fail(error,
                "invalid Joy Manual Controller config: "
                + config_path.string());
        }

        const auto& input = controller_root.at("input");
        std::string message_type;
        if (!required_string(input, "pdu_name", controller.pdu_name,
                error, "Manual Controller input")
            || !required_string(input, "message_type", message_type,
                error, "Manual Controller input")
            || message_type != "sensor_msgs/Joy") {
            return fail(error,
                "Manual Controller input message_type must be sensor_msgs/Joy");
        }
        if (!input.contains("update_rate_hz")
            || !input.at("update_rate_hz").is_number()
            || !input.contains("timeout_sec")
            || !input.at("timeout_sec").is_number()) {
            return fail(error,
                "Manual Controller update_rate_hz/timeout_sec is missing or invalid");
        }

        controller.update_rate_hz = input.at("update_rate_hz").get<double>();
        const double timeout_sec = input.at("timeout_sec").get<double>();
        constexpr double usec_per_sec = 1'000'000.0;
        const double timeout_value = timeout_sec * usec_per_sec;
        if (!std::isfinite(controller.update_rate_hz)
            || controller.update_rate_hz <= 0.0
            || !std::isfinite(timeout_value)
            || timeout_value < 1.0
            || timeout_value > static_cast<double>(
                std::numeric_limits<std::int64_t>::max())) {
            return fail(error,
                "Manual Controller update rate or timeout is outside its valid range");
        }
        controller.input_timeout_usec =
            static_cast<std::uint64_t>(std::llround(timeout_value));

        std::string layout_value;
        if (!required_string(controller_root, "joy_layout", layout_value,
                error, "Manual Controller")) {
            return false;
        }
        const fs::path layout_path = resolve(
            config_path.parent_path(), layout_value);
        if (!regular_file(layout_path, error)) {
            return false;
        }

        json layout;
        if (!read_json(layout_path, layout, error)
            || layout.value("schema_version", 0) != 1
            || !layout.contains("axes")
            || !layout.at("axes").is_array()
            || !layout.contains("buttons")
            || !layout.at("buttons").is_array()) {
            return fail(error,
                "invalid logical Joy layout: " + layout_path.string());
        }

        std::unordered_map<std::string, std::size_t> axis_indices;
        std::unordered_map<std::string, std::size_t> button_indices;
        for (std::size_t index = 0; index < layout.at("axes").size(); ++index) {
            if (!layout.at("axes").at(index).is_string()) {
                return fail(error,
                    "Joy layout axis names must be strings");
            }
            const auto name =
                layout.at("axes").at(index).get<std::string>();
            if (name.empty() || !axis_indices.emplace(name, index).second) {
                return fail(error,
                    "Joy layout axis names must be non-empty and unique");
            }
        }
        for (std::size_t index = 0;
             index < layout.at("buttons").size();
             ++index) {
            if (!layout.at("buttons").at(index).is_string()) {
                return fail(error,
                    "Joy layout button names must be strings");
            }
            const auto name =
                layout.at("buttons").at(index).get<std::string>();
            if (name.empty() || !button_indices.emplace(name, index).second) {
                return fail(error,
                    "Joy layout button names must be non-empty and unique");
            }
        }
        controller.joy_axis_count = axis_indices.size();
        controller.joy_button_count = button_indices.size();

        const auto& spec = controller_root.at("spec");
        std::string manual_enable_button;
        std::string quit_button;
        if (!required_string(spec, "manual_enable_button", manual_enable_button,
                error, "Manual Controller spec")
            || !required_string(spec, "quit_button", quit_button,
                error, "Manual Controller spec")
            || !button_indices.contains(manual_enable_button)
            || !button_indices.contains(quit_button)) {
            return fail(error,
                "Manual enable/quit button is not present in the Joy layout");
        }
        controller.manual_enable_button_index =
            button_indices.at(manual_enable_button);

        if (!spec.contains("deadzone") || !spec.at("deadzone").is_number()
            || !spec.contains("expo") || !spec.at("expo").is_number()
            || !spec.contains("banks") || !spec.at("banks").is_array()
            || spec.at("banks").empty()) {
            return fail(error,
                "Manual Controller spec is incomplete");
        }
        controller.deadzone = spec.at("deadzone").get<double>();
        controller.expo = spec.at("expo").get<double>();
        if (!std::isfinite(controller.deadzone)
            || !std::isfinite(controller.expo)
            || controller.deadzone < 0.0
            || controller.deadzone >= 1.0
            || controller.expo < 0.0
            || controller.expo > 1.0) {
            return fail(error,
                "Manual Controller deadzone/expo is invalid");
        }

        std::unordered_set<std::string> bank_ids;
        std::size_t default_banks = 0;
        for (const auto& bank : spec.at("banks")) {
            RuntimeManualBankConfig parsed_bank;
            if (!required_string(bank, "id", parsed_bank.id,
                    error, "Manual Controller bank")
                || !bank_ids.insert(parsed_bank.id).second
                || !bank.contains("select_button")
                || (!bank.at("select_button").is_null()
                    && !bank.at("select_button").is_string())
                || !bank.contains("bindings")
                || !bank.at("bindings").is_array()
                || bank.at("bindings").empty()) {
                return fail(error,
                    "Manual Controller bank is invalid");
            }

            if (bank.at("select_button").is_null()) {
                ++default_banks;
            } else {
                const auto selector =
                    bank.at("select_button").get<std::string>();
                if (!button_indices.contains(selector)) {
                    return fail(error,
                        "Manual bank selector is not present in the Joy layout: "
                        + selector);
                }
                parsed_bank.select_button_index =
                    button_indices.at(selector);
            }

            for (const auto& binding : bank.at("bindings")) {
                RuntimeManualAxisBinding parsed;
                std::string axis;
                if (!required_string(binding, "axis", axis,
                        error, "Manual axis binding")
                    || !required_string(binding, "joint", parsed.joint_name,
                        error, "Manual axis binding")
                    || !axis_indices.contains(axis)
                    || !actuator_by_control_joint.contains(parsed.joint_name)
                    || !binding.contains("velocity_rad_s")
                    || !binding.at("velocity_rad_s").is_number()) {
                    return fail(error,
                        "Manual axis binding is invalid");
                }
                parsed.axis_index = axis_indices.at(axis);
                parsed.actuator_id =
                    actuator_by_control_joint.at(parsed.joint_name);
                parsed.velocity_per_sec =
                    binding.at("velocity_rad_s").get<double>();
                parsed.invert = binding.value("invert", false);
                if (!std::isfinite(parsed.velocity_per_sec)
                    || parsed.velocity_per_sec <= 0.0) {
                    return fail(error,
                        "Manual joint velocity must be finite and positive");
                }
                parsed_bank.bindings.push_back(std::move(parsed));
            }
            controller.banks.push_back(std::move(parsed_bank));
        }

        if (default_banks != 1) {
            return fail(error,
                "Manual Controller requires exactly one default bank");
        }

        const std::size_t minimum_joy_size = 24 + 152
            + 4 * axis_indices.size() + 4 * button_indices.size();
        if (!verify_pdu_binding(
                context,
                controller.pdu_robot,
                controller.pdu_name,
                "sensor_msgs/Joy",
                minimum_joy_size,
                error)) {
            return false;
        }

        definition.manual_controllers.push_back(std::move(controller));
    }

    return true;
}

} // namespace hakoniwa::robot_runtime::runtime::detail
