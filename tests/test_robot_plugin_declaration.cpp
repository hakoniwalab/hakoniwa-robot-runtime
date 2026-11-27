// Robot plugin declaration JSON and the startup validator.
#undef NDEBUG

#include "runtime/service/robot_plugin_declaration.hpp"

#include <cassert>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace hakoniwa::robot_runtime::runtime;

namespace {

bool contains_message(const std::vector<std::string>& errors, std::string_view part)
{
    for (const auto& error : errors) {
        if (error.find(part) != std::string::npos) {
            return true;
        }
    }
    return false;
}

void expect_parse_error(std::string_view json, std::string_view part)
{
    try {
        (void)parse_robot_plugin_declaration(json);
    } catch (const std::invalid_argument& error) {
        assert(std::string_view(error.what()).find(part) != std::string_view::npos);
        return;
    }
    assert(false && "expected a parse error");
}

const std::vector<RobotServiceSpec> kServices {
    {"ArmService/ServoJ/SO101", "my_robot_srvs_msgs/ServoJ", false},
    {"ArmService/Stop/SO101", "my_robot_srvs_msgs/Stop", false},
};

// A declaration lists the service types it handles and the Plant operations it emits.
void a_full_declaration_is_read()
{
    const auto declaration = parse_robot_plugin_declaration(R"({
        "service_types": ["my_robot_srvs_msgs/ServoJ", "my_robot_srvs_msgs/Stop"],
        "plant_operations": ["surface_friction", "humanoid_motion"]
    })");
    assert(declaration.service_types
        == std::vector<std::string>({"my_robot_srvs_msgs/ServoJ", "my_robot_srvs_msgs/Stop"}));
    assert(declaration.plant_operations.surface_friction);
    assert(!declaration.plant_operations.body_motion);
    assert(declaration.plant_operations.humanoid_motion);
}

// plant_operations may be left out: the plugin emits none.
void plant_operations_are_optional()
{
    const auto declaration = parse_robot_plugin_declaration(R"({"service_types": ["a/B"]})");
    assert(declaration.service_types == std::vector<std::string> {"a/B"});
    assert(!declaration.plant_operations.surface_friction && !declaration.plant_operations.body_motion
        && !declaration.plant_operations.humanoid_motion);
}

// Malformed declarations are refused with a reason.
void malformed_declarations_are_refused()
{
    expect_parse_error("not json", "JSON");
    expect_parse_error(R"({})", "service_types");
    expect_parse_error(R"({"service_types": "a/B"})", "service_types");
    expect_parse_error(R"({"service_types": [1]})", "service_types");
    expect_parse_error(R"({"service_types": ["a/B"], "plant_operations": ["teleport"]})", "teleport");
    expect_parse_error(R"({"service_types": ["a/B"], "extra": 1})", "extra");
}

// Everything matches: no errors.
void a_matching_robot_passes()
{
    const auto declaration = parse_robot_plugin_declaration(R"({
        "service_types": ["my_robot_srvs_msgs/ServoJ", "my_robot_srvs_msgs/Stop"],
        "plant_operations": ["surface_friction"]
    })");
    const auto errors = validate_robot_plugin("SO101", declaration, kServices, {true, false, false});
    assert(errors.empty());
}

// A service whose type the plugin does not declare is reported with the robot, service and type.
void an_undeclared_service_type_is_reported()
{
    const auto declaration = parse_robot_plugin_declaration(R"({"service_types": ["my_robot_srvs_msgs/ServoJ"]})");
    const auto errors = validate_robot_plugin("SO101", declaration, kServices, {});
    assert(errors.size() == 1);
    assert(contains_message(errors, "SO101"));
    assert(contains_message(errors, "ArmService/Stop/SO101"));
    assert(contains_message(errors, "my_robot_srvs_msgs/Stop"));
}

// A Plant operation the plugin declares but the Plant cannot do is reported by name.
void an_operation_the_plant_cannot_do_is_reported()
{
    const auto declaration = parse_robot_plugin_declaration(R"({
        "service_types": ["my_robot_srvs_msgs/ServoJ", "my_robot_srvs_msgs/Stop"],
        "plant_operations": ["surface_friction", "humanoid_motion"]
    })");
    const auto errors = validate_robot_plugin("SO101", declaration, kServices, {true, true, false});
    assert(errors.size() == 1);
    assert(contains_message(errors, "humanoid_motion"));
}

// All problems are reported at once, not only the first.
void all_problems_are_reported_together()
{
    const auto declaration = parse_robot_plugin_declaration(R"({
        "service_types": [],
        "plant_operations": ["body_motion"]
    })");
    const auto errors = validate_robot_plugin("SO101", declaration, kServices, {});
    assert(errors.size() == 3);  // two undeclared types and one missing operation
}

} // namespace

int main()
{
    a_full_declaration_is_read();
    plant_operations_are_optional();
    malformed_declarations_are_refused();
    a_matching_robot_passes();
    an_undeclared_service_type_is_reported();
    an_operation_the_plant_cannot_do_is_reported();
    all_problems_are_reported_together();
    std::cout << "robot plugin declaration: all passed" << std::endl;
    return 0;
}
