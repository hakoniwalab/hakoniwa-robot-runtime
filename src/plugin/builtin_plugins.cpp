#include "plugin/plugin_library.hpp"

#include <array>

extern "C" const HakoPluginHeaderV1* hako_robot_plugin_ackermann_step();
extern "C" const HakoPluginHeaderV1* hako_robot_plugin_person_step();

namespace hakoniwa::robot_runtime::plugin {

std::span<const BuiltinPlugin> builtin_plugins()
{
    static constexpr std::array plugins {
        BuiltinPlugin {"ackermann_step", hako_robot_plugin_ackermann_step,
            R"({"service_types":["hako_srv_msgs/AckermannStep"],"plant_operations":["surface_friction"]})"},
        BuiltinPlugin {"person_step", hako_robot_plugin_person_step,
            R"({"service_types":["hako_srv_msgs/PersonStep"],"plant_operations":["humanoid_motion"]})"},
    };
    return plugins;
}

} // namespace hakoniwa::robot_runtime::plugin
