#include "plugin/plugin_library.hpp"

#include <array>

extern "C" const HakoPluginHeaderV1* hako_robot_plugin_ackermann_step();
extern "C" const HakoPluginHeaderV1* hako_robot_plugin_person_step();

namespace hakoniwa::robot_runtime::plugin {

std::span<const BuiltinPlugin> builtin_plugins()
{
    static constexpr std::array plugins {
        BuiltinPlugin {"ackermann_step", hako_robot_plugin_ackermann_step},
        BuiltinPlugin {"person_step", hako_robot_plugin_person_step},
    };
    return plugins;
}

} // namespace hakoniwa::robot_runtime::plugin
