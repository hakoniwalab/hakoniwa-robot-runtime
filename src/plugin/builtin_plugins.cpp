#include "plugin/plugin_library.hpp"

#include <array>

namespace hakoniwa::robot_runtime::plugin {

std::span<const BuiltinPlugin> builtin_plugins()
{
    // Stage 2c-2 adds car and person plugins.
    static constexpr std::array<BuiltinPlugin, 0> plugins {};
    return plugins;
}

} // namespace hakoniwa::robot_runtime::plugin
