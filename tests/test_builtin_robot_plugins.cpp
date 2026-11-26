// Stage 2c-2: car and person are builtin robot plugins, listed by name and
// loaded through the same validation as a shared library. Written before the
// implementation. Their behaviour is checked by the existing car and person
// tests (Urban), which must stay byte-identical.
#undef NDEBUG

#include "plugin/plugin_library.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>
#include <vector>

using namespace hakoniwa::robot_runtime;

int main()
{
    std::vector<std::string> names;
    for (const auto& builtin : plugin::builtin_plugins()) {
        names.emplace_back(builtin.name);
    }
    std::sort(names.begin(), names.end());
    assert(names == std::vector<std::string>({"ackermann_step", "person_step"}));

    for (const auto& name : names) {
        const auto library = plugin::PluginLibrary::open_builtin(name, HAKO_PLUGIN_KIND_SERVICE);
        assert(library->path() == "builtin:" + name);
        const auto& table = library->service_table();
        assert(table.create && table.destroy && table.on_request && table.on_step && table.on_response
            && table.reset);
        assert(library->plugin_id() != "unknown");
    }
    std::cout << "builtin robot plugins: all passed" << std::endl;
    return 0;
}
