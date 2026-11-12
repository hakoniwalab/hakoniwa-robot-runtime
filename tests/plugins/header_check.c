/* Compiles the plugin headers as strict C99 (no C++ only constructs). */
#include "hakoniwa/robot_runtime/plugin/controller_plugin.h"

const struct HakoTickV1* header_check_tick_tag(const HakoTickV1* tick);

const struct HakoTickV1* header_check_tick_tag(const HakoTickV1* tick)
{
    return tick;
}
