#include "runtime/sensor/sensor_registry.hpp"

#include <algorithm>

namespace hakoniwa::robot_runtime::runtime::sensor {

namespace sa = ::hakoniwa::sensor_adapter;

bool SensorImplementationRegistry::add(
    const sa::SensorAdapterImplementation* implementation,
    std::string* error)
{
    const auto fail = [error](std::string message) {
        if (error != nullptr) {
            *error = std::move(message);
        }
        return false;
    };
    if (implementation == nullptr) {
        return fail("sensor implementation descriptor is null");
    }
    if (implementation->version != sa::kSensorAdapterImplementationVersion) {
        return fail("sensor implementation descriptor has version "
            + std::to_string(implementation->version) + ", expected "
            + std::to_string(sa::kSensorAdapterImplementationVersion));
    }
    if (implementation->name == nullptr || implementation->name[0] == '\0') {
        return fail("sensor implementation descriptor has no name");
    }
    if (!implementations_.emplace(implementation->name, implementation).second) {
        return fail(std::string("sensor implementation registered twice: ") + implementation->name);
    }
    return true;
}

const sa::SensorAdapterImplementation* SensorImplementationRegistry::find(
    const std::string& name) const
{
    const auto found = implementations_.find(name);
    return found == implementations_.end() ? nullptr : found->second;
}

std::set<std::string> SensorImplementationRegistry::names() const
{
    std::set<std::string> result;
    for (const auto& [name, implementation] : implementations_) {
        result.insert(name);
    }
    return result;
}

void SensorImplementationRegistry::clear()
{
    implementations_.clear();
}

SensorImplementationRegistry& sensor_implementation_registry()
{
    static SensorImplementationRegistry registry;
    return registry;
}

bool provides_sensor_type(const sa::SensorAdapterImplementation& implementation, const std::string& type)
{
    const auto types = sa::provided_sensor_types(implementation);
    return std::find(types.begin(), types.end(), type) != types.end();
}

} // namespace hakoniwa::robot_runtime::runtime::sensor
