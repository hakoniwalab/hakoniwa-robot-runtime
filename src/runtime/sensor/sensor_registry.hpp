#pragma once

#include "hakoniwa/sensor_adapter/sensor_adapter_implementation.hpp"

#include <map>
#include <set>
#include <string>

namespace hakoniwa::robot_runtime::runtime::sensor {

/**
 * Sensor implementations linked into this application.
 *
 * The Runtime links no implementation itself. The application registers the
 * descriptor (hakoniwa-sensor-adapter SensorAdapterImplementation) of every
 * implementation library it links, before the manifest is resolved; the
 * registered names are the "linked" implementations of the catalog check.
 */
class SensorImplementationRegistry {
public:
    /**
     * Fails (returns false and sets error) on a null descriptor, another
     * descriptor version, an empty name, or a name registered twice.
     */
    [[nodiscard]] bool add(
        const ::hakoniwa::sensor_adapter::SensorAdapterImplementation* implementation,
        std::string* error = nullptr);
    [[nodiscard]] const ::hakoniwa::sensor_adapter::SensorAdapterImplementation* find(
        const std::string& name) const;
    [[nodiscard]] std::set<std::string> names() const;
    void clear();

private:
    std::map<std::string, const ::hakoniwa::sensor_adapter::SensorAdapterImplementation*>
        implementations_;
};

/** The process-wide registry that the application fills at start-up. */
SensorImplementationRegistry& sensor_implementation_registry();

/** True when the descriptor provides the sensor type (catalog type names). */
[[nodiscard]] bool provides_sensor_type(
    const ::hakoniwa::sensor_adapter::SensorAdapterImplementation& implementation,
    const std::string& type);

} // namespace hakoniwa::robot_runtime::runtime::sensor
