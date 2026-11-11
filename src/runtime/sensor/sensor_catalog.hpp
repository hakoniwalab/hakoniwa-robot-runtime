#pragma once

#include <nlohmann/json.hpp>

#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runtime::sensor {

/**
 * Which sensor types each implementation provides, and which keys of the
 * common sensor definition file it supports for each type.
 *
 * A key path is a dot-separated path into the definition file, for example
 * "noise" or "encoder.bits". A path covers its whole subtree: "noise" allows
 * every key under "noise". The top-level "type" and "$schema" keys are
 * always allowed.
 */
struct SensorCatalog {
    // implementation -> sensor type -> supported key paths
    std::map<std::string, std::map<std::string, std::vector<std::string>>> implementations;
};

[[nodiscard]] bool parse_sensor_catalog(
    const nlohmann::json& value,
    SensorCatalog& output,
    std::string* error);

[[nodiscard]] bool load_sensor_catalog(
    const std::filesystem::path& path,
    SensorCatalog& output,
    std::string* error);

/**
 * Key paths in the definition that the supported paths do not cover, sorted
 * by key (nlohmann::json order). Arrays and scalars are leaves.
 */
[[nodiscard]] std::vector<std::string> find_unsupported_keys(
    const std::vector<std::string>& supported_paths,
    const nlohmann::json& definition);

/**
 * Checks a sensor definition against the catalog, in this order:
 *  1. the implementation is in the catalog,
 *  2. the implementation provides the sensor type (from the sensor instance;
 *     a definition that has a top-level "type" must agree with it - some
 *     formats, such as the drone_config IMU section, have none),
 *  3. every key of the definition is supported by that implementation,
 *  4. the implementation is linked into this application.
 * Fails with a message naming the definition source and every problem found
 * at the first failing step.
 */
[[nodiscard]] bool validate_sensor_definition(
    const SensorCatalog& catalog,
    const std::set<std::string>& linked_implementations,
    const std::string& implementation,
    const std::string& sensor_type,
    const nlohmann::json& definition,
    const std::string& source,
    std::string* error);

} // namespace hakoniwa::robot_runtime::runtime::sensor
