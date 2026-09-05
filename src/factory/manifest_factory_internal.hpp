#pragma once

#include "runtime/runtime_factory_manifest.hpp"

#include <string>

namespace hakoniwa::robot_runtime::factory::detail {

/** Common, Runtime-neutral data resolved from the top-level manifest file. */
struct ResolvedManifest {
    std::string name;
    std::string pdu_definition_path;
    std::string endpoint_path;
    runtime::RuntimeFactoryInput runtime_input;
};

/**
 * Load the manifest document, resolve common paths, and validate only the
 * top-level document contract. Module-specific semantics are delegated to the
 * corresponding module Factory.
 */
[[nodiscard]] bool resolve_manifest(
    const std::string& manifest_path,
    ResolvedManifest& output,
    std::string* error_message = nullptr);

} // namespace hakoniwa::robot_runtime::factory::detail
