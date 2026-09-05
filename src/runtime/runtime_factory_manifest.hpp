#pragma once

#include "runtime/runtime_definition.hpp"

#include <string>

namespace hakoniwa::robot_runtime::runtime {

/**
 * Manifest data delegated by the top-level ManifestFactory to RuntimeFactory.
 * Common document loading/path resolution is already complete here; Runtime
 * component semantics remain owned by the Runtime module.
 *
 * The component document stays serialized across this module boundary so the
 * RuntimeFactory public-internal seam does not expose the JSON library type.
 */
struct RuntimeFactoryInput {
    std::string manifest_base_path;
    std::string model_path;
    std::string pdu_definition_path;
    std::string runtime_config_path;
    std::string components_json;
};

/** Resolve and validate the Runtime-specific portion of one manifest. */
[[nodiscard]] bool resolve_runtime_definition(
    const RuntimeFactoryInput& input,
    RuntimeDefinition& output,
    std::string* error_message = nullptr);

} // namespace hakoniwa::robot_runtime::runtime
