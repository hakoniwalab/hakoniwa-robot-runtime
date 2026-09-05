#pragma once

#include "hakoniwa/robot_runtime/runner/runner.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace hakoniwa::robot_runtime::factory {

struct ManifestFactoryConfig {
    /** Empty means use manifest.name. */
    std::string asset_name;
    std::string endpoint_name;
    std::uint64_t realtime_sync_cycle_msec {0};
};

/**
 * Top-level composition root from a declarative manifest file to an
 * application-facing Runner.
 *
 * ManifestFactory owns common document loading/path resolution and delegates
 * Runtime semantics to RuntimeFactory and execution construction to
 * RunnerFactory.
 */
class ManifestFactory final {
public:
    [[nodiscard]] static std::unique_ptr<runner::IRunner> create(
        const std::string& manifest_path,
        ManifestFactoryConfig config);
};

} // namespace hakoniwa::robot_runtime::factory
