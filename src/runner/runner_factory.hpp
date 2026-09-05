#pragma once

#include "hakoniwa/robot_runtime/runner/runner.hpp"
#include "runner/hakoniwa/hakoniwa_runner.hpp"

#include <memory>

namespace hakoniwa::robot_runtime::runner {

/** Current Runner factory entry point. Standalone can be added beside Hakoniwa. */
[[nodiscard]] std::unique_ptr<IRunner> create_hakoniwa_runner(
    HakoniwaRunnerResourcesFactory resources_factory,
    HakoniwaRunnerConfig config);

} // namespace hakoniwa::robot_runtime::runner
