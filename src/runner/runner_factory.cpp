#include "runner/runner_factory.hpp"

#include <utility>

namespace hakoniwa::robot_runtime::runner {

std::unique_ptr<IRunner> create_hakoniwa_runner(
    HakoniwaRunnerResourcesFactory resources_factory,
    HakoniwaRunnerConfig config)
{
    return std::make_unique<HakoniwaRunner>(
        std::move(resources_factory), std::move(config));
}

} // namespace hakoniwa::robot_runtime::runner
