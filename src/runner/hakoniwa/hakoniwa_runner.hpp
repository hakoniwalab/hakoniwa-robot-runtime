#pragma once

#include "hakoniwa/robot_runtime/adapters/physics/mujoco/mujoco_actuator_plant.hpp"
#include "hakoniwa/robot_runtime/runner/runner.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "runtime/actuator_runtime.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace hakoniwa::robot_runtime::runner {

struct HakoniwaRunnerResources {
    std::unique_ptr<runtime::ActuatorRuntime> runtime;
    std::shared_ptr<adapters::mujoco::MujocoActuatorPlant> plant;
    std::unique_ptr<::hakoniwa::pdu::Endpoint> endpoint;
};

using HakoniwaRunnerResourcesFactory =
    std::function<HakoniwaRunnerResources()>;

struct HakoniwaRunnerConfig {
    std::string asset_name;
    std::string pdu_definition_path;
    std::uint64_t realtime_sync_cycle_msec {0};
};

/** Runs one pre-composed ActuatorRuntime as a Hakoniwa Asset. */
class HakoniwaRunner final : public IRunner {
public:
    HakoniwaRunner(
        HakoniwaRunnerResourcesFactory resources_factory,
        HakoniwaRunnerConfig config);
    ~HakoniwaRunner() override;

    HakoniwaRunner(const HakoniwaRunner&) = delete;
    HakoniwaRunner& operator=(const HakoniwaRunner&) = delete;

    [[nodiscard]] int start() override;
    [[nodiscard]] int wait() override;

    void request_stop() noexcept override;
    void toggle_pause() noexcept override;
    void request_reset() noexcept override;

    [[nodiscard]] mjModel* model() noexcept override;
    [[nodiscard]] mjData* data() noexcept override;
    [[nodiscard]] std::mutex& model_mutex() noexcept override;
    [[nodiscard]] std::atomic_bool& running() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::runner
