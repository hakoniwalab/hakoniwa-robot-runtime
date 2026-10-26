#pragma once

#include "hakoniwa/robot_runtime/adapters/physics/mujoco/mujoco_actuator_plant.hpp"
#include "hakoniwa/robot_runtime/runner/runner.hpp"
#include "hakoniwa/robot_runtime/runner/lockstep_input.hpp"
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
#include "hakoniwa/robot_runtime/runner/async_rpc_input.hpp"
#endif
#include "hakoniwa/pdu/endpoint.hpp"
#include "runtime/actuator_runtime.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runner {

struct HakoniwaRunnerResources {
    std::unique_ptr<runtime::ActuatorRuntime> runtime;
    std::shared_ptr<adapters::mujoco::MujocoActuatorPlant> plant;
    std::unique_ptr<::hakoniwa::pdu::Endpoint> endpoint;
    /** Inputs each step waits for (lockstep mode; empty otherwise). */
    std::vector<std::shared_ptr<ILockstepInput>> lockstep_inputs;
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
    /** Complete multi-robot request batches; opt-in build and empty by default. */
    std::vector<std::shared_ptr<AsyncRpcInput>> async_rpc_inputs;
#endif
};

using HakoniwaRunnerResourcesFactory =
    std::function<HakoniwaRunnerResources()>;

struct HakoniwaRunnerConfig {
    std::string asset_name;
    std::string pdu_definition_path;
    std::uint64_t realtime_sync_cycle_msec {0};
    /** Start/stop a local Conductor. False joins an externally owned one. */
    bool owns_conductor {true};
    /** The local Conductor's max_delay (used only when owns_conductor). */
    std::uint64_t conductor_max_delay_usec {100000};
    /**
     * Physics steps per Hakoniwa step: the asset (and its own Conductor) steps
     * physics_substeps x the Plant's timestep, takes its inputs once and
     * publishes its state once per Hakoniwa step (as CARLA's synchronous tick
     * with substepping). 1 steps Hakoniwa at the Plant's timestep.
     */
    std::uint32_t physics_substeps {1};
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
