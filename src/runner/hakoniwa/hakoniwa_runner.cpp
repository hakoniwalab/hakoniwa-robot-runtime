#include "runner/hakoniwa/hakoniwa_runner.hpp"

#include "runner/hakoniwa/hakoniwa_asset_driver.hpp"
#include "runner/realtime_pacer.hpp"

#include "hako_conductor.h"
#include "hakoniwa/pdu/endpoint.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

namespace hakoniwa::robot_runtime::runner {

class HakoniwaRunner::Impl final
    : public detail::IHakoniwaAssetDriverCallbacks {
public:
    Impl(
        HakoniwaRunnerResourcesFactory resources_factory_value,
        HakoniwaRunnerConfig config_value)
        : resources_factory(std::move(resources_factory_value))
        , config(std::move(config_value))
        , pacer(config.realtime_sync_cycle_msec)
    {
        if (!resources_factory) {
            throw std::invalid_argument(
                "Hakoniwa Runner resources factory must be provided");
        }
        if (config.asset_name.empty()) {
            throw std::invalid_argument(
                "Hakoniwa Runner asset name must not be empty");
        }
        if (config.pdu_definition_path.empty()) {
            throw std::invalid_argument(
                "Hakoniwa Runner PDU definition path must not be empty");
        }
    }

    void fail(std::string message) noexcept
    {
        last_error = std::move(message);
        running.store(false);
    }

    void request_stop() noexcept
    {
        running.store(false);
    }

    void cleanup() noexcept
    {
        if (asset_thread.joinable()) {
            asset_thread.join();
        }
        if (endpoint_started && resources.endpoint != nullptr) {
            (void)resources.endpoint->stop();
            endpoint_started = false;
        }
        if (resources.endpoint != nullptr) {
            resources.endpoint->close();
        }
        if (conductor_started) {
            hako_conductor_stop();
            conductor_started = false;
        }
        driver.reset();
        resources = {};
        pacer.reset();
        paused.store(false);
        reset_requested.store(false);
        started = false;
    }

    int on_initialize() noexcept override
    {
        for (const auto& input : resources.lockstep_inputs) {
            input->lockstep_prepare();
        }
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
        for (const auto& input : resources.async_rpc_inputs) {
            input->lockstep_prepare();
        }
#endif
        if (resources.endpoint == nullptr
            || resources.endpoint->post_start() != HAKO_PDU_ERR_OK) {
            fail("failed to complete endpoint post_start");
            return -1;
        }
        return 0;
    }

    bool sleep_before_first_step() const noexcept override
    {
        return !resources.lockstep_inputs.empty();
    }

    bool ready_to_step() noexcept override
    {
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
        // Propagate pause before consulting any other lockstep input.  A
        // different input may be incomplete and return early below, but that
        // must not leave the asynchronous RPC timeout clock running.
        const bool is_paused = paused.load();
        for (const auto& input : resources.async_rpc_inputs) {
            input->set_waiting_paused(is_paused);
        }
        // With asynchronous RPC, one request batch corresponds to exactly one
        // physics/time step.  A viewer pause therefore blocks the Driver before
        // on_step() and sleep_no_wait().  The legacy PDU-only path remains below.
        if (is_paused && !resources.async_rpc_inputs.empty()) {
            return false;
        }
#endif
        for (const auto& input : resources.lockstep_inputs) {
            if (!input->lockstep_ready()) {
                return false;
            }
        }
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
        for (const auto& input : resources.async_rpc_inputs) {
            if (!input->lockstep_ready()) {
                if (input->failed()) {
                    fail(input->failure_reason());
                }
                return false;
            }
        }
#endif
        return true;
    }

    int on_step() noexcept override
    {
        try {
            std::uint64_t simulation_time_usec = 0;
            {
                std::lock_guard<std::mutex> lock(model_mutex);
                if (reset_requested.exchange(false)) {
                    const auto current_time_usec =
                        resources.plant->read_state().sample_time_usec;
                    (void)resources.runtime->reset(current_time_usec);
                    pacer.reset();
                    std::cout << "[INFO] Robot simulation reset." << std::endl;
                }
                if (paused.load()) {
                    pacer.reset();
                    return 0;
                }
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
                for (const auto& input : resources.async_rpc_inputs) {
                    if (!input->apply_ready_requests()) {
                        throw std::runtime_error(input->failure_reason());
                    }
                }
#endif
                // Inputs are taken at the first substep (none are new later: the
                // controllers hold them); the state goes out after the last.
                const auto substeps = std::max<std::uint32_t>(1, config.physics_substeps);
                for (std::uint32_t substep = 0; substep < substeps; ++substep) {
                    simulation_time_usec = resources.runtime->step(substep + 1 == substeps)
                        .next_step.simulation_time_usec;
                }
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
                for (const auto& input : resources.async_rpc_inputs) {
                    if (!input->complete_step()) {
                        throw std::runtime_error(input->failure_reason());
                    }
                }
#endif
            }
            pacer.pace(
                static_cast<double>(simulation_time_usec) / 1'000'000.0);
            return 0;
        } catch (const std::exception& error) {
            fail(error.what());
            std::cerr << "[ERROR] Hakoniwa Runner step failed: "
                      << error.what() << std::endl;
            return -1;
        }
    }

    int on_reset() noexcept override
    {
        try {
            std::lock_guard<std::mutex> lock(model_mutex);
            (void)resources.runtime->reset();
            reset_requested.store(false);
            pacer.reset();
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
            for (const auto& input : resources.async_rpc_inputs) {
                input->lockstep_prepare();
            }
#endif
            return 0;
        } catch (const std::exception& error) {
            fail(error.what());
            std::cerr << "[ERROR] Hakoniwa Runner reset failed: "
                      << error.what() << std::endl;
            return -1;
        }
    }

    [[nodiscard]] bool should_stop() const noexcept override
    {
        return !running.load();
    }

#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
    [[nodiscard]] int stop_result() const noexcept override
    {
        return last_error.empty() ? 0 : -1;
    }
#endif

    HakoniwaRunnerResourcesFactory resources_factory;
    HakoniwaRunnerConfig config;
    RealtimePacer pacer;
    HakoniwaRunnerResources resources;
    std::unique_ptr<detail::HakoniwaAssetDriver> driver;
    std::thread asset_thread;
    std::atomic_bool running {true};
    std::atomic_bool paused {false};
    std::atomic_bool reset_requested {false};
    std::mutex model_mutex;
    int start_result {1};
    bool started {false};
    bool conductor_started {false};
    bool endpoint_started {false};
    std::string last_error;
};

HakoniwaRunner::HakoniwaRunner(
    HakoniwaRunnerResourcesFactory resources_factory,
    HakoniwaRunnerConfig config)
    : impl_(std::make_unique<Impl>(
        std::move(resources_factory), std::move(config)))
{
}

HakoniwaRunner::~HakoniwaRunner()
{
    request_stop();
    impl_->cleanup();
}

int HakoniwaRunner::start()
{
    if (impl_->started) {
        std::cerr << "[ERROR] Hakoniwa Runner is already started." << std::endl;
        return 1;
    }

    impl_->last_error.clear();
    impl_->running.store(true);
    impl_->paused.store(false);
    impl_->reset_requested.store(false);
    impl_->pacer.reset();
    impl_->start_result = 1;

    try {
        impl_->resources = impl_->resources_factory();
        if (impl_->resources.runtime == nullptr
            || impl_->resources.plant == nullptr
            || impl_->resources.endpoint == nullptr) {
            throw std::runtime_error(
                "Hakoniwa Runner resources factory returned incomplete resources");
        }

        const auto delta_time_usec = impl_->resources.plant->delta_time_usec()
            * std::max<std::uint32_t>(1, impl_->config.physics_substeps);
        impl_->driver = std::make_unique<detail::HakoniwaAssetDriver>(
            detail::HakoniwaAssetDriverConfig {
                impl_->config.asset_name,
                impl_->config.pdu_definition_path,
                delta_time_usec,
                HAKO_ASSET_MODEL_PLANT,
            },
            *impl_);
    } catch (const std::exception& error) {
        std::cerr << "[ERROR] Failed to prepare Hakoniwa Runner: "
                  << error.what() << std::endl;
        impl_->cleanup();
        return 1;
    }

    if (impl_->config.owns_conductor) {
        const auto delta_time_usec = impl_->resources.plant->delta_time_usec()
            * std::max<std::uint32_t>(1, impl_->config.physics_substeps);
        hako_conductor_start(
            static_cast<hako_time_t>(delta_time_usec),
            static_cast<hako_time_t>(impl_->config.conductor_max_delay_usec));
        impl_->conductor_started = true;
    }

    if (impl_->driver->register_asset() != 0) {
        std::cerr << "[ERROR] Failed to register Hakoniwa Asset." << std::endl;
        impl_->cleanup();
        return 1;
    }
    if (impl_->resources.endpoint->start() != HAKO_PDU_ERR_OK) {
        std::cerr << "[ERROR] Failed to start Runtime endpoint." << std::endl;
        impl_->cleanup();
        return 1;
    }

    impl_->endpoint_started = true;
    impl_->started = true;

    std::cout << "[INFO] Starting Hakoniwa Runner: asset="
              << impl_->config.asset_name << std::endl;
    try {
        impl_->asset_thread = std::thread([state = impl_.get()]() {
            state->start_result = state->driver->start();
            state->request_stop();
        });
    } catch (const std::exception& error) {
        std::cerr << "[ERROR] Failed to start Hakoniwa Runner thread: "
                  << error.what() << std::endl;
        impl_->request_stop();
        impl_->cleanup();
        return 1;
    }
    return 0;
}

int HakoniwaRunner::wait()
{
    if (!impl_->started) {
        return 1;
    }
    if (impl_->asset_thread.joinable()) {
        impl_->asset_thread.join();
    }
    const int start_result = impl_->start_result;
    if (!impl_->last_error.empty()) {
        std::cerr << "[ERROR] Hakoniwa Runner stopped: "
                  << impl_->last_error << std::endl;
    }
    const bool succeeded = start_result == 0 && impl_->last_error.empty();
    impl_->cleanup();
    return succeeded ? 0 : 1;
}

void HakoniwaRunner::request_stop() noexcept
{
    impl_->request_stop();
}

void HakoniwaRunner::toggle_pause() noexcept
{
    const bool paused = !impl_->paused.load();
    impl_->paused.store(paused);
    std::cout << "[INFO] " << (paused ? "paused" : "resumed") << std::endl;
}

void HakoniwaRunner::request_reset() noexcept
{
    impl_->reset_requested.store(true);
}

mjModel* HakoniwaRunner::model() noexcept
{
    return impl_->resources.plant == nullptr
        ? nullptr
        : impl_->resources.plant->model();
}

mjData* HakoniwaRunner::data() noexcept
{
    return impl_->resources.plant == nullptr
        ? nullptr
        : impl_->resources.plant->data();
}

std::mutex& HakoniwaRunner::model_mutex() noexcept
{
    return impl_->model_mutex;
}

std::atomic_bool& HakoniwaRunner::running() noexcept
{
    return impl_->running;
}

} // namespace hakoniwa::robot_runtime::runner
