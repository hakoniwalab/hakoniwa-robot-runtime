#pragma once

#include "hako_asset.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace hakoniwa::robot_runtime::runner::detail {

struct HakoniwaAssetDriverConfig {
    std::string asset_name;
    std::string pdu_definition_path;
    std::uint64_t delta_time_usec {0};
    HakoAssetModelType model_type {HAKO_ASSET_MODEL_PLANT};
};

/** Callback seam used by HakoniwaAssetDriver to drive one Runner. */
class IHakoniwaAssetDriverCallbacks {
public:
    virtual ~IHakoniwaAssetDriverCallbacks() = default;
    virtual int on_initialize() noexcept = 0;
    virtual int on_step() noexcept = 0;
    virtual int on_reset() noexcept = 0;
    [[nodiscard]] virtual bool should_stop() const noexcept = 0;
};

/** Test seam around the process-global Hakoniwa C asset API. */
class IHakoniwaAssetApi {
public:
    virtual ~IHakoniwaAssetApi() = default;
    virtual int register_asset(
        const HakoniwaAssetDriverConfig& config,
        hako_asset_callbacks_t* callbacks) noexcept = 0;
    virtual int start_no_wait(int (*is_force_stop)(void)) noexcept = 0;
    virtual int sleep_no_wait(
        hako_time_t sleep_time_usec,
        int (*is_force_stop)(void)) noexcept = 0;
};

/** Production bridge that delegates directly to the Hakoniwa C API. */
class HakoniwaAssetApi final : public IHakoniwaAssetApi {
public:
    int register_asset(
        const HakoniwaAssetDriverConfig& config,
        hako_asset_callbacks_t* callbacks) noexcept override;
    int start_no_wait(int (*is_force_stop)(void)) noexcept override;
    int sleep_no_wait(
        hako_time_t sleep_time_usec,
        int (*is_force_stop)(void)) noexcept override;
};

/**
 * Low-level driver for Hakoniwa's process-global C Asset API.
 *
 * The Driver owns callback wiring and the manual-timing loop. Runtime policy
 * such as Endpoint readiness, pause/reset handling, pacing, and simulation
 * stepping belongs to the callback owner (HakoniwaRunner).
 */
class HakoniwaAssetDriver final {
public:
    HakoniwaAssetDriver(
        HakoniwaAssetDriverConfig config,
        IHakoniwaAssetDriverCallbacks& callbacks,
        std::shared_ptr<IHakoniwaAssetApi> api =
            std::make_shared<HakoniwaAssetApi>());
    ~HakoniwaAssetDriver();

    HakoniwaAssetDriver(const HakoniwaAssetDriver&) = delete;
    HakoniwaAssetDriver& operator=(const HakoniwaAssetDriver&) = delete;

    int register_asset() noexcept;
    int start() noexcept;

private:
    static int on_initialize(hako_asset_context_t*) noexcept;
    static int on_manual_timing_control(hako_asset_context_t*) noexcept;
    static int on_reset(hako_asset_context_t*) noexcept;
    static int is_force_stop() noexcept;

    int run_manual_timing_loop() noexcept;
    void release_active_instance() noexcept;

    HakoniwaAssetDriverConfig config_;
    IHakoniwaAssetDriverCallbacks& callbacks_;
    std::shared_ptr<IHakoniwaAssetApi> api_;
    hako_asset_callbacks_t callbacks_table_ {};
    bool registered_ {false};

    static std::atomic<HakoniwaAssetDriver*> active_instance_;
};

} // namespace hakoniwa::robot_runtime::runner::detail
