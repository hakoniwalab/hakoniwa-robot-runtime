#include "runner/hakoniwa/hakoniwa_asset_driver.hpp"

#include <cerrno>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runner::detail {

std::atomic<HakoniwaAssetDriver*> HakoniwaAssetDriver::active_instance_ {nullptr};

int HakoniwaAssetApi::register_asset(
    const HakoniwaAssetDriverConfig& config,
    hako_asset_callbacks_t* callbacks) noexcept
{
    return hako_asset_register(
        config.asset_name.c_str(),
        config.pdu_definition_path.c_str(),
        callbacks,
        static_cast<hako_time_t>(config.delta_time_usec),
        config.model_type);
}

int HakoniwaAssetApi::start_no_wait(int (*is_force_stop)(void)) noexcept
{
    return hako_asset_start_no_wait(is_force_stop);
}

int HakoniwaAssetApi::sleep_no_wait(
    const hako_time_t sleep_time_usec,
    int (*is_force_stop)(void)) noexcept
{
    return hako_asset_usleep_no_wait(sleep_time_usec, is_force_stop);
}

HakoniwaAssetDriver::HakoniwaAssetDriver(
    HakoniwaAssetDriverConfig config,
    IHakoniwaAssetDriverCallbacks& callbacks,
    std::shared_ptr<IHakoniwaAssetApi> api)
    : config_(std::move(config))
    , callbacks_(callbacks)
    , api_(std::move(api))
{
    if (config_.asset_name.empty()) {
        throw std::invalid_argument("Hakoniwa asset name must not be empty");
    }
    if (config_.pdu_definition_path.empty()) {
        throw std::invalid_argument("Hakoniwa PDU definition path must not be empty");
    }
    if (config_.delta_time_usec == 0
        || config_.delta_time_usec
            > static_cast<std::uint64_t>(std::numeric_limits<hako_time_t>::max())) {
        throw std::invalid_argument("Hakoniwa asset delta time is out of range");
    }
    if (api_ == nullptr) {
        throw std::invalid_argument("Hakoniwa Asset API must not be null");
    }
}

HakoniwaAssetDriver::~HakoniwaAssetDriver()
{
    release_active_instance();
}

int HakoniwaAssetDriver::register_asset() noexcept
{
    if (registered_) {
        return -1;
    }
    HakoniwaAssetDriver* expected = nullptr;
    if (!active_instance_.compare_exchange_strong(expected, this)) {
        return -1;
    }

    callbacks_table_ = {};
    callbacks_table_.on_initialize = &HakoniwaAssetDriver::on_initialize;
    callbacks_table_.on_simulation_step = nullptr;
    callbacks_table_.on_manual_timing_control =
        &HakoniwaAssetDriver::on_manual_timing_control;
    callbacks_table_.on_reset = &HakoniwaAssetDriver::on_reset;

    // Hakoniwa's C API retains this pointer for the full registered lifetime;
    // callbacks_table_ must therefore be owned by the Driver.
    const int result = api_->register_asset(config_, &callbacks_table_);
    if (result != 0) {
        release_active_instance();
        return result;
    }
    registered_ = true;
    return 0;
}

int HakoniwaAssetDriver::start() noexcept
{
    if (!registered_ || active_instance_.load() != this) {
        return -1;
    }
    return api_->start_no_wait(&HakoniwaAssetDriver::is_force_stop);
}

int HakoniwaAssetDriver::on_initialize(hako_asset_context_t*) noexcept
{
    auto* driver = active_instance_.load();
    return driver == nullptr ? -1 : driver->callbacks_.on_initialize();
}

int HakoniwaAssetDriver::on_manual_timing_control(hako_asset_context_t*) noexcept
{
    auto* driver = active_instance_.load();
    return driver == nullptr ? -1 : driver->run_manual_timing_loop();
}

int HakoniwaAssetDriver::on_reset(hako_asset_context_t*) noexcept
{
    auto* driver = active_instance_.load();
    return driver == nullptr ? -1 : driver->callbacks_.on_reset();
}

int HakoniwaAssetDriver::is_force_stop() noexcept
{
    auto* driver = active_instance_.load();
    return driver == nullptr || driver->callbacks_.should_stop() ? 1 : 0;
}

int HakoniwaAssetDriver::run_manual_timing_loop() noexcept
{
    while (!callbacks_.should_stop()) {
        const int step_result = callbacks_.on_step();
        if (step_result != 0) {
            return step_result;
        }
        const int sleep_result = api_->sleep_no_wait(
            static_cast<hako_time_t>(config_.delta_time_usec),
            &HakoniwaAssetDriver::is_force_stop);
        if (sleep_result == EINTR) {
            // Hakoniwa reports a normal external stop/reset sequence as EINTR.
            return 0;
        }
        if (sleep_result != 0 && !callbacks_.should_stop()) {
            std::cerr << "[ERROR] Hakoniwa asset timing step failed: result="
                      << sleep_result << std::endl;
            return sleep_result;
        }
    }
    return 0;
}

void HakoniwaAssetDriver::release_active_instance() noexcept
{
    HakoniwaAssetDriver* expected = this;
    (void)active_instance_.compare_exchange_strong(expected, nullptr);
    registered_ = false;
}

} // namespace hakoniwa::robot_runtime::runner::detail
