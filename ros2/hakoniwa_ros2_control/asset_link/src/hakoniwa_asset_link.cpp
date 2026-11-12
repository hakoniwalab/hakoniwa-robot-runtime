#include "hakoniwa_ros2_control/hakoniwa_asset_link.hpp"

#include <cerrno>
#include <chrono>
#include <iostream>
#include <utility>

namespace hakoniwa_ros2_control
{

std::atomic<HakoniwaAssetLink *> HakoniwaAssetLink::instance_{nullptr};

HakoniwaAssetLink::HakoniwaAssetLink(Config config) : config_(std::move(config)) {}

HakoniwaAssetLink::~HakoniwaAssetLink()
{
  stop();
  HakoniwaAssetLink * self = this;
  instance_.compare_exchange_strong(self, nullptr);
}

bool HakoniwaAssetLink::register_asset()
{
  if (registered_) {
    return true;
  }
  HakoniwaAssetLink * none = nullptr;
  if (!instance_.compare_exchange_strong(none, this)) {
    std::cerr << "[HakoniwaAssetLink] one Hakoniwa asset per process: already registered" << std::endl;
    return false;
  }
  callbacks_ = {};
  callbacks_.on_initialize = &HakoniwaAssetLink::on_initialize;
  callbacks_.on_simulation_step = nullptr;
  callbacks_.on_manual_timing_control = &HakoniwaAssetLink::on_manual_timing_control;
  callbacks_.on_reset = &HakoniwaAssetLink::on_reset;
  // The core accepts a new asset only while the simulation is stopped.
  const int result = hako_asset_register(config_.asset_name.c_str(), config_.pdu_definition_path.c_str(),
                                         &callbacks_, static_cast<hako_time_t>(config_.delta_usec),
                                         HAKO_ASSET_MODEL_CONTROLLER);
  if (result != 0) {
    std::cerr << "[HakoniwaAssetLink] hako_asset_register(" << config_.asset_name << ") failed: " << result
              << " (is the simulation stopped, and the name unique?)" << std::endl;
    HakoniwaAssetLink * self = this;
    instance_.compare_exchange_strong(self, nullptr);
    return false;
  }
  registered_ = true;
  return true;
}

bool HakoniwaAssetLink::start()
{
  if (!registered_ || thread_.joinable()) {
    return false;
  }
  stop_requested_ = false;
  finished_ = false;
  // hako_asset_start_no_wait() waits for START and then runs
  // on_manual_timing_control() on the calling thread until the end: this thread.
  thread_ = std::thread([this]() {
    const int result = hako_asset_start_no_wait(&HakoniwaAssetLink::is_force_stop);
    running_ = false;
    // EINTR: the simulation stopped or was reset, or stop() was called.
    result_ = (result == EINTR) ? 0 : result;
    finished_ = true;
  });
  return true;
}

void HakoniwaAssetLink::stop()
{
  stop_requested_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
}

int HakoniwaAssetLink::on_initialize(hako_asset_context_t *)
{
  return 0;
}

int HakoniwaAssetLink::on_manual_timing_control(hako_asset_context_t *)
{
  auto * self = instance_.load();
  return self == nullptr ? -1 : self->run_steps();
}

int HakoniwaAssetLink::on_reset(hako_asset_context_t *)
{
  return 0;
}

int HakoniwaAssetLink::is_force_stop()
{
  auto * self = instance_.load();
  return (self == nullptr || self->stop_requested_.load()) ? 1 : 0;
}

int HakoniwaAssetLink::run_steps()
{
  running_ = true;
  using Clock = std::chrono::steady_clock;
  const auto wall_anchor = Clock::now();
  const auto time_anchor_usec = static_cast<std::uint64_t>(hako_asset_simulation_time());
  while (!stop_requested_) {
    if (config_.realtime_sync) {
      // The wall time at which this asset's time has passed since the anchor.
      const auto target = wall_anchor + std::chrono::microseconds(
        static_cast<std::uint64_t>(hako_asset_simulation_time()) - time_anchor_usec);
      std::this_thread::sleep_until(target);
    }
    // Advances this asset's time by one step; returns once the core lets it.
    const int result = hako_asset_usleep_no_wait(static_cast<hako_time_t>(config_.delta_usec),
                                                 &HakoniwaAssetLink::is_force_stop);
    if (result == EINTR) {
      return 0;  // the simulation stopped or was reset, or stop() was called
    }
    if (result != 0) {
      std::cerr << "[HakoniwaAssetLink] hako_asset_usleep_no_wait failed: " << result << std::endl;
      return result;
    }
    asset_time_usec_ = static_cast<std::uint64_t>(hako_asset_simulation_time());
    ++steps_;
  }
  return 0;
}

}  // namespace hakoniwa_ros2_control
