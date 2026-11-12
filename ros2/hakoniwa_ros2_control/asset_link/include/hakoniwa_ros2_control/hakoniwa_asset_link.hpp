#pragma once

// The Hakoniwa side of HakoniwaSystem: a Hakoniwa asset that runs on a thread of
// its own, on the Hakoniwa time (one step of delta_usec at a time). It depends
// on the Hakoniwa core only, not on ROS 2, so that it can be checked alone.
//
//   register_asset(): once per process, while the simulation is stopped
//   start():          starts the asset thread; it waits for START, then steps
//   stop():           asks the asset thread to stop and joins it
//
// The design is in docs/ros2-control.md, chapter 10.

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "hako_asset.h"

namespace hakoniwa_ros2_control
{

class HakoniwaAssetLink
{
public:
  struct Config
  {
    std::string asset_name;
    std::string pdu_definition_path;  // the robot's pdudef (hako_asset_register checks it exists)
    std::uint64_t delta_usec{2000};   // the asset's step; the Robot Runtime's dT
    // Before each step, sleep until the wall clock has caught up with the
    // asset's time (as the Robot Runtime's real-time sync does). Without it the
    // step spins on a CPU and runs ahead to the conductor's limit.
    bool realtime_sync{true};
  };

  explicit HakoniwaAssetLink(Config config);
  ~HakoniwaAssetLink();
  HakoniwaAssetLink(const HakoniwaAssetLink &) = delete;
  HakoniwaAssetLink & operator=(const HakoniwaAssetLink &) = delete;

  bool register_asset();
  bool start();
  void stop();

  // The asset's own Hakoniwa time after its last step [usec], and the number of steps.
  std::uint64_t asset_time_usec() const { return asset_time_usec_.load(); }
  std::uint64_t steps() const { return steps_.load(); }
  // True from START until the asset thread ends (stop, simulation stop or reset, error).
  bool running() const { return running_.load(); }
  // The result of the asset thread: 0 for a normal end (stop, simulation stop or reset).
  int result() const { return result_.load(); }
  // True once the asset thread has ended (also when it never reached START).
  bool finished() const { return finished_.load(); }

private:
  static int on_initialize(hako_asset_context_t * context);
  static int on_manual_timing_control(hako_asset_context_t * context);
  static int on_reset(hako_asset_context_t * context);
  static int is_force_stop();
  int run_steps();

  Config config_;
  hako_asset_callbacks_t callbacks_{};  // the core keeps this pointer while registered
  bool registered_{false};
  std::thread thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> finished_{false};
  std::atomic<int> result_{0};
  std::atomic<std::uint64_t> asset_time_usec_{0};
  std::atomic<std::uint64_t> steps_{0};

  // The core's callbacks carry no user pointer: one asset per process.
  static std::atomic<HakoniwaAssetLink *> instance_;
};

}  // namespace hakoniwa_ros2_control
