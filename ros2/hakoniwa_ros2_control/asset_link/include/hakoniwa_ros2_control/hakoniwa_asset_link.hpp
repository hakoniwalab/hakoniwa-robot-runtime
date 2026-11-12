#pragma once

// The Hakoniwa side of HakoniwaSystem: a Hakoniwa asset that runs on a thread of
// its own, on the Hakoniwa time (one step of delta_usec at a time). It depends
// on the Hakoniwa core and PDU Endpoint only, not on ROS 2, so that it can be
// checked alone.
//
//   register_asset(): once per process, while the simulation is stopped
//   start():          starts the asset thread; it waits for START, then steps
//   latest_state():   the joint states the asset thread read last (any thread)
//   put_command():    the latest command for all command joints (any thread); the
//                     asset thread writes it once, retrying the joints refused
//   take_sensor():    the latest raw PDU of a sensor, read at its rate (any thread)
//   stop():           asks the asset thread to stop and joins it
//
// The design is in docs/ros2-control.md, chapter 10.

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hako_asset.h"

namespace hakoniwa_ros2_control
{

// One sensor PDU to read, at a rate in the asset's (Hakoniwa) time.
struct SensorPdu
{
  std::string name;   // the PDU name, e.g. "lidar_scan"
  double rate_hz{0};  // how often to read it
};

// One sensor PDU as the asset thread read it: the raw PDU (decoded by the reader).
struct SensorSample
{
  std::vector<std::byte> data;
  std::uint64_t asset_time_usec{0};  // this asset's time when it read the PDU
  std::uint64_t sequence{0};         // counts reads; 0: none yet
};

// One joint states PDU as the asset thread read it.
struct JointStateSample
{
  std::vector<std::string> name;
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  double stamp_sec{0.0};             // the PDU's header stamp: the Hakoniwa time it was measured
  std::uint64_t asset_time_usec{0};  // this asset's time when it read the PDU
  std::chrono::steady_clock::time_point received{};  // the wall time when it read the PDU
  std::uint64_t sequence{0};         // counts PDUs with a new stamp; 0: none yet
};

class HakoniwaAssetLink
{
public:
  struct Config
  {
    std::string asset_name;
    std::string pdu_definition_path;   // the robot's pdudef (hako_asset_register checks it exists)
    std::string endpoint_config_path;  // the PDU Endpoint (shared memory) of the same robot
    std::string robot_name;            // the PDU robot, e.g. "SO101"
    std::string joint_states_pdu{"joint_states"};
    // The joints commanded, each through its PDU "actuator/<joint>/command" (std_msgs/Float64).
    std::vector<std::string> command_joints;
    // Sensor PDUs to read (raw), each at its own rate.
    std::vector<SensorPdu> sensors;
    std::uint64_t delta_usec{2000};    // the asset's step; the Robot Runtime's dT
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

  // Copies the latest joint states into out; false while none has been read.
  bool latest_state(JointStateSample & out) const;

  // Sets the command for all command_joints (same order). The asset thread
  // writes it on its next step; a newer command replaces one not yet written.
  void put_command(const std::vector<double> & values);

  struct CommandCounts
  {
    std::uint64_t put{0};         // commands given with put_command()
    std::uint64_t written{0};     // commands written completely (every joint)
    std::uint64_t replaced{0};    // commands replaced by a newer one before written completely (or taken)
    std::uint64_t pdu_writes{0};  // joint PDU writes accepted
    std::uint64_t pdu_refused{0}; // joint PDU writes refused (the receiver had not taken the last one)
  };
  CommandCounts command_counts() const;

  // The PDU type of sensor i (Config::sensors order), from the PDU definition;
  // known after register_asset(), e.g. "sensor_msgs/PointCloud2".
  std::string sensor_type(std::size_t i) const;
  // Moves the latest read of sensor i into out when it is newer than the last
  // one taken; false otherwise.
  bool take_sensor(std::size_t i, SensorSample & out);

  // The asset's own Hakoniwa time after its last step [usec], and the number of steps.
  std::uint64_t asset_time_usec() const { return asset_time_usec_.load(); }
  // The world time seen after the last step [usec].
  std::uint64_t world_time_usec() const { return world_time_usec_.load(); }
  std::uint64_t steps() const { return steps_.load(); }
  // Steps whose joint states read failed (none written yet, or a decode error).
  std::uint64_t state_read_errors() const { return state_read_errors_.load(); }
  // True from START until the asset thread ends (stop, simulation stop or reset, error).
  bool running() const { return running_.load(); }
  // True once the asset thread has ended (also when it never reached START).
  bool finished() const { return finished_.load(); }
  // The result of the asset thread: 0 for a normal end (stop, simulation stop or reset).
  int result() const { return result_.load(); }

private:
  struct Pdu;  // the PDU Endpoint and its typed channels (kept out of this header)

  static int on_initialize(hako_asset_context_t * context);
  static int on_manual_timing_control(hako_asset_context_t * context);
  static int on_reset(hako_asset_context_t * context);
  static int is_force_stop();
  int run_steps();
  void read_joint_states();
  void write_command();
  void read_sensors();

  Config config_;
  std::unique_ptr<Pdu> pdu_;
  hako_asset_callbacks_t callbacks_{};  // the core keeps this pointer while registered
  bool registered_{false};
  std::thread thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> finished_{false};
  std::atomic<int> result_{0};
  std::atomic<std::uint64_t> asset_time_usec_{0};
  std::atomic<std::uint64_t> world_time_usec_{0};
  std::atomic<std::uint64_t> steps_{0};
  std::atomic<std::uint64_t> state_read_errors_{0};

  mutable std::mutex state_mutex_;  // guards state_: written by the asset thread, read by any thread
  JointStateSample state_;

  // The command: put by any thread, written by the asset thread.
  mutable std::mutex command_mutex_;
  std::vector<double> command_;
  std::uint64_t command_sequence_{0};  // counts put_command() calls
  CommandCounts counts_;
  // The asset thread's copy of the command being written, and which joints still wait.
  std::vector<double> writing_;
  std::vector<bool> pending_;
  std::uint64_t writing_sequence_{0};

  // Sensors: types and sizes (set at registration), read times (asset thread),
  // the latest reads (guarded by sensor_mutex_).
  std::vector<std::string> sensor_types_;
  std::vector<std::size_t> sensor_sizes_;
  std::vector<std::uint64_t> sensor_next_usec_;
  std::vector<std::uint64_t> sensor_taken_;
  std::mutex sensor_mutex_;
  std::vector<SensorSample> sensor_latest_;
  std::vector<std::byte> sensor_buffer_;

  // The core's callbacks carry no user pointer: one asset per process.
  static std::atomic<HakoniwaAssetLink *> instance_;
};

}  // namespace hakoniwa_ros2_control
