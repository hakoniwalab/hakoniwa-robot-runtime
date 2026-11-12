// Checks HakoniwaAssetLink without ROS 2: registers the asset, steps on the
// Hakoniwa time, and prints once a second
//   wall [s], asset time [s], steps, time ratio over the last second, CPU [%] of this process, world time [s],
//   joint states read so far, read errors, the latest stamp [s] and its age in asset time [ms]
// until the simulation stops (or --seconds have passed after START). With
// --state-log, every new joint states PDU goes to a CSV file:
//   stamp_s, asset_s, <joint>... (positions)
//
// hakoniwa_asset_link_probe --pdu-def so101-pdudef.json --endpoint so101_endpoint.json --robot SO101
//                           [--asset-name HakoniwaSystem] [--delta-usec 2000] [--realtime-sync 1]
//                           [--seconds 0] [--state-log states.csv]
//                           [--command-joint shoulder_pan --command-hz 100 --amplitude 0.5 --period 5
//                            --command-log commands.csv]
// With --sensor NAME:RATE (repeatable), the probe takes each sensor's raw PDU
// and prints, at the end, how many it took and their sizes.
// With --command-joint, the probe plays controller_manager: it puts
// amplitude * sin(2 pi t / period) for that joint at command-hz (wall clock),
// logs each command (asset_s, value) and prints the command counts at the end.

#include <sys/resource.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>

#include "hakoniwa_ros2_control/hakoniwa_asset_link.hpp"

namespace
{
double cpu_seconds()
{
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec * 1e-6 + usage.ru_stime.tv_sec +
         usage.ru_stime.tv_usec * 1e-6;
}
}  // namespace

int main(int argc, char ** argv)
{
  hakoniwa_ros2_control::HakoniwaAssetLink::Config config;
  config.asset_name = "HakoniwaSystem";
  double seconds = 0.0;  // 0: until the simulation stops
  std::string state_log;
  std::string command_log;
  double command_hz = 100.0;
  double amplitude = 0.5;
  double period = 5.0;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string key = argv[i];
    const std::string value = argv[i + 1];
    if (key == "--pdu-def") {
      config.pdu_definition_path = value;
    } else if (key == "--endpoint") {
      config.endpoint_config_path = value;
    } else if (key == "--robot") {
      config.robot_name = value;
    } else if (key == "--sensor") {
      const auto colon = value.find(':');
      config.sensors.push_back({value.substr(0, colon), std::atof(value.substr(colon + 1).c_str())});
    } else if (key == "--command-joint") {
      config.command_joints.push_back(value);
    } else if (key == "--command-hz") {
      command_hz = std::atof(value.c_str());
    } else if (key == "--amplitude") {
      amplitude = std::atof(value.c_str());
    } else if (key == "--period") {
      period = std::atof(value.c_str());
    } else if (key == "--command-log") {
      command_log = value;
    } else if (key == "--state-log") {
      state_log = value;
    } else if (key == "--asset-name") {
      config.asset_name = value;
    } else if (key == "--delta-usec") {
      config.delta_usec = std::strtoull(value.c_str(), nullptr, 10);
    } else if (key == "--realtime-sync") {
      config.realtime_sync = value != "0";
    } else if (key == "--seconds") {
      seconds = std::atof(value.c_str());
    } else {
      std::fprintf(stderr, "unknown option %s\n", key.c_str());
      return 2;
    }
  }
  if (config.pdu_definition_path.empty() || config.endpoint_config_path.empty() || config.robot_name.empty()) {
    std::fprintf(stderr, "--pdu-def, --endpoint and --robot are required\n");
    return 2;
  }

  hakoniwa_ros2_control::HakoniwaAssetLink link(config);
  if (!link.register_asset()) {
    return 1;
  }
  std::printf("registered %s (delta %llu usec); waiting for START\n", config.asset_name.c_str(),
              static_cast<unsigned long long>(config.delta_usec));
  std::fflush(stdout);
  link.start();

  while (!link.running() && !link.finished()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::printf("started\n# wall_s asset_s steps ratio cpu_pct world_s states read_errors stamp_s age_ms\n");
  std::ofstream log;
  if (!state_log.empty()) {
    log.open(state_log);
  }
  hakoniwa_ros2_control::JointStateSample sample;
  std::uint64_t logged = 0;
  std::vector<std::uint64_t> sensor_taken(config.sensors.size(), 0);
  std::vector<std::size_t> sensor_bytes(config.sensors.size(), 0);
  std::vector<double> sensor_first(config.sensors.size(), -1.0);
  std::vector<double> sensor_last(config.sensors.size(), 0.0);
  hakoniwa_ros2_control::SensorSample sensor_sample;
  std::ofstream commands;
  if (!command_log.empty()) {
    commands.open(command_log);
    commands << "asset_s,value\n";
  }
  const auto command_start = std::chrono::steady_clock::now();
  auto next_command = command_start;
  const auto command_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / command_hz));
  const auto begin = std::chrono::steady_clock::now();
  double last_wall = 0.0;
  double last_asset = link.asset_time_usec() * 1e-6;
  double last_cpu = cpu_seconds();
  auto next_print = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (link.running()) {
    // Every new joint states PDU to the log (polled; the probe is not real time).
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (!config.command_joints.empty() && std::chrono::steady_clock::now() >= next_command) {
      const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - command_start).count();
      const double value = amplitude * std::sin(2.0 * M_PI * t / period);
      link.put_command(std::vector<double>(config.command_joints.size(), value));
      if (commands.is_open()) {
        commands << std::fixed;
        commands.precision(6);
        commands << link.asset_time_usec() * 1e-6 << ",";
        commands.precision(9);
        commands << value << "\n";
      }
      next_command += command_period;
    }
    for (std::size_t i = 0; i < config.sensors.size(); ++i) {
      if (link.take_sensor(i, sensor_sample)) {
        ++sensor_taken[i];
        sensor_bytes[i] = sensor_sample.data.size();
        if (sensor_first[i] < 0.0) {
          sensor_first[i] = sensor_sample.asset_time_usec * 1e-6;
        }
        sensor_last[i] = sensor_sample.asset_time_usec * 1e-6;
      }
    }
    if (link.latest_state(sample) && sample.sequence != logged) {
      if (log.is_open()) {
        if (logged == 0) {
          log << "stamp_s,asset_s";
          for (const auto & name : sample.name) {
            log << "," << name;
          }
          log << "\n";
        }
        log << std::fixed;
        log.precision(6);
        log << sample.stamp_sec << "," << sample.asset_time_usec * 1e-6;
        log.precision(9);
        for (const double value : sample.position) {
          log << "," << value;
        }
        log << "\n";
      }
      logged = sample.sequence;
    }
    if (std::chrono::steady_clock::now() < next_print) {
      continue;
    }
    next_print += std::chrono::seconds(1);
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    const double asset = link.asset_time_usec() * 1e-6;
    const double cpu = cpu_seconds();
    const bool have = link.latest_state(sample);
    std::printf("%.3f %.6f %llu %.3f %.1f %.6f %llu %llu %.6f %.1f\n", wall, asset,
                static_cast<unsigned long long>(link.steps()), (asset - last_asset) / (wall - last_wall),
                100.0 * (cpu - last_cpu) / (wall - last_wall), link.world_time_usec() * 1e-6,
                static_cast<unsigned long long>(have ? sample.sequence : 0),
                static_cast<unsigned long long>(link.state_read_errors()), have ? sample.stamp_sec : 0.0,
                have ? (asset - sample.stamp_sec) * 1000.0 : 0.0);
    std::fflush(stdout);
    last_wall = wall;
    last_asset = asset;
    last_cpu = cpu;
    if (seconds > 0.0 && wall >= seconds) {
      break;
    }
  }
  link.stop();
  for (std::size_t i = 0; i < config.sensors.size(); ++i) {
    const double span = sensor_last[i] - sensor_first[i];
    std::printf("sensor %s (%s): taken %llu, %zu bytes, %.2f Hz in asset time\n", config.sensors[i].name.c_str(),
                link.sensor_type(i).c_str(), static_cast<unsigned long long>(sensor_taken[i]), sensor_bytes[i],
                (sensor_taken[i] > 1 && span > 0.0) ? (sensor_taken[i] - 1) / span : 0.0);
  }
  const auto counts = link.command_counts();
  std::printf("commands: put %llu, written %llu, replaced %llu; joint PDU writes %llu, refused %llu\n",
              static_cast<unsigned long long>(counts.put), static_cast<unsigned long long>(counts.written),
              static_cast<unsigned long long>(counts.replaced), static_cast<unsigned long long>(counts.pdu_writes),
              static_cast<unsigned long long>(counts.pdu_refused));
  std::printf("ended: result %d, asset time %.6f s, %llu steps\n", link.result(), link.asset_time_usec() * 1e-6,
              static_cast<unsigned long long>(link.steps()));
  return link.result() == 0 ? 0 : 1;
}
