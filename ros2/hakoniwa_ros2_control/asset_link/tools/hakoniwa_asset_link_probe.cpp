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

#include <sys/resource.h>

#include <chrono>
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
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string key = argv[i];
    const std::string value = argv[i + 1];
    if (key == "--pdu-def") {
      config.pdu_definition_path = value;
    } else if (key == "--endpoint") {
      config.endpoint_config_path = value;
    } else if (key == "--robot") {
      config.robot_name = value;
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
  const auto begin = std::chrono::steady_clock::now();
  double last_wall = 0.0;
  double last_asset = link.asset_time_usec() * 1e-6;
  double last_cpu = cpu_seconds();
  auto next_print = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (link.running()) {
    // Every new joint states PDU to the log (polled; the probe is not real time).
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
  std::printf("ended: result %d, asset time %.6f s, %llu steps\n", link.result(), link.asset_time_usec() * 1e-6,
              static_cast<unsigned long long>(link.steps()));
  return link.result() == 0 ? 0 : 1;
}
