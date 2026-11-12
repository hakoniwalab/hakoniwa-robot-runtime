// Checks HakoniwaAssetLink without ROS 2: registers the asset, steps on the
// Hakoniwa time, and prints once a second
//   wall [s], asset time [s], steps, time ratio over the last second, CPU [%] of this process
// until the simulation stops (or --seconds have passed after START).
//
// hakoniwa_asset_link_probe --pdu-def so101-pdudef.json [--asset-name HakoniwaSystem]
//                           [--delta-usec 2000] [--realtime-sync 1] [--seconds 0]

#include <sys/resource.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string key = argv[i];
    const std::string value = argv[i + 1];
    if (key == "--pdu-def") {
      config.pdu_definition_path = value;
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
  if (config.pdu_definition_path.empty()) {
    std::fprintf(stderr, "--pdu-def is required\n");
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
  std::printf("started\n# wall_s asset_s steps ratio cpu_pct\n");
  const auto begin = std::chrono::steady_clock::now();
  double last_wall = 0.0;
  double last_asset = link.asset_time_usec() * 1e-6;
  double last_cpu = cpu_seconds();
  while (link.running()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    const double asset = link.asset_time_usec() * 1e-6;
    const double cpu = cpu_seconds();
    std::printf("%.3f %.6f %llu %.3f %.1f\n", wall, asset, static_cast<unsigned long long>(link.steps()),
                (asset - last_asset) / (wall - last_wall), 100.0 * (cpu - last_cpu) / (wall - last_wall));
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
