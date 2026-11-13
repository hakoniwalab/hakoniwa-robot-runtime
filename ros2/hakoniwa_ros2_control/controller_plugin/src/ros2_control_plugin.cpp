// The ros2_control controller plugin of the Robot Runtime (design:
// docs/design-controller-plugin.md 5.3-5.6). The Robot Runtime loads this shared
// library and calls update() on the control-period steps of its simulation thread;
// update() runs ControllerManager::read/update/write in simulation time. The
// hardware interface (HakoniwaPluginSystem) and the PluginBridge are in this same
// library. ROS 2 communication (MoveIt's trajectory actions, the controllers'
// topics, /clock, sensors) runs on threads of this plugin; nothing here calls the
// Robot Runtime. No exception crosses the C boundary.
//
// Time: once its node clock has started, ros2_control gives the controllers its
// node clock's now(), not the time passed to update(). That clock normally follows
// /clock through a subscription, which falls behind when the simulation runs
// faster than real time (the controllers then see a frozen time and jump). So the
// node clocks of controller_manager and of every controller do not subscribe to
// /clock here; update() sets them to the step's simulation time itself, before
// read/update/write. /clock is still published for the other programs.
//
// The plugin's configuration file (YAML, paths relative to it):
//   robot_description: so101_plugin.urdf     # URDF with <hardware> HakoniwaPluginSystem
//   params_files: [ros2_controllers.yaml]      # ROS parameters (controller_manager, controllers)
//   activate: [joint_state_broadcaster, arm_controller, gripper_controller]
//   publish_clock: true                        # /clock from the simulation time
//   sensor_topic_prefix: /pdu/                 # sensors as in the separate-process form
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "hakoniwa/robot_runtime/plugin/controller_plugin.h"

#include "plugin_bridge.hpp"
#include "sensor_relay.hpp"

namespace hakoniwa_ros2_control
{
namespace
{

namespace fs = std::filesystem;

std::atomic<bool> g_instance_exists{false};

struct PluginConfig
{
  std::string robot_description;            // the URDF text
  std::vector<std::string> params_files;
  std::vector<std::string> activate;
  bool publish_clock{true};
  std::string sensor_topic_prefix{"/pdu/"};
};

std::string read_text(const fs::path & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot read " + path.string());
  }
  std::stringstream text;
  text << stream.rdbuf();
  return text.str();
}

PluginConfig load_config(const std::string & config_path)
{
  const fs::path file(config_path);
  const fs::path dir = file.parent_path();
  const YAML::Node root = YAML::LoadFile(config_path);
  PluginConfig config;
  if (!root["robot_description"]) {
    throw std::runtime_error(config_path + ": robot_description is missing");
  }
  config.robot_description = read_text(dir / root["robot_description"].as<std::string>());
  for (const auto & item : root["params_files"]) {
    config.params_files.push_back((dir / item.as<std::string>()).string());
  }
  for (const auto & item : root["activate"]) {
    config.activate.push_back(item.as<std::string>());
  }
  if (root["publish_clock"]) {
    config.publish_clock = root["publish_clock"].as<bool>();
  }
  if (root["sensor_topic_prefix"]) {
    config.sensor_topic_prefix = root["sensor_topic_prefix"].as<std::string>();
  }
  if (config.activate.empty()) {
    throw std::runtime_error(config_path + ": activate lists no controller");
  }
  return config;
}

void copy_error(const std::string & text, char * error, size_t error_size)
{
  if (error != nullptr && error_size > 0) {
    std::snprintf(error, error_size, "%s", text.c_str());
  }
}

}  // namespace

// Switches controllers on a thread of its own: ros2_control applies a switch inside
// update(), which the Robot Runtime calls, so a switch requested on the simulation
// thread would wait for itself.
class SwitchWorker
{
public:
  explicit SwitchWorker(controller_manager::ControllerManager & cm)
  : cm_(cm), thread_([this]() { run(); })
  {
  }

  ~SwitchWorker()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    wake_.notify_all();
    thread_.join();
  }

  // done (may be null) is set to true once the switch has been applied.
  void request(std::vector<std::string> activate, std::vector<std::string> deactivate,
               std::shared_ptr<std::atomic<bool>> done)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.push_back({std::move(activate), std::move(deactivate), std::move(done)});
    }
    wake_.notify_one();
  }

private:
  struct Request
  {
    std::vector<std::string> activate;
    std::vector<std::string> deactivate;
    std::shared_ptr<std::atomic<bool>> done;
  };

  void run()
  {
    while (true) {
      Request request;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [this]() { return stop_ || !queue_.empty(); });
        if (stop_) {
          return;
        }
        request = std::move(queue_.front());
        queue_.pop_front();
      }
      const auto result = cm_.switch_controller(
        request.activate, request.deactivate,
        controller_manager_msgs::srv::SwitchController::Request::STRICT, true,
        rclcpp::Duration::from_seconds(10.0));
      if (result != controller_interface::return_type::OK) {
        RCLCPP_ERROR(rclcpp::get_logger("hakoniwa_controller_plugin"), "switching controllers failed");
        continue;
      }
      if (request.done) {
        request.done->store(true);
      }
    }
  }

  controller_manager::ControllerManager & cm_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<Request> queue_;
  bool stop_{false};
  std::thread thread_;
};

struct Ros2ControlPlugin
{
  PluginConfig config;
  std::vector<std::string> motion_controllers;   // activate minus the broadcasters
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::shared_ptr<controller_manager::ControllerManager> cm;
  rclcpp::Node::SharedPtr node;                  // /clock and sensors
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
  std::unique_ptr<SensorRelay> sensors;
  std::unique_ptr<SwitchWorker> switcher;
  std::thread spin;
  std::uint64_t period_ns{0};
  rclcpp::Duration period{0, 0};
  bool activation_requested{false};
  // Hold the current positions until the motion controllers are (re)activated:
  // at start, after a reset, and while/after another control took over.
  std::shared_ptr<std::atomic<bool>> active = std::make_shared<std::atomic<bool>>(false);
  bool was_selected{false};
  bool preempted{false};
  bool updated{false};   // an update has run since the controllers were requested
  // Diagnostics: with HAKO_CONTROLLER_PLUGIN_TRACE=<file>, one line per update:
  // step_index sim_time_ns selected reset active wall_ns, then a "J" line with the
  // joints and the commands of the step.
  std::FILE * trace{nullptr};
  // The clocks set to the simulation time on every update (see the top).
  std::vector<rclcpp::Clock::SharedPtr> sim_clocks;

  void set_sim_time(std::uint64_t sim_time_ns)
  {
    for (const auto & clock : sim_clocks) {
      std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
      rcl_clock_t * handle = clock->get_clock_handle();
      if (rcl_enable_ros_time_override(handle) != RCL_RET_OK ||
          rcl_set_ros_time_override(handle, static_cast<rcl_time_point_value_t>(sim_time_ns)) != RCL_RET_OK) {
        rcl_reset_error();
        throw std::runtime_error("cannot set a node clock to the simulation time");
      }
    }
  }

  void request_activation()
  {
    active->store(false);
    switcher->request(config.activate, {}, active);
    activation_requested = true;
  }

  // Drops whatever the motion controllers were doing (a trajectory in progress);
  // they hold the current position when activated again.
  void request_restart(bool reactivate)
  {
    active->store(false);
    switcher->request({}, motion_controllers, nullptr);
    if (reactivate) {
      switcher->request(motion_controllers, {}, active);
    }
  }
};

namespace
{

void * create(const HakoControllerCreateV1 * args, char * error, size_t error_size)
{
  if (args == nullptr || args->struct_size < sizeof(HakoControllerCreateV1)) {
    copy_error("create arguments are missing or too small", error, error_size);
    return nullptr;
  }
  bool expected = false;
  if (!g_instance_exists.compare_exchange_strong(expected, true)) {
    copy_error("one ros2_control plugin per process (rclcpp is initialized once)", error, error_size);
    return nullptr;
  }
  auto plugin = std::make_unique<Ros2ControlPlugin>();
  try {
    plugin->config = load_config(args->config_path != nullptr ? args->config_path : "");
    for (const auto & name : plugin->config.activate) {
      if (name.find("broadcaster") == std::string::npos) {
        plugin->motion_controllers.push_back(name);
      }
    }

    std::vector<std::string> joints;
    for (uint32_t i = 0; i < args->joint_count; ++i) {
      joints.emplace_back(args->joint_names[i]);
    }
    PluginBridge::instance().configure(joints);

    // ROS arguments: the parameter files apply to every node of the process
    // (controller_manager and each controller), all on simulation time.
    std::vector<std::string> arguments = {"hakoniwa_controller_plugin", "--ros-args"};
    for (const auto & file : plugin->config.params_files) {
      arguments.push_back("--params-file");
      arguments.push_back(file);
    }
    arguments.push_back("-p");
    arguments.push_back("use_sim_time:=true");
    std::vector<const char *> argv;
    for (const auto & argument : arguments) {
      argv.push_back(argument.c_str());
    }
    // Signals belong to the Robot Runtime: rclcpp must not shut itself down on
    // SIGINT/SIGTERM before the runtime has stopped the plugin.
    rclcpp::init(static_cast<int>(argv.size()), argv.data(), rclcpp::InitOptions(),
                 rclcpp::SignalHandlerOptions::None);

    plugin->executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    // The parameter files also go to the controller_manager node's own arguments:
    // ros2_control passes them on to every controller node it creates (global
    // arguments alone do not reach the controllers).
    rclcpp::NodeOptions cm_options = controller_manager::get_cm_node_options();
    std::vector<std::string> cm_arguments(arguments.begin() + 1, arguments.end());
    // controller_manager and the controllers (ros2_control passes these arguments on
    // to them) take their time from update(), not from /clock (see the top).
    cm_arguments.push_back("-r");
    cm_arguments.push_back("/clock:=/hakoniwa_controller_plugin/unused_clock");
    cm_options.arguments(cm_arguments);
    plugin->cm = std::make_shared<controller_manager::ControllerManager>(
      plugin->executor, plugin->config.robot_description, true, "controller_manager", "", cm_options);
    plugin->executor->add_node(plugin->cm);
    plugin->node = std::make_shared<rclcpp::Node>("hakoniwa_controller_plugin");
    plugin->executor->add_node(plugin->node);
    if (plugin->config.publish_clock) {
      plugin->clock = plugin->node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    }
    std::vector<SensorRelay::Channel> channels;
    for (uint32_t i = 0; i < args->pdu_channel_count; ++i) {
      channels.push_back({args->pdu_names[i], args->pdu_types[i]});
    }
    plugin->sensors = std::make_unique<SensorRelay>(plugin->node, std::move(channels),
                                                    plugin->config.sensor_topic_prefix);

    const unsigned int rate = plugin->cm->get_update_rate();
    if (rate == 0 || 1000000000ULL % rate != 0) {
      throw std::runtime_error("controller_manager update_rate must divide one second exactly");
    }
    plugin->period_ns = 1000000000ULL / rate;
    plugin->period = rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(plugin->period_ns));
    if (args->physics_step_ns == 0 || plugin->period_ns % args->physics_step_ns != 0) {
      throw std::runtime_error("the control period is not a whole multiple of the physics step");
    }

    for (const auto & name : plugin->config.activate) {
      if (!plugin->cm->load_controller(name)) {
        throw std::runtime_error("cannot load controller " + name);
      }
      if (plugin->cm->configure_controller(name) != controller_interface::return_type::OK) {
        throw std::runtime_error("cannot configure controller " + name);
      }
    }
    plugin->sim_clocks.push_back(plugin->cm->get_clock());
    for (const auto & controller : plugin->cm->get_loaded_controllers()) {
      plugin->sim_clocks.push_back(controller.c->get_node()->get_clock());
    }
    if (const char * trace_path = std::getenv("HAKO_CONTROLLER_PLUGIN_TRACE")) {
      plugin->trace = std::fopen(trace_path, "w");
    }
    plugin->spin = std::thread([executor = plugin->executor]() { executor->spin(); });
    plugin->switcher = std::make_unique<SwitchWorker>(*plugin->cm);
    return plugin.release();
  } catch (const std::exception & e) {
    copy_error(std::string("ros2_control plugin: ") + e.what(), error, error_size);
  }
  // Unwind a partly built plugin.
  plugin->switcher.reset();
  if (plugin->executor) {
    plugin->executor->cancel();
  }
  if (plugin->spin.joinable()) {
    plugin->spin.join();
  }
  plugin->sensors.reset();
  plugin->cm.reset();
  plugin->node.reset();
  plugin->executor.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  PluginBridge::instance().clear();
  g_instance_exists.store(false);
  return nullptr;
}

uint64_t update_period_ns(void * p)
{
  return static_cast<Ros2ControlPlugin *>(p)->period_ns;
}

int32_t on_start(void * p, char * error, size_t error_size)
{
  try {
    auto * plugin = static_cast<Ros2ControlPlugin *>(p);
    if (!plugin->activation_requested) {
      plugin->request_activation();   // applied inside the coming updates
    }
    return 0;
  } catch (const std::exception & e) {
    copy_error(e.what(), error, error_size);
    return -1;
  }
}

int32_t update(void * p, const HakoTickV1 * tick, const HakoJointStatesV1 * states,
               uint32_t pdu_count, const HakoPduViewV1 * pdus,
               HakoJointCommandsV1 * commands, char * error, size_t error_size)
{
  try {
    auto * plugin = static_cast<Ros2ControlPlugin *>(p);
    PluginBridge & bridge = PluginBridge::instance();
    if (states->count != bridge.size() || commands->count != bridge.size()) {
      throw std::runtime_error("joint count differs from create");
    }
    for (uint32_t i = 0; i < states->count; ++i) {
      bridge.set_state(i, states->position[i], states->velocity[i], states->effort[i]);
    }
    for (uint32_t i = 0; i < pdu_count; ++i) {
      plugin->sensors->post(pdus[i].channel, pdus[i].data, pdus[i].size);
    }
    const rclcpp::Time now(static_cast<int64_t>(tick->sim_time_ns), RCL_ROS_TIME);
    if (plugin->clock) {
      rosgraph_msgs::msg::Clock clock;
      clock.clock = now;
      plugin->clock->publish(clock);
    }

    // Start (when the runner gave no start event), reset, and take-over. The
    // reset flag of the first update after create/start is no reset of a run in
    // progress: the controllers are just being activated.
    if (!plugin->activation_requested) {
      plugin->request_activation();
    } else if (tick->reset && plugin->updated) {
      plugin->request_restart(true);
    } else if (plugin->was_selected && !tick->selected) {
      plugin->request_restart(false);          // another control took over: drop the trajectory
      plugin->preempted = true;
    } else if (plugin->preempted && tick->selected) {
      plugin->switcher->request(plugin->motion_controllers, {}, plugin->active);  // back: hold, then follow
      plugin->preempted = false;
    }
    plugin->was_selected = tick->selected != 0;
    plugin->updated = true;
    if (plugin->trace != nullptr) {
      const auto wall = std::chrono::steady_clock::now().time_since_epoch();
      std::fprintf(plugin->trace, "%llu %llu %d %d %d %lld\n",
                   static_cast<unsigned long long>(tick->step_index),
                   static_cast<unsigned long long>(tick->sim_time_ns),
                   tick->selected, tick->reset, plugin->active->load() ? 1 : 0,
                   static_cast<long long>(std::chrono::duration_cast<std::chrono::nanoseconds>(wall).count()));
    }

    bridge.clear_commands();
    // ros2_control refuses time 0 before it has received /clock: the step at time
    // 0 only holds the joints where they are.
    const bool at_time_zero = tick->sim_time_ns == 0;
    if (!at_time_zero) {
      plugin->set_sim_time(tick->sim_time_ns);
      plugin->cm->read(now, plugin->period);
      plugin->cm->update(now, plugin->period);
      plugin->cm->write(now, plugin->period);
    }

    const bool holding = at_time_zero || !plugin->active->load() || plugin->preempted;
    for (uint32_t i = 0; i < commands->count; ++i) {
      if (holding) {
        commands->has_command[i] = 1;
        commands->command_type[i] = HAKO_JOINT_COMMAND_POSITION;
        commands->value[i] = states->position[i];
      } else if (bridge.has_command(i)) {
        commands->has_command[i] = 1;
        commands->command_type[i] = bridge.command_type(i);
        commands->value[i] = bridge.command(i);
      }
    }
    if (plugin->trace != nullptr) {
      // The joints this step and the commands returned, exactly (hexadecimal floats):
      // "J step_index sim_time_ns position... | command...".
      std::fprintf(plugin->trace, "J %llu %llu",
                   static_cast<unsigned long long>(tick->step_index),
                   static_cast<unsigned long long>(tick->sim_time_ns));
      for (uint32_t i = 0; i < states->count; ++i) {
        std::fprintf(plugin->trace, " %a", states->position[i]);
      }
      std::fprintf(plugin->trace, " |");
      for (uint32_t i = 0; i < commands->count; ++i) {
        std::fprintf(plugin->trace, " %a", commands->has_command[i] ? commands->value[i] : 0.0);
      }
      std::fprintf(plugin->trace, "\n");
    }
    return 0;
  } catch (const std::exception & e) {
    copy_error(e.what(), error, error_size);
    return -1;
  }
}

int32_t on_stop(void * p, char * error, size_t error_size)
{
  try {
    auto * plugin = static_cast<Ros2ControlPlugin *>(p);
    plugin->active->store(false);
    if (rclcpp::ok()) {
      plugin->switcher->request({}, plugin->motion_controllers, nullptr);
    }
    plugin->activation_requested = false;
    plugin->updated = false;
    return 0;
  } catch (const std::exception & e) {
    copy_error(e.what(), error, error_size);
    return -1;
  }
}

int32_t reset(void * p, char * error, size_t error_size)
{
  // The restart itself is requested by the first update after the reset
  // (tick.reset), once the state after the reset is known.
  (void)p;
  (void)error;
  (void)error_size;
  return 0;
}

void destroy(void * p)
{
  auto * plugin = static_cast<Ros2ControlPlugin *>(p);
  if (plugin == nullptr) {
    return;
  }
  try {
    plugin->switcher.reset();
    if (rclcpp::ok()) {
      plugin->cm->shutdown_controllers();   // before the libraries they came from are released
    }
    plugin->executor->cancel();
    if (plugin->spin.joinable()) {
      plugin->spin.join();
    }
    plugin->sensors.reset();
    plugin->clock.reset();
    plugin->node.reset();
    plugin->cm.reset();
    plugin->executor.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  } catch (...) {
    // Nothing may cross the C boundary.
  }
  if (plugin->trace != nullptr) {
    std::fclose(plugin->trace);
  }
  delete plugin;
  PluginBridge::instance().clear();
  g_instance_exists.store(false);
}

const HakoControllerPluginV1 kPlugin = {
  {
    sizeof(HakoControllerPluginV1),
    HAKO_CONTROLLER_PLUGIN_ABI_VERSION,
    HAKO_PLUGIN_KIND_CONTROLLER,
    HAKO_CONTROLLER_PLUGIN_INTERFACE_REVISION,
    "hakoniwa_ros2_control",
  },
  create,
  update_period_ns,
  on_start,
  update,
  on_stop,
  reset,
  destroy,
};

}  // namespace
}  // namespace hakoniwa_ros2_control

extern "C" HAKO_PLUGIN_EXPORT const HakoPluginHeaderV1 * hako_robot_runtime_plugin_v1(void)
{
  return &hakoniwa_ros2_control::kPlugin.header;
}
