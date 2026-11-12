#include "hakoniwa_ros2_control/hakoniwa_asset_system.hpp"

#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

#include "hakoniwa_ros2_control/hakoniwa_asset_link.hpp"
#include "hakoniwa_ros2_control/hakoniwa_sensor_publisher.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace hakoniwa_ros2_control
{
namespace
{
std::string parameter(const hardware_interface::HardwareInfo & info, const std::string & name,
                      const std::string & fallback)
{
  const auto found = info.hardware_parameters.find(name);
  return found != info.hardware_parameters.end() ? found->second : fallback;
}
}  // namespace

HakoniwaAssetSystem::HakoniwaAssetSystem() = default;

HakoniwaAssetSystem::~HakoniwaAssetSystem()
{
  publisher_.reset();  // before the link it reads
  link_.reset();       // stops the asset thread, closes the Endpoint
}

HakoniwaAssetSystem::CallbackReturn HakoniwaAssetSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & info = get_hardware_info();
  state_timeout_sec_ = std::stod(parameter(info, "state_timeout_sec", "0.5"));
  first_state_timeout_sec_ = std::stod(parameter(info, "first_state_timeout_sec", "60"));
  joints_.clear();
  for (const auto & joint_info : info.joints) {
    Joint joint;
    joint.name = joint_info.name;
    // One Float64 PDU per joint carries one command; the Runtime's actuator
    // configuration decides whether it is a position, a velocity or a torque.
    if (joint_info.command_interfaces.size() != 1) {
      RCLCPP_ERROR(get_logger(), "joint %s needs exactly one command interface", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    joint.command_interface = joint.name + "/" + joint_info.command_interfaces[0].name;
    for (const auto & state : joint_info.state_interfaces) {
      if (state.name == hardware_interface::HW_IF_POSITION) {
        joint.has_position = true;
      } else if (state.name == hardware_interface::HW_IF_VELOCITY) {
        joint.has_velocity = true;
      } else if (state.name == hardware_interface::HW_IF_EFFORT) {
        joint.has_effort = true;
      } else {
        RCLCPP_ERROR(get_logger(), "joint %s: unsupported state interface %s", joint.name.c_str(),
                     state.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    joints_.push_back(std::move(joint));
  }
  return CallbackReturn::SUCCESS;
}

HakoniwaAssetSystem::CallbackReturn HakoniwaAssetSystem::on_configure(const rclcpp_lifecycle::State &)
{
  if (link_) {
    return CallbackReturn::SUCCESS;  // a Hakoniwa asset registers once per process
  }
  const auto & info = get_hardware_info();
  HakoniwaAssetLink::Config config;
  config.asset_name = parameter(info, "asset_name", "HakoniwaSystem");
  config.pdu_definition_path = parameter(info, "pdu_definition_path", "");
  config.endpoint_config_path = parameter(info, "endpoint_config_path", "");
  config.robot_name = parameter(info, "robot_name", "");
  config.delta_usec = std::stoull(parameter(info, "delta_usec", "2000"));
  // false: no wall-clock pacing here (another asset, e.g. the Robot Runtime, keeps
  // real time, or the simulation runs as fast as it can); the world time is still waited for.
  config.realtime_sync = parameter(info, "realtime_sync", "true") == "true";
  for (const auto & joint : joints_) {
    config.command_joints.push_back(joint.name);
  }
  const auto sensor_config = parameter(info, "sensor_publish_config", "");
  if (!sensor_config.empty()) {
    try {
      config.sensors = load_sensor_publish_config(sensor_config);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return CallbackReturn::ERROR;
    }
  }
  if (config.pdu_definition_path.empty() || config.endpoint_config_path.empty() || config.robot_name.empty()) {
    RCLCPP_ERROR(get_logger(), "needs the hardware parameters pdu_definition_path, endpoint_config_path, robot_name");
    return CallbackReturn::ERROR;
  }
  link_ = std::make_unique<HakoniwaAssetLink>(config);
  // The Hakoniwa core accepts a new asset only while the simulation is stopped.
  if (!link_->register_asset()) {
    RCLCPP_ERROR(get_logger(), "cannot register the Hakoniwa asset %s: start ros2_control before the simulation starts",
                 config.asset_name.c_str());
    link_.reset();
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Hakoniwa asset %s registered (robot %s, %zu joints, %zu sensors, dT %llu usec)",
              config.asset_name.c_str(), config.robot_name.c_str(), joints_.size(), config.sensors.size(),
              static_cast<unsigned long long>(config.delta_usec));
  std::vector<std::string> sensor_names;
  for (const auto & sensor : config.sensors) {
    sensor_names.push_back(sensor.name);
  }
  const bool publish_clock = parameter(info, "publish_clock", "true") == "true";
  try {
    publisher_ = std::make_unique<HakoniwaSensorPublisher>(get_node(), *link_, sensor_names, publish_clock);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

HakoniwaAssetSystem::CallbackReturn HakoniwaAssetSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!link_) {
    return CallbackReturn::ERROR;
  }
  if (!started_) {
    started_ = link_->start();
    if (publisher_) {
      publisher_->start();
    }
  }
  // Start from where the robot is: wait for the first joint states. They come
  // once the simulation has started.
  JointStateSample first;
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(first_state_timeout_sec_);
  while (!link_->latest_state(first)) {
    if (std::chrono::steady_clock::now() > end || link_->finished()) {
      RCLCPP_ERROR(get_logger(), "no joint states from Hakoniwa within %.0f s: has the simulation started?",
                   first_state_timeout_sec_);
      return CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!apply(first)) {
    RCLCPP_ERROR(get_logger(), "the Hakoniwa joint states do not carry every configured joint");
    return CallbackReturn::ERROR;
  }
  applied_sequence_ = first.sequence;
  hold_position();
  RCLCPP_INFO(get_logger(), "first joint states at Hakoniwa time %.3f s", first.stamp_sec);
  return CallbackReturn::SUCCESS;
}

HakoniwaAssetSystem::CallbackReturn HakoniwaAssetSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Hold where the robot is. The asset thread keeps stepping (stopping it would
  // hold the whole simulation); it writes this command once, then nothing new.
  hold_position();
  (void)write(rclcpp::Time(0), rclcpp::Duration(0, 0));
  return CallbackReturn::SUCCESS;
}

HakoniwaAssetSystem::CallbackReturn HakoniwaAssetSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  publisher_.reset();  // before the link it reads
  link_.reset();       // stops the asset thread and closes the PDU Endpoint
  started_ = false;
  return CallbackReturn::SUCCESS;
}

HakoniwaAssetSystem::CallbackReturn HakoniwaAssetSystem::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

hardware_interface::return_type HakoniwaAssetSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  JointStateSample sample;
  if (!link_ || !link_->latest_state(sample)) {
    return hardware_interface::return_type::ERROR;
  }
  // Too old: the simulation (or the Robot Runtime) has stopped.
  const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - sample.received).count();
  if (age > state_timeout_sec_ || link_->finished()) {
    RCLCPP_ERROR(get_logger(), "Hakoniwa joint states are %.2f s old (limit %.2f s)", age, state_timeout_sec_);
    return hardware_interface::return_type::ERROR;
  }
  if (sample.sequence != applied_sequence_) {
    apply(sample);
    applied_sequence_ = sample.sequence;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HakoniwaAssetSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!link_) {
    return hardware_interface::return_type::ERROR;
  }
  std::vector<double> values;
  values.reserve(joints_.size());
  for (const auto & joint : joints_) {
    const double value = get_command(joint.command_interface);
    if (!std::isfinite(value)) {
      return hardware_interface::return_type::OK;  // no command yet
    }
    values.push_back(value);
  }
  link_->put_command(values);
  return hardware_interface::return_type::OK;
}

bool HakoniwaAssetSystem::apply(const JointStateSample & sample)
{
  bool complete = true;
  for (const auto & joint : joints_) {
    std::size_t k = 0;
    while (k < sample.name.size() && sample.name[k] != joint.name) {
      ++k;
    }
    if (k == sample.name.size()) {
      complete = false;
      continue;
    }
    if (joint.has_position && k < sample.position.size()) {
      set_state(joint.name + "/" + hardware_interface::HW_IF_POSITION, sample.position[k]);
    }
    if (joint.has_velocity && k < sample.velocity.size()) {
      set_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, sample.velocity[k]);
    }
    if (joint.has_effort && k < sample.effort.size()) {
      set_state(joint.name + "/" + hardware_interface::HW_IF_EFFORT, sample.effort[k]);
    }
  }
  return complete;
}

void HakoniwaAssetSystem::hold_position()
{
  for (const auto & joint : joints_) {
    set_command(joint.command_interface, get_state(joint.name + "/" + hardware_interface::HW_IF_POSITION));
  }
}

}  // namespace hakoniwa_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hakoniwa_ros2_control::HakoniwaAssetSystem, hardware_interface::SystemInterface)
