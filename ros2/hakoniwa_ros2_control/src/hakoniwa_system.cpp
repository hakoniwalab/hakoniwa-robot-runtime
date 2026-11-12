#include "hakoniwa_ros2_control/hakoniwa_system.hpp"

#include <chrono>
#include <cmath>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/wait_for_message.hpp"

namespace
{
constexpr auto kFirstStateTimeout = std::chrono::seconds(10);
}  // namespace

namespace hakoniwa_ros2_control
{

HakoniwaSystem::CallbackReturn HakoniwaSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & info = get_hardware_info();
  const auto state_topic = info.hardware_parameters.find("state_topic");
  if (state_topic != info.hardware_parameters.end()) {
    state_topic_ = state_topic->second;
  }
  joints_.clear();
  for (const auto & joint_info : info.joints) {
    Joint joint;
    joint.name = joint_info.name;
    // One Float64 topic per joint carries one command; the Runtime's actuator
    // configuration decides whether it is a position, a velocity or a torque.
    if (joint_info.command_interfaces.size() != 1) {
      RCLCPP_ERROR(get_logger(), "joint %s needs exactly one command interface", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    joint.command_interface = joint.name + "/" + joint_info.command_interfaces[0].name;
    const auto topic = joint_info.parameters.find("command_topic");
    joint.command_topic = topic != joint_info.parameters.end() ? topic->second
                                                                : "/actuator/" + joint.name + "/command";
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

HakoniwaSystem::CallbackReturn HakoniwaSystem::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  // The bridge publishes the joint states best effort (sensor data).
  subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) { on_joint_states(message); });
  for (auto & joint : joints_) {
    joint.publisher = node->create_publisher<std_msgs::msg::Float64>(joint.command_topic, rclcpp::QoS(10));
  }
  RCLCPP_INFO(get_logger(), "reading %s, writing %zu command topics", state_topic_.c_str(), joints_.size());
  return CallbackReturn::SUCCESS;
}

HakoniwaSystem::CallbackReturn HakoniwaSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // Start from where the robot is: wait for the first joint states and hold
  // them, so that the state interfaces never carry a made-up value and no
  // command moves the robot before a controller does. The wait uses a node of
  // its own: this component's subscription is not served during activation.
  auto waiter = std::make_shared<rclcpp::Node>(
    "hakoniwa_system_first_state", rclcpp::NodeOptions().start_parameter_services(false));
  sensor_msgs::msg::JointState first;
  if (!rclcpp::wait_for_message(first, waiter, state_topic_, kFirstStateTimeout, rclcpp::SensorDataQoS())) {
    RCLCPP_ERROR(get_logger(),
                 "no %s within %lld s: is the Hakoniwa ROS 2 Bridge running, in the same ROS_DOMAIN_ID?",
                 state_topic_.c_str(), static_cast<long long>(kFirstStateTimeout.count()));
    return CallbackReturn::ERROR;
  }
  if (!apply_joint_states(first)) {
    RCLCPP_ERROR(get_logger(), "%s does not carry every configured joint", state_topic_.c_str());
    return CallbackReturn::ERROR;
  }
  for (const auto & joint : joints_) {
    set_command(joint.command_interface, get_state(joint.name + "/" + hardware_interface::HW_IF_POSITION));
  }
  return CallbackReturn::SUCCESS;
}

HakoniwaSystem::CallbackReturn HakoniwaSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void HakoniwaSystem::on_joint_states(const sensor_msgs::msg::JointState::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = *message;
}

bool HakoniwaSystem::apply_joint_states(const sensor_msgs::msg::JointState & message)
{
  bool complete = true;
  for (const auto & joint : joints_) {
    std::size_t k = 0;
    while (k < message.name.size() && message.name[k] != joint.name) {
      ++k;
    }
    if (k == message.name.size()) {
      complete = false;
      continue;
    }
    if (joint.has_position && k < message.position.size()) {
      set_state(joint.name + "/" + hardware_interface::HW_IF_POSITION, message.position[k]);
    }
    if (joint.has_velocity && k < message.velocity.size()) {
      set_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, message.velocity[k]);
    }
    if (joint.has_effort && k < message.effort.size()) {
      set_state(joint.name + "/" + hardware_interface::HW_IF_EFFORT, message.effort[k]);
    }
  }
  return complete;
}

hardware_interface::return_type HakoniwaSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::optional<sensor_msgs::msg::JointState> message;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    message.swap(latest_);
  }
  if (message) {
    apply_joint_states(*message);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HakoniwaSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (const auto & joint : joints_) {
    const double value = get_command(joint.command_interface);
    if (!std::isfinite(value)) {
      continue;  // no command yet
    }
    std_msgs::msg::Float64 message;
    message.data = value;
    joint.publisher->publish(message);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace hakoniwa_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hakoniwa_ros2_control::HakoniwaSystem, hardware_interface::SystemInterface)
