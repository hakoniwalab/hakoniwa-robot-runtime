#pragma once

// ros2_control hardware for a Hakoniwa robot. It touches no Hakoniwa PDU itself:
// the Hakoniwa ROS 2 Bridge turns the PDUs into ROS 2 topics.
//   read():  the latest joint states topic (default /pdu/joint_states) -> state interfaces
//   write(): each joint's command interface -> its std_msgs/Float64 topic
//            (default /actuator/<joint>/command)
// URDF parameters: hardware "state_topic"; joint "command_topic".

#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

namespace hakoniwa_ros2_control
{

class HakoniwaSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    std::string name;
    std::string command_interface;  // "<joint>/position" (or velocity, effort)
    std::string command_topic;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;
    bool has_position{false};
    bool has_velocity{false};
    bool has_effort{false};
  };

  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr message);
  // Copies the joint states message into the state interfaces. False until a
  // message has carried every configured joint.
  bool apply_joint_states(const sensor_msgs::msg::JointState & message);

  std::string state_topic_{"/pdu/joint_states"};
  std::vector<Joint> joints_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

  std::mutex mutex_;  // the latest message, written by the subscription, read by read()
  std::optional<sensor_msgs::msg::JointState> latest_;
};

}  // namespace hakoniwa_ros2_control
