#pragma once

// ros2_control hardware for a Hakoniwa robot that is itself a Hakoniwa asset:
// it reads and writes the joint PDUs in shared memory (HakoniwaAssetLink),
// without the Hakoniwa ROS 2 Bridge. For ROS 2 and Hakoniwa on the same Linux
// (Ubuntu, Docker only). HakoniwaSystem (through the Bridge) is unchanged.
//   read():  the latest joint states the asset thread read -> state interfaces;
//            ERROR when they are older than state_timeout_sec (wall clock)
//   write(): all joints' command interfaces -> one command for the asset thread
// URDF hardware parameters: asset_name (HakoniwaSystem), pdu_definition_path,
// endpoint_config_path, robot_name, delta_usec (2000), state_timeout_sec (0.5),
// first_state_timeout_sec (60), sensor_publish_config (a JSON file of sensor PDU
// name -> rate [Hz]; published as /pdu/<name>), publish_clock (true: /clock from
// the Hakoniwa time). Each joint: one command interface, position (velocity,
// effort) state interfaces.
// The design is in docs/ros2-control.md, chapter 10.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hakoniwa_ros2_control
{

class HakoniwaAssetLink;
class HakoniwaSensorPublisher;
struct JointStateSample;

class HakoniwaAssetSystem : public hardware_interface::SystemInterface
{
public:
  HakoniwaAssetSystem();
  ~HakoniwaAssetSystem() override;

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    std::string name;
    std::string command_interface;  // "<joint>/position" (or velocity, effort)
    bool has_position{false};
    bool has_velocity{false};
    bool has_effort{false};
  };

  // Copies a sample into the state interfaces. False unless it carries every configured joint.
  bool apply(const JointStateSample & sample);
  void hold_position();

  std::vector<Joint> joints_;
  std::unique_ptr<HakoniwaAssetLink> link_;
  std::unique_ptr<HakoniwaSensorPublisher> publisher_;  // sensors and /clock
  bool started_{false};
  double state_timeout_sec_{0.5};
  double first_state_timeout_sec_{60.0};
  std::uint64_t applied_sequence_{0};
};

}  // namespace hakoniwa_ros2_control
