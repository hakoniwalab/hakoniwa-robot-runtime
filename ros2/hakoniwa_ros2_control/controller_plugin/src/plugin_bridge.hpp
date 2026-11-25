// Joint states in and joint commands out between the controller plugin's update()
// and the hardware interface (HakoniwaPluginSystem). Both run on the Robot
// Runtime's simulation thread (the plugin calls ControllerManager::read/update/write
// inside update()), so there is no lock. One bridge per process: the hardware
// interface is created by ros2_control (pluginlib) from the URDF and reaches the
// plugin through instance(); both live in the same shared library.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hakoniwa_ros2_control
{

class PluginBridge
{
public:
  static PluginBridge & instance();

  // The joints in the Robot Runtime's order (its actuator ids). Called before the
  // ControllerManager loads the hardware interface.
  void configure(const std::vector<std::string> & joints);
  void clear();

  // Index of a joint by name, -1 when the Robot Runtime does not have it.
  int index(const std::string & joint) const;
  std::size_t size() const;

  void set_state(std::size_t index, double position, double velocity, double effort);
  double position(std::size_t index) const;
  double velocity(std::size_t index) const;
  double effort(std::size_t index) const;

  // command_type: 0 position, 1 velocity, 2 effort (HakoJointCommandsV1).
  void set_command_type(std::size_t index, std::uint8_t command_type);
  void set_command(std::size_t index, double value);
  void clear_commands();
  bool has_command(std::size_t index) const;
  std::uint8_t command_type(std::size_t index) const;
  double command(std::size_t index) const;

  // Force-torque sensors: the wrench PDUs the Robot Runtime passes to update(),
  // by PDU name, for the hardware interface's sensor state interfaces
  // (force.x, force.y, force.z, torque.x, torque.y, torque.z). update() sets the
  // latest value before ControllerManager::read(), so a controller reads the
  // wrench of the same control step.
  void configure_wrenches(const std::vector<std::string> & pdu_names);
  int wrench_index(const std::string & pdu_name) const;
  void set_wrench(std::size_t index, const std::array<double, 6> & wrench);
  bool has_wrench(std::size_t index) const;
  const std::array<double, 6> & wrench(std::size_t index) const;

private:
  std::vector<std::string> joints_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;
  std::vector<std::uint8_t> has_command_;
  std::vector<std::uint8_t> command_type_;
  std::vector<double> command_;
  std::vector<std::string> wrench_names_;
  std::vector<std::array<double, 6>> wrench_;
  std::vector<std::uint8_t> has_wrench_;
};

}  // namespace hakoniwa_ros2_control
