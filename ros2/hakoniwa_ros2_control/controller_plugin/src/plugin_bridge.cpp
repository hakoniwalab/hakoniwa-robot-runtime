#include "plugin_bridge.hpp"

namespace hakoniwa_ros2_control
{

PluginBridge & PluginBridge::instance()
{
  static PluginBridge bridge;
  return bridge;
}

void PluginBridge::configure(const std::vector<std::string> & joints)
{
  joints_ = joints;
  const std::size_t count = joints_.size();
  position_.assign(count, 0.0);
  velocity_.assign(count, 0.0);
  effort_.assign(count, 0.0);
  has_command_.assign(count, 0);
  command_type_.assign(count, 0);
  command_.assign(count, 0.0);
}

void PluginBridge::clear()
{
  configure({});
}

int PluginBridge::index(const std::string & joint) const
{
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i] == joint) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

std::size_t PluginBridge::size() const
{
  return joints_.size();
}

void PluginBridge::set_state(std::size_t index, double position, double velocity, double effort)
{
  position_[index] = position;
  velocity_[index] = velocity;
  effort_[index] = effort;
}

double PluginBridge::position(std::size_t index) const
{
  return position_[index];
}

double PluginBridge::velocity(std::size_t index) const
{
  return velocity_[index];
}

double PluginBridge::effort(std::size_t index) const
{
  return effort_[index];
}

void PluginBridge::set_command_type(std::size_t index, std::uint8_t command_type)
{
  command_type_[index] = command_type;
}

void PluginBridge::set_command(std::size_t index, double value)
{
  command_[index] = value;
  has_command_[index] = 1;
}

void PluginBridge::clear_commands()
{
  for (auto & flag : has_command_) {
    flag = 0;
  }
}

bool PluginBridge::has_command(std::size_t index) const
{
  return has_command_[index] != 0;
}

std::uint8_t PluginBridge::command_type(std::size_t index) const
{
  return command_type_[index];
}

double PluginBridge::command(std::size_t index) const
{
  return command_[index];
}

}  // namespace hakoniwa_ros2_control
