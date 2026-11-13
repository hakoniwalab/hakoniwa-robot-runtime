// HakoniwaPluginSystem: the ros2_control hardware interface of the controller
// plugin (design: docs/design-controller-plugin.md 5.3). ros2_control creates it
// from the URDF's <hardware><plugin>hakoniwa_ros2_control/HakoniwaPluginSystem</plugin>;
// read() and write() run inside the plugin's update(), on the Robot Runtime's
// simulation thread, and exchange joint values with the PluginBridge. No PDU, no
// Hakoniwa asset.
#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "plugin_bridge.hpp"

namespace hakoniwa_ros2_control
{

class HakoniwaPluginSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }
    const PluginBridge & bridge = PluginBridge::instance();
    joints_.clear();
    for (const auto & info : get_hardware_info().joints) {
      Joint joint;
      joint.name = info.name;
      const int index = bridge.index(joint.name);
      if (index < 0) {
        RCLCPP_ERROR(get_logger(), "joint %s is not a joint of the Robot Runtime's plugin component",
                     joint.name.c_str());
        return CallbackReturn::ERROR;
      }
      joint.index = static_cast<std::size_t>(index);
      if (info.command_interfaces.size() != 1) {
        RCLCPP_ERROR(get_logger(), "joint %s needs exactly one command interface", joint.name.c_str());
        return CallbackReturn::ERROR;
      }
      const std::string & command = info.command_interfaces[0].name;
      if (command == hardware_interface::HW_IF_POSITION) {
        joint.command_type = 0;
      } else if (command == hardware_interface::HW_IF_VELOCITY) {
        joint.command_type = 1;
      } else if (command == hardware_interface::HW_IF_EFFORT) {
        joint.command_type = 2;
      } else {
        RCLCPP_ERROR(get_logger(), "joint %s: unsupported command interface %s", joint.name.c_str(),
                     command.c_str());
        return CallbackReturn::ERROR;
      }
      joint.command_interface = joint.name + "/" + command;
      for (const auto & state : info.state_interfaces) {
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
      joints_.push_back(joint);
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // Commands start where the joints are, so activating moves nothing.
    read(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration(0, 0));
    for (const auto & joint : joints_) {
      if (joint.command_type == 0 && joint.has_position) {
        set_command(joint.command_interface, get_state(joint.name + "/" + hardware_interface::HW_IF_POSITION));
      }
    }
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    const PluginBridge & bridge = PluginBridge::instance();
    for (const auto & joint : joints_) {
      if (joint.has_position) {
        set_state(joint.name + "/" + hardware_interface::HW_IF_POSITION, bridge.position(joint.index));
      }
      if (joint.has_velocity) {
        set_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, bridge.velocity(joint.index));
      }
      if (joint.has_effort) {
        set_state(joint.name + "/" + hardware_interface::HW_IF_EFFORT, bridge.effort(joint.index));
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    PluginBridge & bridge = PluginBridge::instance();
    for (const auto & joint : joints_) {
      const double value = get_command(joint.command_interface);
      if (!std::isfinite(value)) {
        continue;  // no command for this joint yet
      }
      bridge.set_command_type(joint.index, joint.command_type);
      bridge.set_command(joint.index, value);
    }
    return hardware_interface::return_type::OK;
  }

private:
  struct Joint
  {
    std::string name;
    std::size_t index{0};
    std::string command_interface;
    std::uint8_t command_type{0};
    bool has_position{false};
    bool has_velocity{false};
    bool has_effort{false};
  };
  std::vector<Joint> joints_;
};

}  // namespace hakoniwa_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hakoniwa_ros2_control::HakoniwaPluginSystem, hardware_interface::SystemInterface)
