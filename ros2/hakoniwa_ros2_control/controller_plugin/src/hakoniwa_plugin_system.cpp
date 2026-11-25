// HakoniwaPluginSystem: the ros2_control hardware interface of the controller
// plugin (design: docs/design-controller-plugin.md 5.3). ros2_control creates it
// from the URDF's <hardware><plugin>hakoniwa_ros2_control/HakoniwaPluginSystem</plugin>;
// read() and write() run inside the plugin's update(), on the Robot Runtime's
// simulation thread, and exchange joint values with the PluginBridge. No PDU, no
// Hakoniwa asset.
//
// Force-torque sensors: a URDF <sensor> whose state interfaces are among force.x,
// force.y, force.z, torque.x, torque.y, torque.z (the names of ros2_control's
// ForceTorqueSensor) reads the wrench PDU named by its parameter "pdu" (default:
// the sensor's name), the latest value the Robot Runtime passed to update() for
// this control step. Before the first value the interfaces read 0.
#include <cmath>
#include <string>
#include <utility>
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
    static const char * const axes[6] = {"force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};
    sensors_.clear();
    for (const auto & info : get_hardware_info().sensors) {
      Sensor sensor;
      const auto pdu = info.parameters.find("pdu");
      const std::string pdu_name = pdu != info.parameters.end() ? pdu->second : info.name;
      const int index = bridge.wrench_index(pdu_name);
      if (index < 0) {
        RCLCPP_ERROR(get_logger(), "sensor %s: no wrench PDU %s among the plugin component's pdus",
                     info.name.c_str(), pdu_name.c_str());
        return CallbackReturn::ERROR;
      }
      sensor.index = static_cast<std::size_t>(index);
      for (const auto & state : info.state_interfaces) {
        int axis = -1;
        for (int a = 0; a < 6; ++a) {
          if (state.name == axes[a]) {
            axis = a;
          }
        }
        if (axis < 0) {
          RCLCPP_ERROR(get_logger(), "sensor %s: unsupported state interface %s", info.name.c_str(),
                       state.name.c_str());
          return CallbackReturn::ERROR;
        }
        sensor.interfaces.push_back({info.name + "/" + state.name, axis});
      }
      sensors_.push_back(sensor);
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
    for (const auto & sensor : sensors_) {
      if (!bridge.has_wrench(sensor.index)) {
        continue;
      }
      const auto & wrench = bridge.wrench(sensor.index);
      for (const auto & [name, axis] : sensor.interfaces) {
        set_state(name, wrench[axis]);
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
  struct Sensor
  {
    std::size_t index{0};                               // in the bridge's wrenches
    std::vector<std::pair<std::string, int>> interfaces;  // "<sensor>/force.x" and its axis
  };
  std::vector<Sensor> sensors_;
};

}  // namespace hakoniwa_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hakoniwa_ros2_control::HakoniwaPluginSystem, hardware_interface::SystemInterface)
