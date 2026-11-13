// SensorRelay: publishes the sensor PDUs the Robot Runtime passes to the controller
// plugin as ROS 2 messages on /pdu/<PDU name>, as HakoniwaSensorPublisher does in
// the separate-process form (same topics, same QoS). post() runs on the simulation
// thread and only keeps the latest bytes of a channel; a thread of its own decodes
// (pdu2ros) and publishes, so control never waits for a point cloud.
#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace hakoniwa_ros2_control
{

class SensorRelay
{
public:
  struct Channel
  {
    std::string name;  // PDU name: the topic is prefix + name
    std::string type;  // "sensor_msgs/PointCloud2" or "geometry_msgs/WrenchStamped"
  };

  // Throws std::runtime_error for a type it cannot publish.
  SensorRelay(rclcpp::Node::SharedPtr node, std::vector<Channel> channels, const std::string & topic_prefix);
  ~SensorRelay();

  // Copies the PDU bytes (the caller's buffer is valid only during the call).
  void post(std::uint32_t channel, const void * data, std::uint32_t size);

private:
  struct Slot;
  void run();
  void publish(Slot & slot, const std::vector<char> & pdu);

  rclcpp::Node::SharedPtr node_;
  std::vector<std::unique_ptr<Slot>> slots_;
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stop_{false};
  std::thread thread_;
};

}  // namespace hakoniwa_ros2_control
