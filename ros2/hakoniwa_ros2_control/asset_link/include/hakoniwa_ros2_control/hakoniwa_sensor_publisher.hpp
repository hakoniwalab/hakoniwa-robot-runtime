#pragma once

// Publishes, on a thread of its own (real time), what HakoniwaAssetLink's asset
// thread read: the sensor PDUs as ROS 2 topics /pdu/<PDU name> and the
// Hakoniwa time as /clock. The asset thread only copies the raw PDUs; decoding
// and publishing here keeps it (and so the whole simulation) from waiting.
// Types: sensor_msgs/PointCloud2, geometry_msgs/WrenchStamped.
// The design is in docs/ros2-control.md, chapter 10.6.

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace hakoniwa_ros2_control
{

class HakoniwaAssetLink;
struct SensorPdu;

// Reads the sensor publish configuration: a JSON object of PDU name -> rate [Hz],
// e.g. {"lidar_scan": 2.0, "wrist_wrench": 50.0}. Throws on a bad file.
std::vector<SensorPdu> load_sensor_publish_config(const std::string & path);

class HakoniwaSensorPublisher
{
public:
  // link must outlive this publisher; its Config::sensors are the sensors published.
  HakoniwaSensorPublisher(rclcpp::Node::SharedPtr node, HakoniwaAssetLink & link,
                          std::vector<std::string> sensor_names, bool publish_clock);
  ~HakoniwaSensorPublisher();
  HakoniwaSensorPublisher(const HakoniwaSensorPublisher &) = delete;
  HakoniwaSensorPublisher & operator=(const HakoniwaSensorPublisher &) = delete;

  void start();
  void stop();

private:
  struct Topic;  // one sensor: its type and publisher

  void run();

  rclcpp::Node::SharedPtr node_;
  HakoniwaAssetLink & link_;
  std::vector<std::unique_ptr<Topic>> topics_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
};

}  // namespace hakoniwa_ros2_control
