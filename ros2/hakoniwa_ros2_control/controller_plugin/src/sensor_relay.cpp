#include "sensor_relay.hpp"

#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "geometry_msgs/pdu_ctype_conv_WrenchStamped.hpp"
#include "sensor_msgs/pdu_ctype_conv_PointCloud2.hpp"

namespace hakoniwa_ros2_control
{

struct SensorRelay::Slot
{
  Channel channel;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench;
  sensor_msgs::msg::PointCloud2 cloud_msg;          // kept: no arrays allocated per message
  geometry_msgs::msg::WrenchStamped wrench_msg;
  std::vector<char> pending;                        // latest bytes from post()
  bool has_pending{false};
  std::vector<char> working;                        // the bytes being published
};

SensorRelay::SensorRelay(rclcpp::Node::SharedPtr node, std::vector<Channel> channels, const std::string & topic_prefix)
: node_(std::move(node))
{
  for (auto & channel : channels) {
    auto slot = std::make_unique<Slot>();
    const std::string topic = topic_prefix + channel.name;
    if (channel.type == "sensor_msgs/PointCloud2") {
      slot->cloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic, rclcpp::SensorDataQoS());
    } else if (channel.type == "geometry_msgs/WrenchStamped") {
      slot->wrench = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(topic, rclcpp::SensorDataQoS());
    } else {
      throw std::runtime_error("sensor PDU " + channel.name + ": unsupported type " + channel.type);
    }
    slot->channel = std::move(channel);
    slots_.push_back(std::move(slot));
  }
  thread_ = std::thread([this]() { run(); });
}

SensorRelay::~SensorRelay()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  wake_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void SensorRelay::post(std::uint32_t channel, const void * data, std::uint32_t size)
{
  if (channel >= slots_.size() || data == nullptr) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Slot & slot = *slots_[channel];
    const auto * bytes = static_cast<const char *>(data);
    slot.pending.assign(bytes, bytes + size);   // capacity is kept across posts
    slot.has_pending = true;
  }
  wake_.notify_one();
}

void SensorRelay::run()
{
  while (true) {
    Slot * next = nullptr;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this]() {
        if (stop_) {
          return true;
        }
        for (const auto & slot : slots_) {
          if (slot->has_pending) {
            return true;
          }
        }
        return false;
      });
      if (stop_) {
        return;
      }
      for (auto & slot : slots_) {
        if (slot->has_pending) {
          slot->working.swap(slot->pending);
          slot->has_pending = false;
          next = slot.get();
          break;
        }
      }
    }
    if (next != nullptr) {
      publish(*next, next->working);
    }
  }
}

void SensorRelay::publish(Slot & slot, const std::vector<char> & pdu)
{
  void * base = hako_get_base_ptr_pdu(const_cast<char *>(pdu.data()));
  if (base == nullptr) {
    RCLCPP_WARN(node_->get_logger(), "sensor PDU %s: invalid PDU", slot.channel.name.c_str());
    return;
  }
  if (slot.cloud) {
    // Emptied first (capacity kept): an array empty in the PDU must not keep the
    // previous message's elements with converters that leave it as it was.
    slot.cloud_msg.fields.clear();
    slot.cloud_msg.data.clear();
    if (hako_convert_pdu2ros_PointCloud2(*static_cast<Hako_PointCloud2 *>(base), slot.cloud_msg) != 0) {
      RCLCPP_WARN(node_->get_logger(), "sensor PDU %s: cannot decode", slot.channel.name.c_str());
      return;
    }
    slot.cloud->publish(slot.cloud_msg);
    return;
  }
  if (hako_convert_pdu2ros_WrenchStamped(*static_cast<Hako_WrenchStamped *>(base), slot.wrench_msg) != 0) {
    RCLCPP_WARN(node_->get_logger(), "sensor PDU %s: cannot decode", slot.channel.name.c_str());
    return;
  }
  slot.wrench->publish(slot.wrench_msg);
}

}  // namespace hakoniwa_ros2_control
