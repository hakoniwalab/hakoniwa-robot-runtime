#include "hakoniwa_ros2_control/hakoniwa_sensor_publisher.hpp"

#include <chrono>
#include <fstream>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hakoniwa_ros2_control/hakoniwa_asset_link.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "geometry_msgs/pdu_cpptype_conv_WrenchStamped.hpp"
#include "sensor_msgs/pdu_cpptype_conv_PointCloud2.hpp"

namespace hakoniwa_ros2_control
{
namespace
{
constexpr auto kPoll = std::chrono::milliseconds(1);

builtin_interfaces::msg::Time to_ros(const HakoCpp_Time & t)
{
  builtin_interfaces::msg::Time out;
  out.sec = t.sec;
  out.nanosec = t.nanosec;
  return out;
}

std_msgs::msg::Header to_ros(const HakoCpp_Header & h)
{
  std_msgs::msg::Header out;
  out.stamp = to_ros(h.stamp);
  out.frame_id = h.frame_id;
  return out;
}
}  // namespace

std::vector<SensorPdu> load_sensor_publish_config(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open the sensor publish configuration " + path);
  }
  const auto json = nlohmann::json::parse(stream);
  if (!json.is_object()) {
    throw std::runtime_error(path + ": expected an object of PDU name -> rate [Hz]");
  }
  std::vector<SensorPdu> sensors;
  for (const auto & [name, rate] : json.items()) {
    if (!rate.is_number() || rate.get<double>() <= 0.0) {
      throw std::runtime_error(path + ": the rate of " + name + " must be a number > 0");
    }
    sensors.push_back({name, rate.get<double>()});
  }
  return sensors;
}

// One sensor: decodes its raw PDU (of the given type) and publishes the message.
struct HakoniwaSensorPublisher::Topic
{
  std::size_t index{0};
  std::string type;
  std::string name;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench;
  builtin_interfaces::msg::Time last_stamp;
  bool published{false};

  // False when the PDU cannot be decoded; a PDU with the stamp already
  // published is skipped (true).
  bool publish(std::vector<std::byte> & raw)
  {
    char * top = reinterpret_cast<char *>(raw.data());
    if (cloud) {
      HakoCpp_PointCloud2 pdu{};
      if (!hako::pdu::msgs::sensor_msgs::PointCloud2().pdu2cpp(top, pdu)) {
        return false;
      }
      sensor_msgs::msg::PointCloud2 msg;
      msg.header = to_ros(pdu.header);
      if (published && msg.header.stamp == last_stamp) {
        return true;
      }
      msg.height = pdu.height;
      msg.width = pdu.width;
      for (const auto & f : pdu.fields) {
        sensor_msgs::msg::PointField field;
        field.name = f.name;
        field.offset = f.offset;
        field.datatype = f.datatype;
        field.count = f.count;
        msg.fields.push_back(field);
      }
      msg.is_bigendian = pdu.is_bigendian;
      msg.point_step = pdu.point_step;
      msg.row_step = pdu.row_step;
      msg.data = std::move(pdu.data);
      msg.is_dense = pdu.is_dense;
      last_stamp = msg.header.stamp;
      published = true;
      cloud->publish(std::move(msg));
      return true;
    }
    HakoCpp_WrenchStamped pdu{};
    if (!hako::pdu::msgs::geometry_msgs::WrenchStamped().pdu2cpp(top, pdu)) {
      return false;
    }
    geometry_msgs::msg::WrenchStamped msg;
    msg.header = to_ros(pdu.header);
    if (published && msg.header.stamp == last_stamp) {
      return true;
    }
    msg.wrench.force.x = pdu.wrench.force.x;
    msg.wrench.force.y = pdu.wrench.force.y;
    msg.wrench.force.z = pdu.wrench.force.z;
    msg.wrench.torque.x = pdu.wrench.torque.x;
    msg.wrench.torque.y = pdu.wrench.torque.y;
    msg.wrench.torque.z = pdu.wrench.torque.z;
    last_stamp = msg.header.stamp;
    published = true;
    wrench->publish(msg);
    return true;
  }
};

HakoniwaSensorPublisher::HakoniwaSensorPublisher(rclcpp::Node::SharedPtr node, HakoniwaAssetLink & link,
                                                 std::vector<std::string> sensor_names, bool publish_clock)
: node_(std::move(node)), link_(link)
{
  for (std::size_t i = 0; i < sensor_names.size(); ++i) {
    auto topic = std::make_unique<Topic>();
    topic->index = i;
    topic->name = sensor_names[i];
    topic->type = link_.sensor_type(i);
    const std::string name = "/pdu/" + sensor_names[i];  // as the Hakoniwa ROS 2 Bridge names it
    if (topic->type == "sensor_msgs/PointCloud2") {
      topic->cloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name, rclcpp::SensorDataQoS());
    } else if (topic->type == "geometry_msgs/WrenchStamped") {
      topic->wrench = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(name, rclcpp::SensorDataQoS());
    } else {
      throw std::runtime_error("sensor " + sensor_names[i] + ": type " + topic->type + " is not published");
    }
    topics_.push_back(std::move(topic));
  }
  if (publish_clock) {
    clock_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  }
}

HakoniwaSensorPublisher::~HakoniwaSensorPublisher()
{
  stop();
}

void HakoniwaSensorPublisher::start()
{
  if (thread_.joinable()) {
    return;
  }
  stop_ = false;
  thread_ = std::thread([this]() { run(); });
}

void HakoniwaSensorPublisher::stop()
{
  stop_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
}

void HakoniwaSensorPublisher::run()
{
  SensorSample sample;
  std::uint64_t last_clock_usec = 0;
  while (!stop_) {
    std::this_thread::sleep_for(kPoll);
    // The Hakoniwa time: this asset's time, published when it has advanced.
    const std::uint64_t now = link_.asset_time_usec();
    if (clock_ && now != last_clock_usec) {
      rosgraph_msgs::msg::Clock clock;
      clock.clock.sec = static_cast<int32_t>(now / 1000000);
      clock.clock.nanosec = static_cast<uint32_t>((now % 1000000) * 1000);
      clock_->publish(clock);
      last_clock_usec = now;
    }
    for (auto & topic : topics_) {
      if (link_.take_sensor(topic->index, sample) && !topic->publish(sample.data)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "cannot decode the sensor PDU %s (%s)", topic->name.c_str(), topic->type.c_str());
      }
    }
  }
}

}  // namespace hakoniwa_ros2_control
