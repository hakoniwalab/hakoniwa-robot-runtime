// One test driver, two transports: sensor_msgs/PointCloud2 from a sender to a
// receiver either as ROS 2 messages or as a Hakoniwa PDU in shared memory. Only
// the transport differs; the payload, the timing and the checks are shared.
//
//   transport_bench --transport ros|pdu --role send|recv --size <bytes> --rate <Hz>
//                   --count <n> [--out <json>] [--pdu-config <pdudef>] [--pdu-size <bytes>]
//                   [--poll-us <us>] [--qos reliable|best_effort] [--trace <file>]
//
// Latency is measured type to type: from the sender holding a ROS message to
// the receiver holding a ROS message again (the PDU path includes the PDU
// conversion both ways, the ROS path the serialisation both ways). Both ends
// use CLOCK_MONOTONIC (the same clock in one machine).
//
// The PDU ends are Hakoniwa assets that never advance time: each runs its loop
// in on_manual_timing_control without sleeping in Hakoniwa time, so no time
// step is in the path (an external process cannot attach the PDU memory while
// another process created it).
#include <sys/resource.h>
#include <time.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "hako_asset.h"
#include "sensor_msgs/pdu_cpptype_conv_PointCloud2.hpp"

namespace
{
struct Options
{
  std::string transport;
  std::string role;
  std::size_t size{1024};
  double rate{100.0};
  std::uint64_t count{1000};
  std::string out;
  std::string pdu_config;
  std::size_t pdu_size{0};
  int poll_us{0};
  std::string qos{"reliable"};
  std::string trace;  // receiver: one line per message, "sequence latency_us"
};

constexpr const char * kRobot = "Bench";
constexpr HakoPduChannelIdType kChannel = 0;
constexpr const char * kTopic = "/bench/cloud";
constexpr std::size_t kHead = 16;  // sequence (8 bytes) and send time [ns] (8 bytes)

std::int64_t now_ns()
{
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

double cpu_seconds()
{
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec * 1e-6 +
         usage.ru_stime.tv_sec + usage.ru_stime.tv_usec * 1e-6;
}

std::uint8_t pattern(std::uint64_t seq, std::size_t i)
{
  return static_cast<std::uint8_t>((seq * 31u + i) & 0xffu);
}

// The payload: a one-row cloud of size bytes (x, y, z float32 per point is
// declared; the bytes carry the sequence, the send time and a pattern).
void fill(sensor_msgs::msg::PointCloud2 & msg, std::uint64_t seq, std::size_t size)
{
  if (msg.fields.empty()) {
    const char * names[] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
      sensor_msgs::msg::PointField field;
      field.name = names[i];
      field.offset = 4 * i;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      msg.fields.push_back(field);
    }
    msg.header.frame_id = "bench";
    msg.height = 1;
    msg.point_step = 12;
    msg.is_bigendian = false;
    msg.is_dense = true;
  }
  msg.width = static_cast<std::uint32_t>(size / 12);
  msg.row_step = static_cast<std::uint32_t>(size);
  msg.data.resize(size);
  for (std::size_t i = kHead; i < size; ++i) {
    msg.data[i] = pattern(seq, i);
  }
  std::memcpy(msg.data.data(), &seq, 8);
}

void stamp(sensor_msgs::msg::PointCloud2 & msg)
{
  const std::int64_t t = now_ns();
  std::memcpy(msg.data.data() + 8, &t, 8);
}

// ---- receiver bookkeeping (shared by both transports) ----
struct Record
{
  std::vector<double> latency_us;
  std::vector<std::uint64_t> seqs;
  std::uint64_t received{0};
  std::uint64_t corrupt{0};
  std::uint64_t out_of_order{0};
  std::uint64_t last_seq{0};
  bool any{false};

  void take(const sensor_msgs::msg::PointCloud2 & msg, std::int64_t t_recv, std::size_t size)
  {
    std::uint64_t seq = 0;
    std::int64_t t_send = 0;
    if (msg.data.size() != size) {
      ++corrupt;
      return;
    }
    std::memcpy(&seq, msg.data.data(), 8);
    std::memcpy(&t_send, msg.data.data() + 8, 8);
    bool ok = msg.fields.size() == 3 && msg.width == size / 12;
    for (std::size_t i = kHead; ok && i < size; ++i) {
      ok = msg.data[i] == pattern(seq, i);
    }
    if (!ok) {
      ++corrupt;
      return;
    }
    if (any && seq <= last_seq) {
      ++out_of_order;
      return;
    }
    any = true;
    last_seq = seq;
    ++received;
    latency_us.push_back((t_recv - t_send) * 1e-3);
    seqs.push_back(seq);
  }
};

double percentile(std::vector<double> values, double p)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t index = std::min(values.size() - 1, static_cast<std::size_t>(p * (values.size() - 1) + 0.5));
  return values[index];
}

void write_result(const Options & o, const std::map<std::string, double> & values,
                  const std::map<std::string, std::string> & text)
{
  std::ostringstream json;
  json << "{\"transport\": \"" << o.transport << "\", \"role\": \"" << o.role << "\", \"size\": " << o.size
       << ", \"rate\": " << o.rate << ", \"count\": " << o.count;
  for (const auto & [key, value] : text) {
    json << ", \"" << key << "\": \"" << value << "\"";
  }
  for (const auto & [key, value] : values) {
    json << ", \"" << key << "\": " << value;
  }
  json << "}";
  std::cout << json.str() << std::endl;
  if (!o.out.empty()) {
    std::ofstream(o.out) << json.str() << "\n";
  }
}

void write_receiver(const Options & o, const Record & r, double cpu, double wall,
                    const std::map<std::string, std::string> & text)
{
  double mean = 0.0;
  for (double v : r.latency_us) {
    mean += v;
  }
  mean = r.latency_us.empty() ? 0.0 : mean / r.latency_us.size();
  if (!o.trace.empty()) {
    std::ofstream trace(o.trace);
    for (std::size_t i = 0; i < r.seqs.size(); ++i) {
      trace << r.seqs[i] << " " << r.latency_us[i] << "\n";
    }
  }
  write_result(o, {{"received", static_cast<double>(r.received)},
                   {"corrupt", static_cast<double>(r.corrupt)},
                   {"out_of_order", static_cast<double>(r.out_of_order)},
                   {"lost", static_cast<double>(o.count - std::min<std::uint64_t>(o.count, r.received))},
                   {"latency_us_mean", mean},
                   {"latency_us_p50", percentile(r.latency_us, 0.50)},
                   {"latency_us_p99", percentile(r.latency_us, 0.99)},
                   {"latency_us_max", percentile(r.latency_us, 1.0)},
                   {"cpu_s", cpu},
                   {"wall_s", wall}},
               text);
}

// The sender's schedule: count messages at rate, each stamped just before it is
// handed to the transport.
template <typename Send>
void send_loop(const Options & o, Send send)
{
  sensor_msgs::msg::PointCloud2 msg;
  const auto period = std::chrono::nanoseconds(static_cast<std::int64_t>(1e9 / o.rate));
  auto next = std::chrono::steady_clock::now();
  const double cpu0 = cpu_seconds();
  const auto t0 = std::chrono::steady_clock::now();
  std::uint64_t failed = 0;
  for (std::uint64_t seq = 1; seq <= o.count; ++seq) {
    fill(msg, seq, o.size);
    std::this_thread::sleep_until(next);
    next += period;
    stamp(msg);
    if (!send(msg)) {
      ++failed;
    }
  }
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  write_result(o, {{"failed", static_cast<double>(failed)}, {"cpu_s", cpu_seconds() - cpu0}, {"wall_s", wall}}, {});
}

// The receiver stops at count messages or after idle seconds without one.
constexpr double kIdleSec = 3.0;

// ---- ROS 2 ----
rclcpp::QoS make_qos(const Options & o)
{
  rclcpp::QoS qos(10);
  if (o.qos == "best_effort") {
    qos.best_effort();
  } else {
    qos.reliable();
  }
  return qos;
}

int run_ros(const Options & o, int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("transport_bench_" + o.role);
  if (o.role == "send") {
    auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(kTopic, make_qos(o));
    // Start once the receiver is matched (and a second more for discovery to settle).
    while (rclcpp::ok() && pub->get_subscription_count() == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    send_loop(o, [&](sensor_msgs::msg::PointCloud2 & msg) {
      pub->publish(msg);
      return true;
    });
    std::this_thread::sleep_for(std::chrono::seconds(1));
  } else {
    Record record;
    auto last = std::chrono::steady_clock::now();
    bool started = false;
    std::chrono::steady_clock::time_point t0;
    double cpu0 = 0.0;
    auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      kTopic, make_qos(o), [&](const sensor_msgs::msg::PointCloud2 & msg) {
        const std::int64_t t = now_ns();
        if (!started) {
          started = true;
          t0 = std::chrono::steady_clock::now();
          cpu0 = cpu_seconds();
        }
        record.take(msg, t, o.size);
        last = std::chrono::steady_clock::now();
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && record.received < o.count) {
      executor.spin_once(std::chrono::milliseconds(100));
      if (started && std::chrono::duration<double>(std::chrono::steady_clock::now() - last).count() > kIdleSec) {
        break;
      }
    }
    const double wall = started ? std::chrono::duration<double>(last - t0).count() : 0.0;
    write_receiver(o, record, cpu_seconds() - cpu0, wall, {{"rmw", rmw_get_implementation_identifier()}});
  }
  rclcpp::shutdown();
  return 0;
}

// ---- Hakoniwa PDU ----
const Options * g_options = nullptr;
int g_result = 0;

int pdu_send()
{
  const Options & o = *g_options;
  std::vector<char> buffer(o.pdu_size);
  hako::pdu::msgs::sensor_msgs::PointCloud2 conv;
  std::this_thread::sleep_for(std::chrono::seconds(1));  // the receiver registers before the start
  send_loop(o, [&](sensor_msgs::msg::PointCloud2 & msg) {
    HakoCpp_PointCloud2 cpp{};
    cpp.header.stamp.sec = msg.header.stamp.sec;
    cpp.header.stamp.nanosec = msg.header.stamp.nanosec;
    cpp.header.frame_id = msg.header.frame_id;
    cpp.height = msg.height;
    cpp.width = msg.width;
    for (const auto & f : msg.fields) {
      HakoCpp_PointField field{};
      field.name = f.name;
      field.offset = f.offset;
      field.datatype = f.datatype;
      field.count = f.count;
      cpp.fields.push_back(field);
    }
    cpp.is_bigendian = msg.is_bigendian;
    cpp.point_step = msg.point_step;
    cpp.row_step = msg.row_step;
    cpp.data = msg.data;
    cpp.is_dense = msg.is_dense;
    const int n = conv.cpp2pdu(cpp, buffer.data(), static_cast<int>(buffer.size()));
    if (n <= 0) {
      return false;
    }
    return hako_asset_pdu_write(kRobot, kChannel, buffer.data(), static_cast<std::size_t>(n)) == 0;
  });
  return 0;
}

int pdu_recv()
{
  const Options & o = *g_options;
  std::vector<char> buffer(o.pdu_size);
  hako::pdu::msgs::sensor_msgs::PointCloud2 conv;
  Record record;
  bool started = false;
  auto last = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point t0;
  double cpu0 = cpu_seconds();
  const auto idle_start = std::chrono::steady_clock::now();
  while (record.received < o.count) {
    if (hako_asset_check_data_recv_event(kRobot, kChannel) == 0) {
      if (hako_asset_pdu_read(kRobot, kChannel, buffer.data(), buffer.size()) != 0) {
        g_result = 1;
        break;
      }
      HakoCpp_PointCloud2 cpp{};
      sensor_msgs::msg::PointCloud2 msg;
      if (conv.pdu2cpp(buffer.data(), cpp)) {
        msg.header.stamp.sec = cpp.header.stamp.sec;
        msg.header.stamp.nanosec = cpp.header.stamp.nanosec;
        msg.header.frame_id = cpp.header.frame_id;
        msg.height = cpp.height;
        msg.width = cpp.width;
        for (const auto & f : cpp.fields) {
          sensor_msgs::msg::PointField field;
          field.name = f.name;
          field.offset = f.offset;
          field.datatype = f.datatype;
          field.count = f.count;
          msg.fields.push_back(field);
        }
        msg.is_bigendian = cpp.is_bigendian;
        msg.point_step = cpp.point_step;
        msg.row_step = cpp.row_step;
        msg.data = std::move(cpp.data);
        msg.is_dense = cpp.is_dense;
      }
      const std::int64_t t = now_ns();
      if (!started) {
        started = true;
        t0 = std::chrono::steady_clock::now();
        cpu0 = cpu_seconds();
      }
      record.take(msg, t, o.size);
      last = std::chrono::steady_clock::now();
      continue;
    }
    const auto now = std::chrono::steady_clock::now();
    const double idle = std::chrono::duration<double>(now - (started ? last : idle_start)).count();
    if (idle > (started ? kIdleSec : 30.0)) {
      break;
    }
    if (o.poll_us > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(o.poll_us));
    }
  }
  const double wall = started ? std::chrono::duration<double>(last - t0).count() : 0.0;
  write_receiver(o, record, cpu_seconds() - cpu0, wall, {{"poll_us", std::to_string(o.poll_us)}});
  return 0;
}

int on_initialize(hako_asset_context_t *)
{
  if (g_options->role == "recv") {
    int id = -1;
    if (hako_asset_register_data_recv_event(kRobot, kChannel, nullptr, &id) != 0) {
      std::cerr << "cannot register the receive event" << std::endl;
      g_result = 1;
    }
  }
  return 0;
}

int on_manual_timing_control(hako_asset_context_t *)
{
  if (g_result == 0) {
    g_result = g_options->role == "send" ? pdu_send() : pdu_recv();
  }
  return 0;
}

int on_reset(hako_asset_context_t *) { return 0; }

int run_pdu(const Options & o)
{
  if (o.pdu_config.empty() || o.pdu_size == 0) {
    std::cerr << "--pdu-config and --pdu-size are required with --transport pdu" << std::endl;
    return 2;
  }
  g_options = &o;
  hako_asset_callbacks_t callbacks{on_initialize, nullptr, on_manual_timing_control, on_reset};
  const std::string name = "TransportBench_" + o.role;
  if (hako_asset_register(name.c_str(), o.pdu_config.c_str(), &callbacks, 1000, HAKO_ASSET_MODEL_CONTROLLER) != 0) {
    std::cerr << "cannot register the asset " << name << ": " << hako_asset_last_error() << std::endl;
    return 1;
  }
  hako_asset_start();  // returns when the simulation stops
  return g_result;
}

bool parse(int argc, char ** argv, Options & o)
{
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string key = argv[i];
    const std::string value = argv[i + 1];
    if (key == "--transport") o.transport = value;
    else if (key == "--role") o.role = value;
    else if (key == "--size") o.size = std::stoul(value);
    else if (key == "--rate") o.rate = std::stod(value);
    else if (key == "--count") o.count = std::stoull(value);
    else if (key == "--out") o.out = value;
    else if (key == "--pdu-config") o.pdu_config = value;
    else if (key == "--pdu-size") o.pdu_size = std::stoul(value);
    else if (key == "--poll-us") o.poll_us = std::stoi(value);
    else if (key == "--qos") o.qos = value;
    else if (key == "--trace") o.trace = value;
    else if (key.rfind("--ros-args", 0) == 0) break;
    else {
      std::cerr << "unknown option " << key << std::endl;
      return false;
    }
  }
  return (o.transport == "ros" || o.transport == "pdu") && (o.role == "send" || o.role == "recv") &&
         o.size >= 24 && o.rate > 0.0 && o.count > 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  Options o;
  if (!parse(argc, argv, o)) {
    std::cerr << "usage: transport_bench --transport ros|pdu --role send|recv --size <bytes >= 24> --rate <Hz> "
                 "--count <n> [--out <json>] [--pdu-config <pdudef>] [--pdu-size <bytes>] [--poll-us <us>] "
                 "[--qos reliable|best_effort] [--trace <file>]"
              << std::endl;
    return 2;
  }
  return o.transport == "ros" ? run_ros(o, argc, argv) : run_pdu(o);
}
