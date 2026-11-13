// One test driver, several transports: a point cloud from a sender to a
// receiver as ROS 2 messages, as a Hakoniwa PDU in shared memory, or (when
// built with gz-transport) as Gazebo transport messages. Only the transport
// differs; the payload, the timing and the checks are shared.
//
//   transport_bench --transport ros|pdu|gz --role send|recv --size <bytes> --rate <Hz>
//                   --count <n> [--out <json>] [--pdu-config <pdudef>] [--pdu-size <bytes>]
//                   [--poll-us <us>] [--qos reliable|best_effort] [--depth <n>] [--trace <file>]
//                   [--type cloud|fixed]
//
// --type cloud (default): sensor_msgs/PointCloud2, as a sensor is used; the PDU
// path converts the message to the PDU and back.
// --type fixed: a fixed-size message (hakoniwa_transport_bench_msgs/Fixed*,
// size 1024, 65536, 327680, 1048576 or 4194304), the lightest form on each
// side: ROS 2 lends the message in shared memory (loaned message, zero copy
// when the RMW allows it); the PDU path writes the message's bytes as they are
// (no conversion; one copy into and one out of the shared memory).
//
// --transport gz: Gazebo's own transport (gz-transport, default settings; it
// has no zero copy). cloud is gz.msgs.PointCloudPacked (what Gazebo's lidar
// publishes), fixed is gz.msgs.Bytes; both are protobuf, serialised and parsed
// on the way.
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
#include <functional>
#include <map>
#include <optional>
#include <sstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "hakoniwa_transport_bench_msgs/msg/fixed1_k.hpp"
#include "hakoniwa_transport_bench_msgs/msg/fixed64_k.hpp"
#include "hakoniwa_transport_bench_msgs/msg/fixed320_k.hpp"
#include "hakoniwa_transport_bench_msgs/msg/fixed1_m.hpp"
#include "hakoniwa_transport_bench_msgs/msg/fixed4_m.hpp"

#ifdef HAKO_BENCH_WITH_GZ
#include <atomic>
#include <mutex>

#include <gz/msgs/bytes.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>
#endif

#include "hako_asset.h"
#include "sensor_msgs/pdu_ctype_conv_PointCloud2.hpp"

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
  std::size_t depth{10};  // ROS 2 history depth (KEEP_LAST)
  std::string trace;  // receiver: one line per message, "sequence latency_us"
  std::string type{"cloud"};
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

// The content of byte i of message seq is (seq * 31 + i) mod 256: a window of
// one table, so filling and checking are a memcpy and a memcmp.
const std::uint8_t * pattern(std::uint64_t seq, std::size_t size)
{
  static std::vector<std::uint8_t> table;
  if (table.size() < size + 256) {
    table.resize(size + 256);
    for (std::size_t i = 0; i < table.size(); ++i) {
      table[i] = static_cast<std::uint8_t>(i & 0xffu);
    }
  }
  return table.data() + ((seq * 31u) & 0xffu);
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
  std::memcpy(msg.data.data() + kHead, pattern(seq, size) + kHead, size - kHead);
  std::memcpy(msg.data.data(), &seq, 8);
}

void stamp_bytes(std::uint8_t * data)
{
  const std::int64_t t = now_ns();
  std::memcpy(data + 8, &t, 8);
}

void stamp(sensor_msgs::msg::PointCloud2 & msg) { stamp_bytes(msg.data.data()); }

void fill_bytes(std::uint8_t * data, std::uint64_t seq, std::size_t size)
{
  std::memcpy(data + kHead, pattern(seq, size) + kHead, size - kHead);
  std::memcpy(data, &seq, 8);
}

// A corrupted message, in one line: the sequence in its head, the first byte
// that differs from that sequence's pattern, and the sequence (mod 256) each part
// of the body belongs to (byte i of message seq is (seq * 31 + i) mod 256, and
// 223 * 31 = 1 mod 256), at the start, the middle and the end, read twice: a
// body that changes between the two reads is being written while it is read.
std::string describe_corrupt(const std::uint8_t * data, std::size_t size, std::uint64_t seq)
{
  const std::uint8_t * expected = pattern(seq, size);
  std::size_t first = kHead;
  while (first < size && data[first] == expected[first]) {
    ++first;
  }
  // volatile: each read goes to the (shared) memory, the second one too.
  const volatile std::uint8_t * live = data;
  auto owner = [&](std::size_t i) { return ((static_cast<unsigned>(live[i]) - i) * 223u) & 0xffu; };
  const std::size_t at[3] = {kHead, size / 2, size - 1};
  std::ostringstream line;
  line << "seq " << seq << " (mod 256 " << (seq & 0xffu) << ") first_diff " << first << " owner";
  for (std::size_t i : at) {
    line << " " << owner(i);
  }
  line << " reread";
  for (std::size_t i : at) {
    line << " " << owner(i);
  }
  return line.str();
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
  // The first corrupted messages: what they carried, to tell an overwritten
  // message from a damaged one (see describe_corrupt()).
  std::vector<std::string> corrupt_detail;

  void take(const sensor_msgs::msg::PointCloud2 & msg, std::int64_t t_recv, std::size_t size)
  {
    take_bytes(msg.data.data(), msg.data.size(), msg.fields.size() == 3 && msg.width == size / 12, t_recv, size);
  }

  void take_bytes(const std::uint8_t * data, std::size_t n, bool shape_ok, std::int64_t t_recv, std::size_t size)
  {
    std::uint64_t seq = 0;
    std::int64_t t_send = 0;
    if (n != size) {
      ++corrupt;
      return;
    }
    std::memcpy(&seq, data, 8);
    std::memcpy(&t_send, data + 8, 8);
    bool ok = shape_ok && std::memcmp(data + kHead, pattern(seq, size) + kHead, size - kHead) == 0;
    if (!ok) {
      ++corrupt;
      if (corrupt_detail.size() < 50) {
        corrupt_detail.push_back(describe_corrupt(data, size, seq));
      }
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
    if (!r.corrupt_detail.empty()) {
      std::ofstream corrupt(o.trace + ".corrupt");
      for (const auto & line : r.corrupt_detail) {
        corrupt << line << "\n";
      }
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
// prepare(seq) builds message seq before its time; send() stamps it and hands
// it over (false: failed).
template <typename Prepare, typename Send>
void schedule(const Options & o, Prepare prepare, Send send, const std::map<std::string, std::string> & text = {})
{
  const auto period = std::chrono::nanoseconds(static_cast<std::int64_t>(1e9 / o.rate));
  auto next = std::chrono::steady_clock::now();
  const double cpu0 = cpu_seconds();
  const auto t0 = std::chrono::steady_clock::now();
  std::uint64_t failed = 0;
  for (std::uint64_t seq = 1; seq <= o.count; ++seq) {
    prepare(seq);
    std::this_thread::sleep_until(next);
    next += period;
    if (!send()) {
      ++failed;
    }
  }
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  write_result(o, {{"failed", static_cast<double>(failed)}, {"cpu_s", cpu_seconds() - cpu0}, {"wall_s", wall}}, text);
}

template <typename Send>
void send_loop(const Options & o, Send send)
{
  sensor_msgs::msg::PointCloud2 msg;
  schedule(o, [&](std::uint64_t seq) { fill(msg, seq, o.size); },
           [&]() {
             stamp(msg);
             return send(msg);
           });
}

// The receiver's clock: wall and CPU time from the first message to the last,
// so the idle wait after the last message counts in neither.
struct RecvClock
{
  bool started{false};
  std::chrono::steady_clock::time_point t0;
  std::chrono::steady_clock::time_point last{std::chrono::steady_clock::now()};
  double cpu0{0.0};
  double cpu_last{0.0};

  void hit()
  {
    const auto now = std::chrono::steady_clock::now();
    const double cpu_now = cpu_seconds();
    if (!started) {
      started = true;
      t0 = now;
      cpu0 = cpu_now;
    }
    last = now;
    cpu_last = cpu_now;
  }
  double wall() const { return started ? std::chrono::duration<double>(last - t0).count() : 0.0; }
  double cpu() const { return started ? cpu_last - cpu0 : 0.0; }
  double idle() const { return std::chrono::duration<double>(std::chrono::steady_clock::now() - last).count(); }
};

// The receiver stops at count messages or after idle seconds without one.
constexpr double kIdleSec = 3.0;

// ---- ROS 2 ----
rclcpp::QoS make_qos(const Options & o)
{
  rclcpp::QoS qos(o.depth);
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
    RecvClock clock;
    auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      kTopic, make_qos(o), [&](const sensor_msgs::msg::PointCloud2 & msg) {
        const std::int64_t t = now_ns();
        record.take(msg, t, o.size);
        clock.hit();
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && record.received < o.count) {
      executor.spin_once(std::chrono::milliseconds(100));
      if (clock.started && clock.idle() > kIdleSec) {
        break;
      }
    }
    write_receiver(o, record, clock.cpu(), clock.wall(), {{"rmw", rmw_get_implementation_identifier()}});
  }
  rclcpp::shutdown();
  return 0;
}

// ---- Hakoniwa PDU ----
const Options * g_options = nullptr;
int g_result = 0;
std::function<int()> g_send;
std::function<int()> g_recv;

int pdu_send()
{
  const Options & o = *g_options;
  std::vector<char> buffer(o.pdu_size);
  std::this_thread::sleep_for(std::chrono::seconds(1));  // the receiver registers before the start
  // The ROS 2 message straight into the PDU (no Hakoniwa C++ type in between).
  send_loop(o, [&](sensor_msgs::msg::PointCloud2 & msg) {
    const int n = hako_convert_ros2pdu_into_PointCloud2(msg, buffer.data(), static_cast<int>(buffer.size()));
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
  Record record;
  RecvClock clock;
  // One destination for every message, as a receiver keeps it: no arrays are
  // allocated per message. The PDU straight into the ROS 2 message.
  sensor_msgs::msg::PointCloud2 msg;
  const auto idle_start = std::chrono::steady_clock::now();
  while (record.received < o.count) {
    if (hako_asset_check_data_recv_event(kRobot, kChannel) == 0) {
      if (hako_asset_pdu_read(kRobot, kChannel, buffer.data(), buffer.size()) != 0) {
        g_result = 1;
        break;
      }
      auto * base = static_cast<Hako_PointCloud2 *>(hako_get_base_ptr_pdu(buffer.data()));
      if (base == nullptr || hako_convert_pdu2ros_PointCloud2(*base, msg) != 0) {
        g_result = 1;
        break;
      }
      const std::int64_t t = now_ns();
      record.take(msg, t, o.size);
      clock.hit();
      continue;
    }
    const double idle = clock.started ? clock.idle()
                                      : std::chrono::duration<double>(std::chrono::steady_clock::now() - idle_start).count();
    if (idle > (clock.started ? kIdleSec : 30.0)) {
      break;
    }
    if (o.poll_us > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(o.poll_us));
    }
  }
  write_receiver(o, record, clock.cpu(), clock.wall(), {{"poll_us", std::to_string(o.poll_us)}});
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
    g_result = g_options->role == "send" ? g_send() : g_recv();
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
  if (!g_send) {
    g_send = pdu_send;
    g_recv = pdu_recv;
  }
  hako_asset_callbacks_t callbacks{on_initialize, nullptr, on_manual_timing_control, on_reset};
  const std::string name = "TransportBench_" + o.role;
  if (hako_asset_register(name.c_str(), o.pdu_config.c_str(), &callbacks, 1000, HAKO_ASSET_MODEL_CONTROLLER) != 0) {
    std::cerr << "cannot register the asset " << name << ": " << hako_asset_last_error() << std::endl;
    return 1;
  }
  hako_asset_start();  // returns when the simulation stops
  return g_result;
}

// ---- fixed-size messages ----
template <typename MsgT>
int run_fixed_ros(const Options & o, int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("transport_bench_" + o.role);
  if (o.role == "send") {
    auto pub = node->create_publisher<MsgT>(kTopic, make_qos(o));
    while (rclcpp::ok() && pub->get_subscription_count() == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::optional<rclcpp::LoanedMessage<MsgT>> loaned;
    schedule(
      o,
      [&](std::uint64_t seq) {
        loaned.emplace(pub->borrow_loaned_message());
        fill_bytes(loaned->get().data.data(), seq, o.size);
      },
      [&]() {
        stamp_bytes(loaned->get().data.data());
        pub->publish(std::move(*loaned));
        loaned.reset();
        return true;
      },
      {{"loan", pub->can_loan_messages() ? "yes" : "no"}});
    std::this_thread::sleep_for(std::chrono::seconds(1));
  } else {
    Record record;
    RecvClock clock;
    auto sub = node->create_subscription<MsgT>(kTopic, make_qos(o), [&](const MsgT & msg) {
      const std::int64_t t = now_ns();
      clock.hit();
      record.take_bytes(msg.data.data(), msg.data.size(), true, t, o.size);
    });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && record.received < o.count) {
      executor.spin_once(std::chrono::milliseconds(100));
      if (clock.started &&
          std::chrono::duration<double>(std::chrono::steady_clock::now() - clock.last).count() > kIdleSec) {
        break;
      }
    }
    write_receiver(o, record, clock.cpu(), clock.wall(),
                   {{"rmw", rmw_get_implementation_identifier()}, {"loan", sub->can_loan_messages() ? "yes" : "no"}});
  }
  rclcpp::shutdown();
  return 0;
}

template <typename MsgT>
int run_fixed_pdu(const Options & o)
{
  g_send = [&o]() {
    auto msg = std::make_unique<MsgT>();
    std::this_thread::sleep_for(std::chrono::seconds(1));  // the receiver registers before the start
    schedule(o, [&](std::uint64_t seq) { fill_bytes(msg->data.data(), seq, o.size); },
             [&]() {
               stamp_bytes(msg->data.data());
               return hako_asset_pdu_write(kRobot, kChannel, reinterpret_cast<const char *>(msg->data.data()),
                                           o.size) == 0;
             });
    return 0;
  };
  g_recv = [&o]() {
    auto msg = std::make_unique<MsgT>();
    Record record;
    RecvClock clock;
    const auto idle_start = std::chrono::steady_clock::now();
    while (record.received < o.count) {
      if (hako_asset_check_data_recv_event(kRobot, kChannel) == 0) {
        if (hako_asset_pdu_read(kRobot, kChannel, reinterpret_cast<char *>(msg->data.data()), o.size) != 0) {
          return 1;
        }
        const std::int64_t t = now_ns();
        clock.hit();
        record.take_bytes(msg->data.data(), msg->data.size(), true, t, o.size);
        continue;
      }
      const double idle = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                        (clock.started ? clock.last : idle_start)).count();
      if (idle > (clock.started ? kIdleSec : 30.0)) {
        break;
      }
      if (o.poll_us > 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(o.poll_us));
      }
    }
    write_receiver(o, record, clock.cpu(), clock.wall(), {{"poll_us", std::to_string(o.poll_us)}});
    return 0;
  };
  return run_pdu(o);
}

template <typename MsgT>
int run_fixed(const Options & o, int argc, char ** argv)
{
  return o.transport == "ros" ? run_fixed_ros<MsgT>(o, argc, argv) : run_fixed_pdu<MsgT>(o);
}

int run_fixed_size(const Options & o, int argc, char ** argv)
{
  namespace m = hakoniwa_transport_bench_msgs::msg;
  switch (o.size) {
    case 1024: return run_fixed<m::Fixed1K>(o, argc, argv);
    case 65536: return run_fixed<m::Fixed64K>(o, argc, argv);
    case 327680: return run_fixed<m::Fixed320K>(o, argc, argv);
    case 1048576: return run_fixed<m::Fixed1M>(o, argc, argv);
    case 4194304: return run_fixed<m::Fixed4M>(o, argc, argv);
    default:
      std::cerr << "--type fixed needs --size 1024, 65536, 327680, 1048576 or 4194304" << std::endl;
      return 2;
  }
}

#ifdef HAKO_BENCH_WITH_GZ
// ---- Gazebo transport ----
constexpr const char * kGzTopic = "/transport_bench/cloud";

void gz_shape(gz::msgs::PointCloudPacked & msg, std::size_t size)
{
  const char * names[] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    auto * field = msg.add_field();
    field->set_name(names[i]);
    field->set_offset(4 * i);
    field->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
    field->set_count(1);
  }
  msg.mutable_header()->add_data()->set_key("frame_id");
  msg.mutable_header()->mutable_data(0)->add_value("bench");
  msg.set_height(1);
  msg.set_width(static_cast<std::uint32_t>(size / 12));
  msg.set_point_step(12);
  msg.set_row_step(static_cast<std::uint32_t>(size));
  msg.set_is_bigendian(false);
  msg.set_is_dense(true);
}

bool gz_shape_ok(const gz::msgs::PointCloudPacked & msg, std::size_t size)
{
  return msg.field_size() == 3 && msg.width() == size / 12;
}
bool gz_shape_ok(const gz::msgs::Bytes &, std::size_t) { return true; }
void gz_shape(gz::msgs::Bytes &, std::size_t) {}

// MsgT has a bytes field "data": PointCloudPacked or Bytes.
template <typename MsgT>
int run_gz(const Options & o)
{
  gz::transport::Node node;
  if (o.role == "send") {
    auto pub = node.Advertise<MsgT>(kGzTopic);
    if (!pub) {
      std::cerr << "gz: advertise failed" << std::endl;
      return 1;
    }
    while (!pub.HasConnections()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    MsgT msg;
    gz_shape(msg, o.size);
    msg.mutable_data()->resize(o.size);
    auto data = [&]() { return reinterpret_cast<std::uint8_t *>(msg.mutable_data()->data()); };
    schedule(o, [&](std::uint64_t seq) { fill_bytes(data(), seq, o.size); },
             [&]() {
               stamp_bytes(data());
               return pub.Publish(msg);
             });
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return 0;
  }
  // The callback runs on gz-transport's thread; the main thread only waits.
  std::mutex lock;
  Record record;
  RecvClock clock;
  std::function<void(const MsgT &)> callback = [&](const MsgT & msg) {
    const std::int64_t t = now_ns();
    std::lock_guard<std::mutex> guard(lock);
    record.take_bytes(reinterpret_cast<const std::uint8_t *>(msg.data().data()), msg.data().size(),
                      gz_shape_ok(msg, o.size), t, o.size);
    clock.hit();
  };
  if (!node.Subscribe(kGzTopic, callback)) {
    std::cerr << "gz: subscribe failed" << std::endl;
    return 1;
  }
  const auto idle_start = std::chrono::steady_clock::now();
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::lock_guard<std::mutex> guard(lock);
    if (record.received >= o.count) {
      break;
    }
    const double idle = clock.started ? clock.idle()
                                      : std::chrono::duration<double>(std::chrono::steady_clock::now() - idle_start).count();
    if (idle > (clock.started ? kIdleSec : 30.0)) {
      break;
    }
  }
  node.Unsubscribe(kGzTopic);
  std::lock_guard<std::mutex> guard(lock);
  write_receiver(o, record, clock.cpu(), clock.wall(), {{"gz_transport", "13"}});
  return 0;
}
#endif

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
    else if (key == "--depth") o.depth = std::stoul(value);
    else if (key == "--trace") o.trace = value;
    else if (key == "--type") o.type = value;
    else if (key.rfind("--ros-args", 0) == 0) break;
    else {
      std::cerr << "unknown option " << key << std::endl;
      return false;
    }
  }
  return (o.transport == "ros" || o.transport == "pdu" || o.transport == "gz") && (o.role == "send" || o.role == "recv") &&
         (o.type == "cloud" || o.type == "fixed") && o.size >= 24 && o.rate > 0.0 && o.count > 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  Options o;
  if (!parse(argc, argv, o)) {
    std::cerr << "usage: transport_bench --transport ros|pdu|gz --role send|recv --size <bytes >= 24> --rate <Hz> "
                 "--count <n> [--out <json>] [--pdu-config <pdudef>] [--pdu-size <bytes>] [--poll-us <us>] "
                 "[--qos reliable|best_effort] [--depth <n>] [--trace <file>] [--type cloud|fixed]"
              << std::endl;
    return 2;
  }
  if (o.transport == "gz") {
#ifdef HAKO_BENCH_WITH_GZ
    return o.type == "fixed" ? run_gz<gz::msgs::Bytes>(o) : run_gz<gz::msgs::PointCloudPacked>(o);
#else
    std::cerr << "built without gz-transport" << std::endl;
    return 2;
#endif
  }
  if (o.type == "fixed") {
    return run_fixed_size(o, argc, argv);
  }
  return o.transport == "ros" ? run_ros(o, argc, argv) : run_pdu(o);
}
