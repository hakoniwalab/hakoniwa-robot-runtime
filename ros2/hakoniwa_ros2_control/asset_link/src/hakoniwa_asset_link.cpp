#include "hakoniwa_ros2_control/hakoniwa_asset_link.hpp"

#include <cerrno>
#include <iostream>
#include <span>
#include <utility>

// The generated C++ converter calls the string-array helpers by their *_cpp_*
// names; the primitive header provides them as *_ros_* (as in hakoniwa-mujoco-robots).
#define hako_convert_pdu2cpp_array_string_varray hako_convert_pdu2ros_array_string_varray
#define hako_convert_cpp2pdu_array_string_varray hako_convert_ros2pdu_array_string_varray
#include "sensor_msgs/pdu_cpptype_conv_JointState.hpp"
#include "std_msgs/pdu_cpptype_conv_Float64.hpp"
#undef hako_convert_pdu2cpp_array_string_varray
#undef hako_convert_cpp2pdu_array_string_varray

#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"

namespace hakoniwa_ros2_control
{

struct HakoniwaAssetLink::Pdu
{
  using Command = hakoniwa::pdu::TypedEndpoint<HakoCpp_Float64, hako::pdu::msgs::std_msgs::Float64>;

  Pdu(const std::string & robot, const std::string & joint_states, const std::vector<std::string> & joints)
  : endpoint("hakoniwa_system_endpoint", HAKO_PDU_ENDPOINT_DIRECTION_INOUT),
    joint_states(endpoint, hakoniwa::pdu::PduKey{robot, joint_states})
  {
    for (const auto & joint : joints) {
      commands.push_back(std::make_unique<Command>(endpoint, hakoniwa::pdu::PduKey{robot, "actuator/" + joint + "/command"}));
    }
  }
  hakoniwa::pdu::Endpoint endpoint;
  hakoniwa::pdu::TypedEndpoint<HakoCpp_JointState, hako::pdu::msgs::sensor_msgs::JointState> joint_states;
  std::vector<std::unique_ptr<Command>> commands;  // in the order of Config::command_joints
  bool started{false};
};

namespace
{
// How often the asset thread looks at the world time while it waits for it.
constexpr auto kWorldTimePoll = std::chrono::microseconds(200);
}  // namespace

std::atomic<HakoniwaAssetLink *> HakoniwaAssetLink::instance_{nullptr};

HakoniwaAssetLink::HakoniwaAssetLink(Config config) : config_(std::move(config)) {}

HakoniwaAssetLink::~HakoniwaAssetLink()
{
  stop();
  if (pdu_ && pdu_->started) {
    (void)pdu_->endpoint.stop();
  }
  if (pdu_) {
    (void)pdu_->endpoint.close();
  }
  HakoniwaAssetLink * self = this;
  instance_.compare_exchange_strong(self, nullptr);
}

bool HakoniwaAssetLink::register_asset()
{
  if (registered_) {
    return true;
  }
  HakoniwaAssetLink * none = nullptr;
  if (!instance_.compare_exchange_strong(none, this)) {
    std::cerr << "[HakoniwaAssetLink] one Hakoniwa asset per process: already registered" << std::endl;
    return false;
  }
  auto give_up = [this]() {
    pdu_.reset();
    HakoniwaAssetLink * self = this;
    instance_.compare_exchange_strong(self, nullptr);
    return false;
  };
  // The same order as the Robot Runtime: open the Endpoint, register the
  // asset, start the Endpoint; post_start comes after START (on_initialize).
  pdu_ = std::make_unique<Pdu>(config_.robot_name, config_.joint_states_pdu, config_.command_joints);
  writing_.assign(config_.command_joints.size(), 0.0);
  pending_.assign(config_.command_joints.size(), false);
  if (pdu_->endpoint.open(config_.endpoint_config_path, config_.asset_name.c_str()) != HAKO_PDU_ERR_OK) {
    std::cerr << "[HakoniwaAssetLink] cannot open the PDU Endpoint " << config_.endpoint_config_path << std::endl;
    return give_up();
  }
  callbacks_ = {};
  callbacks_.on_initialize = &HakoniwaAssetLink::on_initialize;
  callbacks_.on_simulation_step = nullptr;
  callbacks_.on_manual_timing_control = &HakoniwaAssetLink::on_manual_timing_control;
  callbacks_.on_reset = &HakoniwaAssetLink::on_reset;
  // The core accepts a new asset only while the simulation is stopped.
  const int result = hako_asset_register(config_.asset_name.c_str(), config_.pdu_definition_path.c_str(),
                                         &callbacks_, static_cast<hako_time_t>(config_.delta_usec),
                                         HAKO_ASSET_MODEL_CONTROLLER);
  if (result != 0) {
    std::cerr << "[HakoniwaAssetLink] hako_asset_register(" << config_.asset_name << ") failed: " << result
              << " (is the simulation stopped, and the name unique?)" << std::endl;
    return give_up();
  }
  if (pdu_->endpoint.start() != HAKO_PDU_ERR_OK) {
    std::cerr << "[HakoniwaAssetLink] cannot start the PDU Endpoint" << std::endl;
    return give_up();
  }
  // Sensor PDU types and sizes from the PDU definition.
  sensor_types_.clear();
  sensor_sizes_.clear();
  for (const auto & sensor : config_.sensors) {
    hakoniwa::pdu::PduDef def;
    const auto definition = pdu_->endpoint.get_pdu_definition();
    if (!definition || !definition->resolve(config_.robot_name, sensor.name, def) || sensor.rate_hz <= 0.0) {
      std::cerr << "[HakoniwaAssetLink] unknown sensor PDU " << config_.robot_name << "/" << sensor.name
                << " (or rate <= 0)" << std::endl;
      return give_up();
    }
    sensor_types_.push_back(def.type);
    sensor_sizes_.push_back(def.pdu_size);
  }
  sensor_next_usec_.assign(config_.sensors.size(), 0);
  sensor_taken_.assign(config_.sensors.size(), 0);
  sensor_latest_.assign(config_.sensors.size(), SensorSample{});
  pdu_->started = true;
  registered_ = true;
  return true;
}

bool HakoniwaAssetLink::start()
{
  if (!registered_ || thread_.joinable()) {
    return false;
  }
  stop_requested_ = false;
  finished_ = false;
  // hako_asset_start_no_wait() waits for START and then runs
  // on_manual_timing_control() on the calling thread until the end: this thread.
  thread_ = std::thread([this]() {
    const int result = hako_asset_start_no_wait(&HakoniwaAssetLink::is_force_stop);
    running_ = false;
    // EINTR: the simulation stopped or was reset, or stop() was called.
    result_ = (result == EINTR) ? 0 : result;
    finished_ = true;
  });
  return true;
}

void HakoniwaAssetLink::stop()
{
  stop_requested_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool HakoniwaAssetLink::latest_state(JointStateSample & out) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_.sequence == 0) {
    return false;
  }
  out = state_;
  return true;
}

void HakoniwaAssetLink::put_command(const std::vector<double> & values)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (command_sequence_ != writing_sequence_) {
    ++counts_.replaced;  // the previous command was never taken by the asset thread
  }
  command_ = values;
  ++command_sequence_;
  ++counts_.put;
}

HakoniwaAssetLink::CommandCounts HakoniwaAssetLink::command_counts() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return counts_;
}

int HakoniwaAssetLink::on_initialize(hako_asset_context_t *)
{
  auto * self = instance_.load();
  if (self == nullptr || !self->pdu_) {
    return -1;
  }
  // The PDU channels exist once the simulation has started.
  if (self->pdu_->endpoint.post_start() != HAKO_PDU_ERR_OK) {
    std::cerr << "[HakoniwaAssetLink] PDU Endpoint post_start failed" << std::endl;
    return -1;
  }
  return 0;
}

int HakoniwaAssetLink::on_manual_timing_control(hako_asset_context_t *)
{
  auto * self = instance_.load();
  return self == nullptr ? -1 : self->run_steps();
}

int HakoniwaAssetLink::on_reset(hako_asset_context_t *)
{
  return 0;
}

int HakoniwaAssetLink::is_force_stop()
{
  auto * self = instance_.load();
  return (self == nullptr || self->stop_requested_.load()) ? 1 : 0;
}

int HakoniwaAssetLink::run_steps()
{
  running_ = true;
  using Clock = std::chrono::steady_clock;
  const auto wall_anchor = Clock::now();
  // hako_asset_current_time(): this asset's time; hako_asset_simulation_time(): the world time.
  const auto time_anchor_usec = static_cast<std::uint64_t>(hako_asset_current_time());
  while (!stop_requested_) {
    if (config_.realtime_sync) {
      // The wall time at which this asset's time has passed since the anchor.
      const auto target = wall_anchor + std::chrono::microseconds(
        static_cast<std::uint64_t>(hako_asset_current_time()) - time_anchor_usec);
      std::this_thread::sleep_until(target);
    }
    // The core lets this asset step once its next time is within the world
    // time, and spins until then: wait for that here, sleeping instead. Not
    // before the first step: at the start the world time stays at 0 until
    // every asset has reported its PDUs written, which this asset does inside
    // hako_asset_usleep_no_wait().
    const auto next_usec = static_cast<std::uint64_t>(hako_asset_current_time()) + config_.delta_usec;
    while (steps_.load() > 0 && !stop_requested_ &&
           static_cast<std::uint64_t>(hako_asset_simulation_time()) < next_usec &&
           hako_asset_simulation_state() == HAKO_ASSET_SIM_STATE_RUNNING) {
      std::this_thread::sleep_for(kWorldTimePoll);
    }
    // Advances this asset's time by one step.
    const int result = hako_asset_usleep_no_wait(static_cast<hako_time_t>(config_.delta_usec),
                                                 &HakoniwaAssetLink::is_force_stop);
    if (result == EINTR) {
      return 0;  // the simulation stopped or was reset, or stop() was called
    }
    if (result != 0) {
      std::cerr << "[HakoniwaAssetLink] hako_asset_usleep_no_wait failed: " << result << std::endl;
      return result;
    }
    asset_time_usec_ = static_cast<std::uint64_t>(hako_asset_current_time());
    world_time_usec_ = static_cast<std::uint64_t>(hako_asset_simulation_time());
    ++steps_;
    read_joint_states();
    write_command();
    read_sensors();
  }
  return 0;
}

void HakoniwaAssetLink::read_joint_states()
{
  // Read without a receive event: the Robot Runtime writes every 2 ms, and a
  // receive event here would refuse its next write until this asset steps.
  HakoCpp_JointState pdu{};
  if (pdu_->joint_states.recv(pdu) != HAKO_PDU_ERR_OK) {
    ++state_read_errors_;
    return;
  }
  const double stamp = pdu.header.stamp.sec + pdu.header.stamp.nanosec * 1e-9;
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_.sequence != 0 && stamp == state_.stamp_sec) {
    return;  // the same PDU as before
  }
  state_.name = std::move(pdu.name);
  state_.position.assign(pdu.position.begin(), pdu.position.end());
  state_.velocity.assign(pdu.velocity.begin(), pdu.velocity.end());
  state_.effort.assign(pdu.effort.begin(), pdu.effort.end());
  state_.stamp_sec = stamp;
  state_.asset_time_usec = asset_time_usec_.load();
  state_.received = std::chrono::steady_clock::now();
  ++state_.sequence;
}

std::string HakoniwaAssetLink::sensor_type(std::size_t i) const
{
  return i < sensor_types_.size() ? sensor_types_[i] : std::string();
}

bool HakoniwaAssetLink::take_sensor(std::size_t i, SensorSample & out)
{
  std::lock_guard<std::mutex> lock(sensor_mutex_);
  if (i >= sensor_latest_.size() || sensor_latest_[i].sequence == sensor_taken_[i]) {
    return false;
  }
  sensor_taken_[i] = sensor_latest_[i].sequence;
  out.data.swap(sensor_latest_[i].data);
  out.asset_time_usec = sensor_latest_[i].asset_time_usec;
  out.sequence = sensor_latest_[i].sequence;
  return true;
}

void HakoniwaAssetLink::read_sensors()
{
  // Each sensor at its rate in this asset's time: a raw copy only, so that the
  // asset thread spends little time here (the reader decodes it).
  const std::uint64_t now = asset_time_usec_.load();
  for (std::size_t i = 0; i < config_.sensors.size(); ++i) {
    if (now < sensor_next_usec_[i]) {
      continue;
    }
    const auto period = static_cast<std::uint64_t>(1e6 / config_.sensors[i].rate_hz);
    sensor_next_usec_[i] = now + period;
    sensor_buffer_.resize(sensor_sizes_[i]);
    std::size_t received = 0;
    if (pdu_->endpoint.recv(hakoniwa::pdu::PduKey{config_.robot_name, config_.sensors[i].name},
                            std::span<std::byte>(sensor_buffer_.data(), sensor_buffer_.size()),
                            received) != HAKO_PDU_ERR_OK || received == 0) {
      continue;
    }
    sensor_buffer_.resize(received);
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    sensor_latest_[i].data.swap(sensor_buffer_);
    sensor_latest_[i].asset_time_usec = now;
    ++sensor_latest_[i].sequence;
  }
}

void HakoniwaAssetLink::write_command()
{
  if (pdu_->commands.empty()) {
    return;
  }
  {
    // Take a newer command, if any: it replaces the one being written.
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (command_sequence_ != writing_sequence_ && command_.size() == writing_.size()) {
      for (bool waiting : pending_) {
        if (waiting) {
          ++counts_.replaced;
          break;
        }
      }
      writing_ = command_;
      pending_.assign(writing_.size(), true);
      writing_sequence_ = command_sequence_;
    }
  }
  // Write each joint still waiting. A refused write (the Robot Runtime has
  // not taken the last value yet) stays waiting for the next step.
  std::uint64_t accepted = 0;
  std::uint64_t refused = 0;
  bool all_written = true;
  bool any_written = false;
  for (std::size_t i = 0; i < writing_.size(); ++i) {
    if (!pending_[i]) {
      continue;
    }
    HakoCpp_Float64 value{};
    value.data = writing_[i];
    if (pdu_->commands[i]->send(value) == HAKO_PDU_ERR_OK) {
      pending_[i] = false;
      ++accepted;
      any_written = true;
    } else {
      ++refused;
      all_written = false;
    }
  }
  if (accepted == 0 && refused == 0) {
    return;  // nothing to write
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  counts_.pdu_writes += accepted;
  counts_.pdu_refused += refused;
  if (all_written && any_written) {
    ++counts_.written;
  }
}

}  // namespace hakoniwa_ros2_control
