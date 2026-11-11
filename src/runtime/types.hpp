#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <memory>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

/** Physical command accepted by an actuator Plant. */
enum class ActuatorCommandType {
    Position,
    Velocity,
    Effort,
};

/**
 * Runtime-neutral actuator command.
 *
 * Position uses radians, velocity uses radians per second, and effort uses
 * newton-metres. `expires_at_usec` is expressed in simulation time and is an
 * exclusive boundary: a command is expired when step time is greater than or
 * equal to it. A missing value means that the command does not expire.
 */
struct ActuatorCommand {
    std::string actuator_id;
    ActuatorCommandType type {ActuatorCommandType::Position};
    double value {0.0};
    std::string source_id;
    std::uint64_t created_at_usec {0};
    std::optional<std::uint64_t> expires_at_usec;
};

/**
 * Immutable per-step context shared by Runtime components.
 *
 * Runtime does not maintain an independent simulation clock. The timing fields
 * are derived from the physical Plant: `simulation_time_usec` is the Plant time
 * at the beginning of the step, `delta_time_usec` is the Plant integration step,
 * and `step_index` is derived from those values for diagnostics/tests.
 *
 * Hakoniwa Asset timing is configured from the same Plant delta at startup, so
 * Asset-local Hakoniwa time and physical backend time advance on the same grid.
 * Wall-clock pacing is a separate Host concern.
 *
 * `previous_selected_control_id` and `previous_selected_commands` are feedback
 * from the previous completed arbitration step. ActuatorRuntime attaches that
 * feedback when it builds the context for the current Plant state.
 */
struct RuntimeStepContext {
    std::uint64_t simulation_time_usec {0};
    std::uint64_t delta_time_usec {0};
    std::uint64_t step_index {0};
    std::optional<std::string> previous_selected_control_id;
    std::vector<ActuatorCommand> previous_selected_commands;

    [[nodiscard]] constexpr bool is_valid() const noexcept
    {
        return delta_time_usec > 0;
    }
};

/** Scalar actuator state sampled at `simulation_time_usec`. */
struct ActuatorState {
    std::string actuator_id;
    double position_rad {0.0};
    double velocity_rad_per_sec {0.0};
    double effort_newton_metre {0.0};
    std::uint64_t simulation_time_usec {0};
};

struct Vector3State {
    double x {0.0};
    double y {0.0};
    double z {0.0};
};

struct QuaternionState {
    double x {0.0};
    double y {0.0};
    double z {0.0};
    double w {1.0};
};

#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
/** One deterministic person contact edge, expressed in City World ENU. */
struct PersonContactEvent {
    std::uint64_t id {0};
    double time_sec {0.0};
    std::string self_name;
    std::string other_name;
    std::string other_kind;
    Vector3State position;
    double relative_speed_mps {0.0};
    bool started {false};
};
#endif

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
struct EulerState {
    double roll {0.0};
    double pitch {0.0};
    double yaw {0.0};
};

enum class MirrorVelocityFrame {
    World,
    Body,
};

/** Backend-independent state command for one externally owned Mirror body. */
struct MirrorBodyCommand {
    std::string mirror_id;
    Vector3State position;
    EulerState orientation;
    Vector3State linear_velocity;
    Vector3State angular_velocity;
    std::uint64_t created_at_usec {0};
};

/** One post-physics contact between a Mirror and a local physical body. */
struct MirrorContactState {
    std::string mirror_id;
    std::string local_body_id;
    /** MuJoCo-compatible signed distance; the smallest value is deepest. */
    double distance_m {0.0};
    double relative_normal_speed_mps {0.0};
    Vector3State self_contact_vector;
    Vector3State normal;
    Vector3State target_contact_vector;
    Vector3State target_velocity;
    Vector3State target_angular_velocity;
    EulerState target_euler;
    Vector3State target_inertia;
    double target_mass {0.0};
};
#endif

struct BodyState {
    std::string body_id;
    Vector3State position;
    QuaternionState orientation;
    Vector3State linear_velocity;
    Vector3State angular_velocity;
    std::uint64_t simulation_time_usec {0};
};

/** Physical-state snapshot observed at the Plant's `sample_time_usec`. */
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR
/**
 * Latest measured value of one sensor, held between its samples.
 * Joint sensors: position, velocity and, when configured, the joint torque.
 * Force-torque sensors: the wrench. IMUs: angular velocity and specific force.
 */
/** One returned LiDAR point, sensor axes at the time it was measured. */
struct LidarPointReading {
    float x {0.0F};
    float y {0.0F};
    float z {0.0F};
    float intensity {0.0F};
    std::uint16_t ring {0};          ///< vertical row, 0 = lowest
    float time_offset {0.0F};        ///< since the start of the scan [s]
};

/**
 * One complete LiDAR scan (2D or 3D), shaped like sensor_msgs/LaserScan plus
 * PointCloud2. Grids are row-major, one row per vertical sample (lowest
 * first); a 2D LiDAR has vertical_count 1.
 */
struct LidarScanReading {
    double angle_min {0.0};          ///< [rad], 0 = forward, positive to the left
    double angle_max {0.0};
    double angle_increment {0.0};
    double vertical_angle_min {0.0}; ///< [rad] from the sensor xy plane, positive up
    double vertical_angle_max {0.0};
    double vertical_angle_increment {0.0};
    std::uint32_t horizontal_count {0};
    std::uint32_t vertical_count {0};
    double time_increment {0.0};     ///< between two rays [s]
    double scan_time {0.0};          ///< between two scans [s]
    double range_min {0.0};          ///< [m]
    double range_max {0.0};
    std::vector<float> ranges;       ///< [m], REP 117 (+inf/-inf outside the range)
    std::vector<float> intensities;
    std::vector<LidarPointReading> points;  ///< returned points (finite ranges)
};

/** One measured range, shaped like sensor_msgs/Range. */
struct RangeReading {
    std::uint8_t radiation_type {2};  ///< 0 ultrasound, 1 infrared, 2 laser
    double field_of_view {0.0};       ///< full cone angle [rad]
    double min_range {0.0};           ///< [m]
    double max_range {0.0};
    double range {0.0};               ///< [m], REP 117 outside the range
    double variance {0.0};            ///< [m^2], 0 when unknown
};

struct SensorReading {
    std::string sensor_name;
    std::string mount_name;          ///< MJCF joint or body the sensor is mounted on
    bool has_sample {false};         ///< false until the first sample
    bool updated {false};            ///< a new sample was taken in this step
    std::uint64_t sample_time_usec {0};
    double position {0.0};
    double velocity {0.0};
    bool has_effort {false};         ///< the sensor measures the joint torque
    double effort {0.0};             ///< measured joint torque [N m]
    /// Force-torque sensors: the measured wrench in the sensor axes, torque
    /// about the sensor origin (what the tool side exerts on the robot side).
    bool has_wrench {false};
    std::array<double, 3> force {0.0, 0.0, 0.0};   ///< [N]
    std::array<double, 3> torque {0.0, 0.0, 0.0};  ///< [N m]
    /// IMUs: the latest gyroscope and accelerometer values, sensor axes.
    bool has_imu {false};
    std::array<double, 3> angular_velocity {0.0, 0.0, 0.0};     ///< [rad/s]
    std::array<double, 3> linear_acceleration {0.0, 0.0, 0.0};  ///< specific force [m/s^2]
    /// LiDARs: the latest complete scan, shared and never changed (a new
    /// scan replaces it), so copying a reading every step stays cheap.
    std::shared_ptr<const LidarScanReading> scan;
    /// Rangefinders: the latest measured range.
    bool has_range {false};
    RangeReading range;
};
#endif

struct RobotState {
    std::vector<ActuatorState> actuators;
    std::uint64_t sample_time_usec {0};
    std::vector<BodyState> bodies;
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
    std::vector<MirrorContactState> mirror_contacts;
#endif
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR
    /** Filled by the sensor stage after the physics step. */
    std::vector<SensorReading> sensor_readings;
#endif
};

/** Inclusive scalar command limits in the unit selected by command type. */
struct ActuatorLimits {
    double minimum {0.0};
    double maximum {0.0};

    [[nodiscard]] constexpr bool is_valid() const noexcept
    {
        return minimum <= maximum;
    }
};

enum class ComponentState {
    Ready,
    WaitingForInput,
    Degraded,
    Error,
    Stopped,
};

/** Runtime-visible component health; `detail` is diagnostic, not control data. */
struct ComponentStatus {
    std::string component_id;
    ComponentState state {ComponentState::Ready};
    std::string detail;

    [[nodiscard]] constexpr bool can_produce_commands() const noexcept
    {
        return state == ComponentState::Ready || state == ComponentState::Degraded;
    }
};

/** Metadata common to every controller-specific input payload. */
struct ControllerInputMetadata {
    std::string source_id;
    std::uint64_t sequence {0};
    std::uint64_t created_at_usec {0};
    std::optional<std::uint64_t> expires_at_usec;
};

struct ControllerOutput {
    std::string controller_id;
    ComponentStatus status;
    std::vector<ActuatorCommand> commands;
};

struct ArbitrationResult {
    std::vector<ActuatorCommand> selected_commands;
    std::vector<std::string> rejected_source_ids;
    std::vector<std::string> conflicting_actuator_ids;
    /** Logical Controller/control-group selected for this step, if any. */
    std::optional<std::string> selected_control_id;
};

} // namespace hakoniwa::robot_runtime::runtime
