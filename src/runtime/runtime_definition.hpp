#pragma once

#include "runtime/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

struct RuntimeInitialBodyPoseConfig {
    std::string mjcf_freejoint;
    Vector3State position;
    std::array<double, 3> rpy_rad {0.0, 0.0, 0.0};
};

struct RuntimePersonSeatConfig {
    std::string name;
    Vector3State offset;
};

struct RuntimePersonVehicleConfig {
    std::string name;
    std::string mjcf_freejoint;
    std::vector<RuntimePersonSeatConfig> seats;
};

/** Initial world yaw of a PersonStep-controlled articulated person. */
struct RuntimePersonInitialPoseConfig {
    std::string name;
    double yaw_rad {0.0};
};

struct RuntimeActuatorConfig {
    std::string component_id;
    std::string component_type;
    std::string config_path;
    std::string pdu_robot;
    std::string pdu_name;
    std::string joint_name;
    std::string mujoco_actuator_name;
    ActuatorCommandType command_type {ActuatorCommandType::Position};
    ActuatorLimits limits {};
    double update_rate_hz {0.0};
    std::uint64_t command_timeout_usec {0};
};

struct RuntimeJointStateBinding {
    std::string joint_name;
    std::string actuator_id;
};

struct RuntimeStateOutputConfig {
    std::string component_id;
    std::string config_path;
    std::string pdu_robot;
    std::string pdu_name;
    double update_rate_hz {0.0};
    std::vector<RuntimeJointStateBinding> joints;
};

struct RuntimeMultiDofBodyBinding {
    std::string name;
    std::string state_id;
    std::string mjcf_freejoint;
};

struct RuntimeMultiDofStateOutputConfig {
    std::string component_id;
    std::string config_path;
    std::string pdu_robot;
    std::string pdu_name;
    std::string frame_id;
    double update_rate_hz {0.0};
    std::vector<RuntimeMultiDofBodyBinding> bodies;
};

struct RuntimeTrajectoryBinding {
    std::string joint_name;
    std::string actuator_id;
};

struct RuntimeTrajectoryControllerConfig {
    std::string component_id;
    std::string config_path;
    std::string pdu_robot;
    std::string pdu_name;
    double update_rate_hz {0.0};
    std::vector<RuntimeTrajectoryBinding> joints;
};

struct RuntimeManualAxisBinding {
    std::size_t axis_index {0};
    std::string joint_name;
    std::string actuator_id;
    double velocity_per_sec {0.0};
    bool invert {false};
};

struct RuntimeManualBankConfig {
    std::string id;
    std::optional<std::size_t> select_button_index;
    std::vector<RuntimeManualAxisBinding> bindings;
};

struct RuntimeManualControllerConfig {
    std::string component_id;
    std::string config_path;
    std::string pdu_robot;
    std::string pdu_name;
    std::uint64_t input_timeout_usec {0};
    double update_rate_hz {0.0};
    double deadzone {0.0};
    double expo {0.0};
    std::size_t joy_axis_count {0};
    std::size_t joy_button_count {0};
    std::size_t manual_enable_button_index {0};
    std::vector<RuntimeManualBankConfig> banks;
};

struct RuntimeAckermannGeometry {
    double wheelbase_m {0.0};
    double track_width_m {0.0};
    double wheel_radius_m {0.0};
    double max_steering_angle_rad {0.0};
    double max_wheel_angular_velocity_rad_s {0.0};
};

struct RuntimeAckermannActuatorBindings {
    std::string steering_left;
    std::string steering_right;
    std::string drive_left;
    std::string drive_right;
};

struct RuntimeAckermannControllerConfig {
    std::string component_id;
    std::string config_path;
    std::string pdu_robot;
    std::string pdu_name;
    std::uint64_t input_timeout_usec {0};
    double update_rate_hz {0.0};
    RuntimeAckermannGeometry geometry;
    RuntimeAckermannActuatorBindings actuators;
};

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
struct RuntimeMirrorContactBodyBinding {
    std::string body_id;
    std::string mjcf_freejoint;
};

struct RuntimeMirrorBodyConfig {
    std::string component_id;
    std::string config_path;
    std::string mirror_id;
    std::string pdu_robot;
    std::string pose_pdu_name;
    std::optional<std::string> velocity_pdu_name;
    MirrorVelocityFrame velocity_frame {MirrorVelocityFrame::World};
    std::string mjcf_freejoint;
    std::vector<RuntimeMirrorContactBodyBinding> contact_bodies;
};

struct RuntimeImpulseCollisionOutputConfig {
    std::string component_id;
    std::string config_path;
    std::string mirror_component_id;
    std::string mirror_id;
    std::string pdu_robot;
    std::string pdu_name;
    double restitution_coefficient {0.3};
    double relative_normal_speed_threshold_mps {0.2};
    std::uint64_t cooldown_usec {100'000};
};

/** Plant Directive controller that sets the sliding friction of named geoms. */
struct RuntimeGeomFrictionConfig {
    std::string component_id;
    std::string config_path;
    std::string pdu_robot;
    std::string pdu_name;
    /**
     * MuJoCo geoms whose sliding friction is set. Which friction wins in a
     * contact is decided by the model (geom priority), not by the Runtime.
     */
    std::vector<std::string> geoms;
};
#endif

/**
 * Resolved Runtime-local definition produced from the declarative manifest.
 *
 * This is not the source manifest document. Paths, component references,
 * message bindings, command types, limits, and controller relationships have
 * already been validated and normalized for Runtime composition.
 */
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR
/** One sensor of a `sensor` component (schemas/components/sensor-instances.schema.json). */
struct RuntimeSensorConfig {
    std::string component_id;
    std::string pdu_robot;
    std::string name;
    std::string type;
    std::string implementation;
    std::string definition_path;     ///< resolved sensor definition file
    bool measures_torque {false};    ///< joint sensor whose definition has "torque"
    bool scans_3d {false};           ///< LiDAR whose definition has more than one vertical sample
    std::string mjcf_joint;          ///< joint mount
    std::string mjcf_body;           ///< body mount
    std::array<double, 3> position_m {0.0, 0.0, 0.0};
    std::array<double, 3> orientation_deg {0.0, 0.0, 0.0};  ///< roll, pitch, yaw
    /// Range sensors: the root body whose subtree the rays never hit. Empty
    /// means the mount body (the sensor housing).
    std::string exclude_mjcf_body;
};

/**
 * Where the simulated world is on the earth (runtime_config.sensors.world).
 * The MuJoCo world axes are NWU (x north, y west, z up) from origin.
 */
struct RuntimeSensorWorld {
    bool has_origin {false};
    double latitude_deg {0.0};
    double longitude_deg {0.0};
    double altitude_m {0.0};              ///< of the world origin, above sea level
    bool has_magnetic_field {false};
    double magnetic_intensity_nT {0.0};
    double magnetic_declination_deg {0.0};  ///< east of true north is positive
    double magnetic_inclination_deg {0.0};  ///< downward is positive
    double sea_level_pressure_pa {101325.0};
    double sea_level_temperature_c {15.0};
};

/** A sensor output (schemas/components/sensor-output.schema.json). */
struct RuntimeSensorOutputConfig {
    std::string component_id;
    std::string pdu_robot;
    std::string pdu_name;
    std::string message_type;        ///< sensor_msgs/JointState or geometry_msgs/WrenchStamped
    double update_rate_hz {0.0};
    std::vector<std::string> sensors;
};
#endif

struct RuntimeDefinition {
    std::string model_path;
    std::vector<RuntimeInitialBodyPoseConfig> initial_body_poses;
    std::vector<RuntimePersonInitialPoseConfig> person_initial_poses;
    std::vector<RuntimePersonVehicleConfig> person_vehicles;
    std::vector<RuntimeActuatorConfig> actuators;
    std::vector<RuntimeStateOutputConfig> state_outputs;
    std::vector<RuntimeMultiDofStateOutputConfig> multi_dof_state_outputs;
    std::vector<RuntimeTrajectoryControllerConfig> trajectory_controllers;
    std::vector<RuntimeManualControllerConfig> manual_controllers;
    std::vector<RuntimeAckermannControllerConfig> ackermann_controllers;
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
    std::vector<RuntimeMirrorBodyConfig> mirror_bodies;
    std::vector<RuntimeImpulseCollisionOutputConfig> impulse_collision_outputs;
    std::vector<RuntimeGeomFrictionConfig> geom_frictions;
#endif
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR
    std::vector<RuntimeSensorConfig> sensors;
    std::vector<RuntimeSensorOutputConfig> sensor_outputs;
    std::uint64_t sensor_seed {0};
    RuntimeSensorWorld sensor_world;
#endif
};

} // namespace hakoniwa::robot_runtime::runtime
