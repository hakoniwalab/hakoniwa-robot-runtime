#include "adapters/physics/mujoco/mujoco_sensor_stage.hpp"

#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace hakoniwa::robot_runtime::adapters::mujoco {

namespace sa = ::hakoniwa::sensor_adapter;

struct MujocoSensorStage::Sensor {
    std::string name;
    bool force_torque {false};
    // Joint sensors
    int joint_id {-1};
    int qpos_address {-1};
    int dof_address {-1};
    bool measures_torque {false};
    std::unique_ptr<sa::IJointSensorAdapter> joint_adapter;
    // Joint and force-torque sensors: the body whose cfrc_int is read
    int body_id {-1};
    int root_id {-1};
    // Force-torque sensors: the mount relative to the body frame
    mjtNum mount_position[3] {0.0, 0.0, 0.0};
    mjtNum mount_rotation[9] {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    std::unique_ptr<sa::IForceTorqueSensorAdapter> force_torque_adapter;
    // IMUs (mounted like force-torque sensors)
    bool imu {false};
    bool gyro_sampled {false};
    bool accel_sampled {false};
    std::unique_ptr<sa::IImuSensorAdapter> imu_adapter;
    // LiDARs and rangefinders (mounted like IMUs); the adapter casts
    // through ray_caster
    bool lidar {false};
    bool rangefinder {false};
    std::shared_ptr<sa::IRayCaster> ray_caster;
    std::unique_ptr<sa::ILidarSensorAdapter> lidar_adapter;
    std::unique_ptr<sa::IRangefinderSensorAdapter> rangefinder_adapter;
    // Magnetometers, barometers and GPS receivers (mounted like IMUs)
    bool mag {false};
    bool baro {false};
    bool gps {false};
    std::unique_ptr<sa::IMagSensorAdapter> mag_adapter;
    std::unique_ptr<sa::IBaroSensorAdapter> baro_adapter;
    std::unique_ptr<sa::IGpsSensorAdapter> gps_adapter;

    runtime::SensorReading reading;
};

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

// Rotation matrix (row-major, body frame <- sensor frame) of roll, pitch, yaw
// in degrees: R = Rz(yaw) * Ry(pitch) * Rx(roll), the usual ZYX convention.
void rotation_from_rpy_deg(const std::array<double, 3>& rpy_deg, mjtNum rotation[9])
{
    const double r = rpy_deg[0] * kDegToRad;
    const double p = rpy_deg[1] * kDegToRad;
    const double y = rpy_deg[2] * kDegToRad;
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    rotation[0] = cy * cp; rotation[1] = cy * sp * sr - sy * cr; rotation[2] = cy * sp * cr + sy * sr;
    rotation[3] = sy * cp; rotation[4] = sy * sp * sr + cy * cr; rotation[5] = sy * sp * cr - cy * sr;
    rotation[6] = -sp;     rotation[7] = cp * sr;                rotation[8] = cp * cr;
}

// A local NWU position [m] from the world origin to WGS-84 latitude,
// longitude [deg] and height [m] above the ellipsoid (through ECEF).
std::array<double, 3> nwu_to_geodetic(const runtime::RuntimeSensorWorld& world, const double nwu[3])
{
    constexpr double a = 6378137.0;
    constexpr double f = 1.0 / 298.257223563;
    constexpr double e2 = f * (2.0 - f);
    const double lat0 = world.latitude_deg * kDegToRad;
    const double lon0 = world.longitude_deg * kDegToRad;
    const double h0 = world.altitude_m;
    const double n0 = a / std::sqrt(1.0 - e2 * std::sin(lat0) * std::sin(lat0));
    const double x0 = (n0 + h0) * std::cos(lat0) * std::cos(lon0);
    const double y0 = (n0 + h0) * std::cos(lat0) * std::sin(lon0);
    const double z0 = (n0 * (1.0 - e2) + h0) * std::sin(lat0);
    // NWU to ENU, then ENU to ECEF at the origin.
    const double east = -nwu[1];
    const double north = nwu[0];
    const double up = nwu[2];
    const double sl = std::sin(lat0), cl = std::cos(lat0);
    const double so = std::sin(lon0), co = std::cos(lon0);
    const double x = x0 - so * east - sl * co * north + cl * co * up;
    const double y = y0 + co * east - sl * so * north + cl * so * up;
    const double z = z0 + cl * north + sl * up;
    // ECEF to geodetic (iterative; converges well below a millimetre).
    const double lon = std::atan2(y, x);
    const double p = std::hypot(x, y);
    double lat = std::atan2(z, p * (1.0 - e2));
    double h = 0.0;
    for (int i = 0; i < 10; ++i) {
        const double n = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
        h = p / std::cos(lat) - n;
        lat = std::atan2(z, p * (1.0 - e2 * n / (n + h)));
    }
    return {lat / kDegToRad, lon / kDegToRad, h};
}

} // namespace

MujocoSensorStage::MujocoSensorStage(
    const mjModel* model,
    mjData* data,
    const std::vector<runtime::RuntimeSensorConfig>& sensors,
    const std::uint64_t seed,
    const std::uint64_t physics_step_usec,
    const runtime::sensor::SensorImplementationRegistry& registry,
    const RayCasterFactory& ray_caster_factory,
    const runtime::RuntimeSensorWorld& world)
    : model_(model)
    , data_(data)
    , physics_step_usec_(physics_step_usec)
    , world_(world)
{
    if (model_ == nullptr || data_ == nullptr) {
        throw std::invalid_argument("sensor stage requires a MuJoCo model and data");
    }
    if (physics_step_usec == 0) {
        throw std::invalid_argument("sensor stage requires a positive physics step");
    }
    for (const auto& config : sensors) {
        const std::string owner = "sensor " + config.name;
        const auto* implementation = registry.find(config.implementation);
        if (implementation == nullptr) {
            throw std::invalid_argument(owner + ": implementation " + config.implementation
                + " is not linked");
        }
        auto sensor = std::make_unique<Sensor>();
        sensor->name = config.name;
        sensor->reading.sensor_name = config.name;
        std::string error;

        if (config.type == "joint") {
            const int joint_id = mj_name2id(model_, mjOBJ_JOINT, config.mjcf_joint.c_str());
            if (joint_id < 0) {
                throw std::invalid_argument(owner + ": MJCF joint not found: " + config.mjcf_joint);
            }
            const int joint_type = model_->jnt_type[joint_id];
            if (joint_type != mjJNT_HINGE && joint_type != mjJNT_SLIDE) {
                throw std::invalid_argument(owner + ": MJCF joint " + config.mjcf_joint
                    + " is not a hinge or slide joint");
            }
            if (implementation->create_joint == nullptr) {
                throw std::invalid_argument(owner + ": implementation " + config.implementation
                    + " has no joint sensor adapter");
            }
            sa::JointSensorAdapterSetup setup;
            setup.config_path = config.definition_path;
            setup.vehicle_name = config.pdu_robot;
            setup.sensor_name = config.name;
            setup.physics_step_ns = physics_step_usec * 1000;
            setup.has_random_seed = true;
            setup.random_seed = seed;
            sensor->joint_adapter = implementation->create_joint(setup, error);
            if (sensor->joint_adapter == nullptr) {
                throw std::invalid_argument(owner + ": cannot create the "
                    + config.implementation + " joint sensor adapter: " + error);
            }
            sensor->joint_id = joint_id;
            sensor->body_id = model_->jnt_bodyid[joint_id];
            sensor->root_id = model_->body_rootid[sensor->body_id];
            sensor->measures_torque = config.measures_torque;
            needs_dynamics_ = needs_dynamics_ || config.measures_torque;
            sensor->qpos_address = model_->jnt_qposadr[joint_id];
            sensor->dof_address = model_->jnt_dofadr[joint_id];
            sensor->reading.mount_name = config.mjcf_joint;
        } else if (config.type == "force_torque") {
            const int body_id = mj_name2id(model_, mjOBJ_BODY, config.mjcf_body.c_str());
            if (body_id < 0) {
                throw std::invalid_argument(owner + ": MJCF body not found: " + config.mjcf_body);
            }
            if (body_id == 0) {
                throw std::invalid_argument(owner + ": a force-torque sensor cannot be mounted on the world");
            }
            if (implementation->create_force_torque == nullptr) {
                throw std::invalid_argument(owner + ": implementation " + config.implementation
                    + " has no force-torque sensor adapter");
            }
            sa::ForceTorqueAdapterSetup setup;
            setup.config_path = config.definition_path;
            setup.vehicle_name = config.pdu_robot;
            setup.sensor_name = config.name;
            setup.physics_step_ns = physics_step_usec * 1000;
            setup.has_random_seed = true;
            setup.random_seed = seed;
            sensor->force_torque_adapter = implementation->create_force_torque(setup, error);
            if (sensor->force_torque_adapter == nullptr) {
                throw std::invalid_argument(owner + ": cannot create the "
                    + config.implementation + " force-torque sensor adapter: " + error);
            }
            sensor->force_torque = true;
            sensor->body_id = body_id;
            sensor->root_id = model_->body_rootid[body_id];
            for (int axis = 0; axis < 3; ++axis) {
                sensor->mount_position[axis] = config.position_m[static_cast<std::size_t>(axis)];
            }
            rotation_from_rpy_deg(config.orientation_deg, sensor->mount_rotation);
            sensor->reading.mount_name = config.mjcf_body;
            needs_dynamics_ = true;
        } else if (config.type == "imu") {
            const int body_id = mj_name2id(model_, mjOBJ_BODY, config.mjcf_body.c_str());
            if (body_id < 0) {
                throw std::invalid_argument(owner + ": MJCF body not found: " + config.mjcf_body);
            }
            if (body_id == 0) {
                throw std::invalid_argument(owner + ": an IMU cannot be mounted on the world");
            }
            if (implementation->create_imu == nullptr) {
                throw std::invalid_argument(owner + ": implementation " + config.implementation
                    + " has no IMU adapter");
            }
            sa::ImuAdapterSetup setup;
            setup.config_path = config.definition_path;
            setup.vehicle_name = config.pdu_robot;
            setup.sensor_name = config.name;
            setup.physics_step_ns = physics_step_usec * 1000;
            setup.has_random_seed = true;
            setup.random_seed = seed;
            sensor->imu_adapter = implementation->create_imu(setup, error);
            if (sensor->imu_adapter == nullptr) {
                throw std::invalid_argument(owner + ": cannot create the "
                    + config.implementation + " IMU adapter: " + error);
            }
            sensor->imu = true;
            sensor->body_id = body_id;
            sensor->root_id = model_->body_rootid[body_id];
            for (int axis = 0; axis < 3; ++axis) {
                sensor->mount_position[axis] = config.position_m[static_cast<std::size_t>(axis)];
            }
            rotation_from_rpy_deg(config.orientation_deg, sensor->mount_rotation);
            sensor->reading.mount_name = config.mjcf_body;
            needs_dynamics_ = true;
        } else if (config.type == "mag" || config.type == "baro" || config.type == "gps") {
            const int body_id = mj_name2id(model_, mjOBJ_BODY, config.mjcf_body.c_str());
            if (body_id < 0) {
                throw std::invalid_argument(owner + ": MJCF body not found: " + config.mjcf_body);
            }
            auto fill = [&](sa::SensorAdapterSetup& setup) {
                setup.config_path = config.definition_path;
                setup.vehicle_name = config.pdu_robot;
                setup.sensor_name = config.name;
                setup.physics_step_ns = physics_step_usec * 1000;
                setup.has_random_seed = true;
                setup.random_seed = seed;
            };
            const bool provided = config.type == "mag" ? implementation->create_mag != nullptr
                : config.type == "baro" ? implementation->create_baro != nullptr
                : implementation->create_gps != nullptr;
            if (!provided) {
                throw std::invalid_argument(owner + ": implementation " + config.implementation
                    + " has no " + config.type + " adapter");
            }
            if (config.type == "mag") {
                sa::MagAdapterSetup setup;
                fill(setup);
                sensor->mag_adapter = implementation->create_mag(setup, error);
                sensor->mag = sensor->mag_adapter != nullptr;
            } else if (config.type == "baro") {
                sa::BaroAdapterSetup setup;
                fill(setup);
                sensor->baro_adapter = implementation->create_baro(setup, error);
                sensor->baro = sensor->baro_adapter != nullptr;
            } else {
                sa::GpsAdapterSetup setup;
                fill(setup);
                setup.origin = {world_.latitude_deg, world_.longitude_deg, world_.altitude_m};
                sensor->gps_adapter = implementation->create_gps(setup, error);
                sensor->gps = sensor->gps_adapter != nullptr;
            }
            if (!sensor->mag && !sensor->baro && !sensor->gps) {
                throw std::invalid_argument(owner + ": cannot create the "
                    + config.implementation + " " + config.type + " adapter: " + error);
            }
            sensor->body_id = body_id;
            sensor->root_id = model_->body_rootid[body_id];
            for (int axis = 0; axis < 3; ++axis) {
                sensor->mount_position[axis] = config.position_m[static_cast<std::size_t>(axis)];
            }
            rotation_from_rpy_deg(config.orientation_deg, sensor->mount_rotation);
            sensor->reading.mount_name = config.mjcf_body;
            needs_kinematics_ = true;
        } else if (config.type == "lidar" || config.type == "rangefinder") {
            const bool is_lidar = config.type == "lidar";
            const int body_id = mj_name2id(model_, mjOBJ_BODY, config.mjcf_body.c_str());
            if (body_id < 0) {
                throw std::invalid_argument(owner + ": MJCF body not found: " + config.mjcf_body);
            }
            if (body_id == 0) {
                throw std::invalid_argument(owner
                    + ": a range sensor is mounted on a body (its housing), not on the world");
            }
            if (!config.exclude_mjcf_body.empty()
                && mj_name2id(model_, mjOBJ_BODY, config.exclude_mjcf_body.c_str()) <= 0) {
                throw std::invalid_argument(owner + ": MJCF body to exclude not found: "
                    + config.exclude_mjcf_body);
            }
            if (is_lidar ? implementation->create_lidar == nullptr
                         : implementation->create_rangefinder == nullptr) {
                throw std::invalid_argument(owner + ": implementation " + config.implementation
                    + (is_lidar ? " has no LiDAR adapter" : " has no rangefinder adapter"));
            }
            if (!ray_caster_factory) {
                throw std::invalid_argument(owner + ": the world provides no ray caster");
            }
            mjtNum mount_rotation[9];
            rotation_from_rpy_deg(config.orientation_deg, mount_rotation);
            mjtNum q[4];
            mju_mat2Quat(q, mount_rotation);
            RayCasterMount mount;
            mount.body = config.mjcf_body;
            mount.position_m = config.position_m;
            mount.orientation_wxyz = {q[0], q[1], q[2], q[3]};
            mount.exclude_root_body = config.exclude_mjcf_body;
            sensor->ray_caster = ray_caster_factory(mount);
            if (sensor->ray_caster == nullptr) {
                throw std::invalid_argument(owner + ": the world provides no ray caster");
            }
            auto fill = [&](sa::RangeImageAdapterSetup& setup) {
                setup.config_path = config.definition_path;
                setup.vehicle_name = config.pdu_robot;
                setup.sensor_name = config.name;
                setup.physics_step_ns = physics_step_usec * 1000;
                setup.has_random_seed = true;
                setup.random_seed = seed;
                setup.ray_caster = sensor->ray_caster.get();
            };
            if (is_lidar) {
                sa::LidarAdapterSetup setup;
                fill(setup);
                sensor->lidar_adapter = implementation->create_lidar(setup, error);
                sensor->lidar = sensor->lidar_adapter != nullptr;
            } else {
                sa::RangefinderAdapterSetup setup;
                fill(setup);
                sensor->rangefinder_adapter = implementation->create_rangefinder(setup, error);
                sensor->rangefinder = sensor->rangefinder_adapter != nullptr;
            }
            if (!sensor->lidar && !sensor->rangefinder) {
                throw std::invalid_argument(owner + ": cannot create the "
                    + config.implementation + " " + config.type + " adapter: " + error);
            }
            sensor->body_id = body_id;
            for (int axis = 0; axis < 3; ++axis) {
                sensor->mount_position[axis] = config.position_m[static_cast<std::size_t>(axis)];
            }
            mju_copy(sensor->mount_rotation, mount_rotation, 9);
            sensor->reading.mount_name = config.mjcf_body;
            needs_kinematics_ = true;
        } else {
            throw std::invalid_argument(owner + ": type " + config.type
                + " is not supported by the MuJoCo sensor stage yet");
        }
        sensors_.push_back(std::move(sensor));
    }
}

MujocoSensorStage::~MujocoSensorStage() = default;

void MujocoSensorStage::step_joint(Sensor& sensor, std::uint64_t local_time_ns)
{
    sa::JointSensorTruth truth;
    truth.sim_time_ns = local_time_ns;
    truth.position = data_->qpos[sensor.qpos_address];
    truth.velocity = data_->qvel[sensor.dof_address];
    truth.actuator_torque = data_->qfrc_actuator[sensor.dof_address];
    if (sensor.measures_torque) {
        // The torque the parent side exerts on the child side about the joint
        // axis: the child body's cfrc_int (at the subtree COM of its root)
        // moved to the joint anchor and projected on the joint axis.
        mjtNum spatial[6];
        mju_transformSpatial(spatial, data_->cfrc_int + 6 * sensor.body_id, 1,
            data_->xanchor + 3 * sensor.joint_id,
            data_->subtree_com + 3 * sensor.root_id, nullptr);
        truth.transmitted_torque = mju_dot3(spatial, data_->xaxis + 3 * sensor.joint_id);
        truth.has_transmitted_torque = true;
    } else {
        truth.transmitted_torque = truth.actuator_torque;
        truth.has_transmitted_torque = false;
    }

    sa::JointSensorSample sample;
    std::string error;
    if (!sensor.joint_adapter->step(truth, sample, error)) {
        throw std::runtime_error("sensor " + sensor.name + " failed: " + error);
    }
    sensor.reading.updated = sample.updated;
    if (sample.updated) {
        sensor.reading.has_sample = true;
        sensor.reading.sample_time_usec = first_step_time_usec_ + sample.sample_time_ns / 1000;
        sensor.reading.position = sample.position;
        sensor.reading.velocity = sample.velocity;
        sensor.reading.has_effort = sample.has_effort;
        sensor.reading.effort = sample.effort;
    }
}

void MujocoSensorStage::step_force_torque(Sensor& sensor, std::uint64_t local_time_ns)
{
    // Sensor origin and axes in the world: the body pose composed with the mount.
    const mjtNum* body_rotation = data_->xmat + 9 * sensor.body_id;
    mjtNum origin[3];
    mju_mulMatVec(origin, body_rotation, sensor.mount_position, 3, 3);
    mju_addTo3(origin, data_->xpos + 3 * sensor.body_id);
    mjtNum rotation[9];  // world <- sensor
    mju_mulMatMat(rotation, body_rotation, sensor.mount_rotation, 3, 3, 3);

    // cfrc_int is what the robot side (parent) exerts on the tool side, at the
    // subtree COM of the root, world axes. Move it to the sensor origin and
    // axes, and negate it: the interface wants what the tool exerts on the
    // robot (a hanging tool reads its weight along gravity).
    mjtNum spatial[6];
    mju_transformSpatial(spatial, data_->cfrc_int + 6 * sensor.body_id, 1, origin,
        data_->subtree_com + 3 * sensor.root_id, rotation);

    sa::ForceTorqueTruth truth;
    truth.sim_time_ns = local_time_ns;
    truth.wrench.torque = {-spatial[0], -spatial[1], -spatial[2]};
    truth.wrench.force = {-spatial[3], -spatial[4], -spatial[5]};
    mjtNum q[4];
    mju_mat2Quat(q, rotation);  // sensor -> world
    truth.orientation_world_to_sensor = {q[0], -q[1], -q[2], -q[3]};
    truth.gravity_world = {model_->opt.gravity[0], model_->opt.gravity[1], model_->opt.gravity[2]};

    sa::ForceTorqueSample sample;
    std::string error;
    if (!sensor.force_torque_adapter->step(truth, sample, error)) {
        throw std::runtime_error("sensor " + sensor.name + " failed: " + error);
    }
    sensor.reading.updated = sample.updated;
    if (sample.updated) {
        sensor.reading.has_sample = true;
        sensor.reading.has_wrench = true;
        sensor.reading.sample_time_usec = first_step_time_usec_ + sample.sample_time_ns / 1000;
        sensor.reading.force = {sample.wrench.force.x, sample.wrench.force.y, sample.wrench.force.z};
        sensor.reading.torque = {sample.wrench.torque.x, sample.wrench.torque.y, sample.wrench.torque.z};
    }
}

void MujocoSensorStage::step_imu(Sensor& sensor, std::uint64_t local_time_ns)
{
    // Sensor origin and axes in the world: the body pose composed with the mount.
    const mjtNum* body_rotation = data_->xmat + 9 * sensor.body_id;
    mjtNum origin[3];
    mju_mulMatVec(origin, body_rotation, sensor.mount_position, 3, 3);
    mju_addTo3(origin, data_->xpos + 3 * sensor.body_id);
    mjtNum rotation[9];  // world <- sensor
    mju_mulMatMat(rotation, body_rotation, sensor.mount_rotation, 3, 3, 3);

    // Velocity and acceleration of the body at the sensor point (world axes).
    // cvel and cacc are given at the subtree COM of the root; cacc includes
    // -gravity, so the classical acceleration below is the specific force an
    // ideal accelerometer measures: a = a_spatial(p) + w x v(p), the same as
    // mj_objectAcceleration.
    const mjtNum* com = data_->subtree_com + 3 * sensor.root_id;
    mjtNum velocity[6], acceleration[6];
    mju_transformSpatial(velocity, data_->cvel + 6 * sensor.body_id, 0, origin, com, nullptr);
    mju_transformSpatial(acceleration, data_->cacc + 6 * sensor.body_id, 0, origin, com, nullptr);
    mjtNum coriolis[3];
    mju_cross(coriolis, velocity, velocity + 3);
    mjtNum specific_force_world[3];
    mju_add3(specific_force_world, acceleration + 3, coriolis);

    mjtNum angular_velocity[3], specific_force[3];
    mju_mulMatTVec3(angular_velocity, rotation, velocity);
    mju_mulMatTVec3(specific_force, rotation, specific_force_world);

    sa::ImuTruth truth;
    truth.sim_time_ns = local_time_ns;
    truth.angular_velocity = {angular_velocity[0], angular_velocity[1], angular_velocity[2]};
    truth.specific_force = {specific_force[0], specific_force[1], specific_force[2]};
    mjtNum q[4];
    mju_mat2Quat(q, rotation);  // sensor -> world
    truth.orientation_world_to_sensor = {q[0], -q[1], -q[2], -q[3]};
    truth.gravity_world = {model_->opt.gravity[0], model_->opt.gravity[1], model_->opt.gravity[2]};

    sa::ImuSample sample;
    std::string error;
    if (!sensor.imu_adapter->step(truth, sample, error)) {
        throw std::runtime_error("sensor " + sensor.name + " failed: " + error);
    }
    sensor.reading.updated = sample.gyro.updated || sample.accel.updated;
    if (sample.gyro.updated) {
        sensor.gyro_sampled = true;
        sensor.reading.angular_velocity = {sample.gyro.value.x, sample.gyro.value.y, sample.gyro.value.z};
        sensor.reading.sample_time_usec = first_step_time_usec_ + sample.gyro.sample_time_ns / 1000;
    }
    if (sample.accel.updated) {
        sensor.accel_sampled = true;
        sensor.reading.linear_acceleration = {sample.accel.value.x, sample.accel.value.y, sample.accel.value.z};
        sensor.reading.sample_time_usec = first_step_time_usec_ + sample.accel.sample_time_ns / 1000;
    }
    // Published once both the gyroscope and the accelerometer have measured.
    sensor.reading.has_imu = sensor.gyro_sampled && sensor.accel_sampled;
    sensor.reading.has_sample = sensor.reading.has_imu;
}

void MujocoSensorStage::mount_pose(const Sensor& sensor, double origin[3], double rotation[9]) const
{
    // The body pose composed with the mount: origin in the world, and the
    // rotation world <- sensor (row-major).
    const mjtNum* body_rotation = data_->xmat + 9 * sensor.body_id;
    mju_mulMatVec(origin, body_rotation, sensor.mount_position, 3, 3);
    mju_addTo3(origin, data_->xpos + 3 * sensor.body_id);
    mju_mulMatMat(rotation, body_rotation, sensor.mount_rotation, 3, 3, 3);
}

void MujocoSensorStage::step_mag(Sensor& sensor, std::uint64_t local_time_ns)
{
    mjtNum origin[3], rotation[9];
    mount_pose(sensor, origin, rotation);
    // The earth field in NED from intensity, declination and inclination,
    // then NWU (x north, y west, z up), then the sensor axes.
    const double f = world_.magnetic_intensity_nT;
    const double d = world_.magnetic_declination_deg * kDegToRad;
    const double i = world_.magnetic_inclination_deg * kDegToRad;
    const mjtNum field_world[3] = {
        f * std::cos(i) * std::cos(d),
        -f * std::cos(i) * std::sin(d),
        -f * std::sin(i)};
    mjtNum field[3];
    mju_mulMatTVec3(field, rotation, field_world);

    sa::MagTruth truth;
    truth.sim_time_ns = local_time_ns;
    truth.magnetic_field = {field[0], field[1], field[2]};
    truth.magnetic_field_world = {field_world[0], field_world[1], field_world[2]};
    mjtNum q[4];
    mju_mat2Quat(q, rotation);  // sensor -> world
    truth.orientation_world_to_sensor = {q[0], -q[1], -q[2], -q[3]};

    sa::MagSample sample;
    std::string error;
    if (!sensor.mag_adapter->step(truth, sample, error)) {
        throw std::runtime_error("sensor " + sensor.name + " failed: " + error);
    }
    sensor.reading.updated = sample.updated;
    if (sample.updated) {
        sensor.reading.has_sample = true;
        sensor.reading.has_magnetic_field = true;
        sensor.reading.sample_time_usec = first_step_time_usec_ + sample.sample_time_ns / 1000;
        sensor.reading.magnetic_field_nT = {sample.value.x, sample.value.y, sample.value.z};
    }
}

void MujocoSensorStage::step_baro(Sensor& sensor, std::uint64_t local_time_ns)
{
    mjtNum origin[3], rotation[9];
    mount_pose(sensor, origin, rotation);
    sa::BaroTruth truth;
    truth.sim_time_ns = local_time_ns;
    truth.altitude_m = world_.altitude_m + origin[2];
    truth.sea_level_pressure_pa = world_.sea_level_pressure_pa;
    truth.sea_level_temperature_c = world_.sea_level_temperature_c;

    sa::BaroSample sample;
    std::string error;
    if (!sensor.baro_adapter->step(truth, sample, error)) {
        throw std::runtime_error("sensor " + sensor.name + " failed: " + error);
    }
    sensor.reading.updated = sample.updated;
    if (sample.updated) {
        sensor.reading.has_sample = true;
        sensor.reading.has_pressure = true;
        sensor.reading.sample_time_usec = first_step_time_usec_ + sample.sample_time_ns / 1000;
        sensor.reading.pressure_pa = sample.pressure_pa;
    }
}

void MujocoSensorStage::step_gps(Sensor& sensor, std::uint64_t local_time_ns)
{
    mjtNum origin[3], rotation[9];
    mount_pose(sensor, origin, rotation);
    // Velocity of the antenna point (world axes, NWU): cvel moved to the point.
    mjtNum velocity[6];
    mju_transformSpatial(velocity, data_->cvel + 6 * sensor.body_id, 0, origin,
        data_->subtree_com + 3 * sensor.root_id, nullptr);

    sa::GpsTruth truth;
    truth.sim_time_ns = local_time_ns;
    truth.position_nwu = {origin[0], origin[1], origin[2]};
    truth.velocity_nwu = {velocity[3], velocity[4], velocity[5]};
    const auto geodetic = nwu_to_geodetic(world_, origin);
    truth.position = {geodetic[0], geodetic[1], geodetic[2]};

    sa::GpsSample sample;
    std::string error;
    if (!sensor.gps_adapter->step(truth, sample, error)) {
        throw std::runtime_error("sensor " + sensor.name + " failed: " + error);
    }
    sensor.reading.updated = sample.updated;
    if (sample.updated) {
        sensor.reading.has_sample = true;
        sensor.reading.has_fix = true;
        sensor.reading.sample_time_usec = first_step_time_usec_ + sample.sample_time_ns / 1000;
        auto& fix = sensor.reading.fix;
        fix.latitude_deg = sample.position.latitude_deg;
        fix.longitude_deg = sample.position.longitude_deg;
        fix.altitude_m = sample.position.altitude_m;
        fix.velocity_nwu = {sample.velocity_nwu.x, sample.velocity_nwu.y, sample.velocity_nwu.z};
        fix.horizontal_accuracy_m = sample.horizontal_accuracy_m;
        fix.vertical_accuracy_m = sample.vertical_accuracy_m;
        fix.fix_type = sample.fix_type;
    }
}

sa::SensorPoseTruth MujocoSensorStage::sensor_pose(const Sensor& sensor, std::uint64_t local_time_ns) const
{
    // Sensor pose in the world: the body pose composed with the mount (the
    // same pose the ray caster casts from).
    const mjtNum* body_rotation = data_->xmat + 9 * sensor.body_id;
    mjtNum origin[3];
    mju_mulMatVec(origin, body_rotation, sensor.mount_position, 3, 3);
    mju_addTo3(origin, data_->xpos + 3 * sensor.body_id);
    mjtNum rotation[9];  // world <- sensor
    mju_mulMatMat(rotation, body_rotation, sensor.mount_rotation, 3, 3, 3);

    sa::SensorPoseTruth truth;
    truth.sim_time_ns = local_time_ns;
    truth.position_world = {origin[0], origin[1], origin[2]};
    mjtNum q[4];
    mju_mat2Quat(q, rotation);  // sensor -> world
    truth.orientation_world_to_sensor = {q[0], -q[1], -q[2], -q[3]};
    return truth;
}

void MujocoSensorStage::step_rangefinder(Sensor& sensor, std::uint64_t local_time_ns)
{
    const auto truth = sensor_pose(sensor, local_time_ns);
    sa::RangeSample sample;
    std::string error;
    if (!sensor.rangefinder_adapter->step(truth, sample, error)) {
        throw std::runtime_error("sensor " + sensor.name + " failed: " + error);
    }
    sensor.reading.updated = sample.updated;
    if (!sample.updated) {
        return;
    }
    sensor.reading.has_sample = true;
    sensor.reading.has_range = true;
    sensor.reading.sample_time_usec = first_step_time_usec_ + sample.sample_time_ns / 1000;
    sensor.reading.range.radiation_type = static_cast<std::uint8_t>(sample.radiation_type);
    sensor.reading.range.field_of_view = sample.field_of_view;
    sensor.reading.range.min_range = sample.min_range;
    sensor.reading.range.max_range = sample.max_range;
    sensor.reading.range.range = sample.range;
    sensor.reading.range.variance = sample.variance;
}

void MujocoSensorStage::step_lidar(Sensor& sensor, std::uint64_t local_time_ns)
{
    const auto truth = sensor_pose(sensor, local_time_ns);
    sa::LidarScan scan;
    std::string error;
    if (!sensor.lidar_adapter->step(truth, scan, error)) {
        throw std::runtime_error("sensor " + sensor.name + " failed: " + error);
    }
    sensor.reading.updated = scan.updated;
    if (!scan.updated) {
        return;
    }
    auto reading = std::make_shared<runtime::LidarScanReading>();
    reading->angle_min = scan.angle_min;
    reading->angle_max = scan.angle_max;
    reading->angle_increment = scan.angle_increment;
    reading->vertical_angle_min = scan.vertical_angle_min;
    reading->vertical_angle_max = scan.vertical_angle_max;
    reading->vertical_angle_increment = scan.vertical_angle_increment;
    reading->horizontal_count = scan.horizontal_count;
    reading->vertical_count = scan.vertical_count;
    reading->time_increment = scan.time_increment;
    reading->scan_time = scan.scan_time;
    reading->range_min = scan.range_min;
    reading->range_max = scan.range_max;
    const std::size_t count =
        static_cast<std::size_t>(scan.horizontal_count) * scan.vertical_count;
    if (count > 0 && scan.ranges != nullptr) {
        reading->ranges.assign(scan.ranges, scan.ranges + count);
    }
    if (count > 0 && scan.intensities != nullptr) {
        reading->intensities.assign(scan.intensities, scan.intensities + count);
    }
    if (scan.points != nullptr) {
        reading->points.reserve(scan.point_count);
        for (std::uint32_t i = 0; i < scan.point_count; ++i) {
            const auto& point = scan.points[i];
            reading->points.push_back(
                {point.x, point.y, point.z, point.intensity, point.ring, point.time_offset});
        }
    }
    sensor.reading.has_sample = true;
    // ROS LaserScan and PointCloud2: the stamp is the time of the first ray.
    sensor.reading.sample_time_usec = first_step_time_usec_ + scan.sample_time_ns / 1000;
    sensor.reading.scan = std::move(reading);
}

void MujocoSensorStage::step(runtime::RobotState& state)
{
    if (steps_ == 0) {
        first_step_time_usec_ = state.sample_time_usec;
    } else if (state.sample_time_usec != first_step_time_usec_ + steps_ * physics_step_usec_) {
        throw std::runtime_error("sensor stage: the Runtime time did not advance by one physics step");
    }
    const std::uint64_t local_time_ns = steps_ * physics_step_usec_ * 1000;
    ++steps_;

    if (needs_dynamics_) {
        // Forces at the state after the step: mj_step computed them for the
        // state before it. Both calls only recompute derived quantities.
        mj_forward(model_, data_);
        mj_rnePostConstraint(model_, data_);
    } else if (needs_kinematics_) {
        // Geom poses at the state after the step, for the rays.
        mj_forward(model_, data_);
    }

    state.sensor_readings.clear();
    state.sensor_readings.reserve(sensors_.size());
    for (auto& sensor : sensors_) {
        if (sensor->force_torque) {
            step_force_torque(*sensor, local_time_ns);
        } else if (sensor->imu) {
            step_imu(*sensor, local_time_ns);
        } else if (sensor->lidar) {
            step_lidar(*sensor, local_time_ns);
        } else if (sensor->rangefinder) {
            step_rangefinder(*sensor, local_time_ns);
        } else if (sensor->mag) {
            step_mag(*sensor, local_time_ns);
        } else if (sensor->baro) {
            step_baro(*sensor, local_time_ns);
        } else if (sensor->gps) {
            step_gps(*sensor, local_time_ns);
        } else {
            step_joint(*sensor, local_time_ns);
        }
        state.sensor_readings.push_back(sensor->reading);
    }
}

void MujocoSensorStage::reset()
{
    steps_ = 0;
    first_step_time_usec_ = 0;
    for (auto& sensor : sensors_) {
        std::string error;
        bool reset = false;
        if (sensor->force_torque) {
            reset = sensor->force_torque_adapter->reset(error);
        } else if (sensor->imu) {
            reset = sensor->imu_adapter->reset(error);
        } else if (sensor->lidar) {
            reset = sensor->lidar_adapter->reset(error);
        } else if (sensor->rangefinder) {
            reset = sensor->rangefinder_adapter->reset(error);
        } else if (sensor->mag) {
            reset = sensor->mag_adapter->reset(error);
        } else if (sensor->baro) {
            reset = sensor->baro_adapter->reset(error);
        } else if (sensor->gps) {
            reset = sensor->gps_adapter->reset(error);
        } else {
            reset = sensor->joint_adapter->reset(error);
        }
        sensor->gyro_sampled = false;
        sensor->accel_sampled = false;
        if (!reset) {
            throw std::runtime_error("sensor " + sensor->name + " reset failed: " + error);
        }
        const auto name = sensor->reading.sensor_name;
        const auto mount = sensor->reading.mount_name;
        sensor->reading = {};
        sensor->reading.sensor_name = name;
        sensor->reading.mount_name = mount;
    }
}

} // namespace hakoniwa::robot_runtime::adapters::mujoco
