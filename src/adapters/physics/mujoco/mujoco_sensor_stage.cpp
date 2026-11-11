#include "adapters/physics/mujoco/mujoco_sensor_stage.hpp"

#include <mujoco/mujoco.h>

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

} // namespace

MujocoSensorStage::MujocoSensorStage(
    const mjModel* model,
    mjData* data,
    const std::vector<runtime::RuntimeSensorConfig>& sensors,
    const std::uint64_t seed,
    const std::uint64_t physics_step_usec,
    const runtime::sensor::SensorImplementationRegistry& registry)
    : model_(model)
    , data_(data)
    , physics_step_usec_(physics_step_usec)
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
    }

    state.sensor_readings.clear();
    state.sensor_readings.reserve(sensors_.size());
    for (auto& sensor : sensors_) {
        if (sensor->force_torque) {
            step_force_torque(*sensor, local_time_ns);
        } else if (sensor->imu) {
            step_imu(*sensor, local_time_ns);
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
        const bool reset = sensor->force_torque
            ? sensor->force_torque_adapter->reset(error)
            : (sensor->imu ? sensor->imu_adapter->reset(error) : sensor->joint_adapter->reset(error));
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
