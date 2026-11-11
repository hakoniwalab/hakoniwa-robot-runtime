#include "adapters/physics/mujoco/mujoco_sensor_stage.hpp"

#include <mujoco/mujoco.h>

#include <stdexcept>
#include <string>
#include <utility>

namespace hakoniwa::robot_runtime::adapters::mujoco {

namespace sa = ::hakoniwa::sensor_adapter;

struct MujocoSensorStage::Sensor {
    std::string name;
    std::string mjcf_joint;
    int joint_id {-1};
    int body_id {-1};
    int root_id {-1};
    int qpos_address {-1};
    int dof_address {-1};
    bool measures_torque {false};
    std::unique_ptr<sa::IJointSensorAdapter> adapter;
    runtime::SensorReading reading;
};

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
        if (config.type != "joint") {
            throw std::invalid_argument(owner + ": type " + config.type
                + " is not supported by the MuJoCo sensor stage yet");
        }
        const int joint_id = mj_name2id(model_, mjOBJ_JOINT, config.mjcf_joint.c_str());
        if (joint_id < 0) {
            throw std::invalid_argument(owner + ": MJCF joint not found: " + config.mjcf_joint);
        }
        const int joint_type = model_->jnt_type[joint_id];
        if (joint_type != mjJNT_HINGE && joint_type != mjJNT_SLIDE) {
            throw std::invalid_argument(owner + ": MJCF joint " + config.mjcf_joint
                + " is not a hinge or slide joint");
        }
        const auto* implementation = registry.find(config.implementation);
        if (implementation == nullptr || implementation->create_joint == nullptr) {
            throw std::invalid_argument(owner + ": implementation " + config.implementation
                + " has no joint sensor adapter linked");
        }

        sa::JointSensorAdapterSetup setup;
        setup.config_path = config.definition_path;
        setup.vehicle_name = config.pdu_robot;
        setup.sensor_name = config.name;
        setup.physics_step_ns = physics_step_usec * 1000;
        setup.has_random_seed = true;
        setup.random_seed = seed;
        std::string error;
        auto adapter = implementation->create_joint(setup, error);
        if (adapter == nullptr) {
            throw std::invalid_argument(owner + ": cannot create the "
                + config.implementation + " joint sensor adapter: " + error);
        }

        auto sensor = std::make_unique<Sensor>();
        sensor->name = config.name;
        sensor->mjcf_joint = config.mjcf_joint;
        sensor->joint_id = joint_id;
        sensor->body_id = model_->jnt_bodyid[joint_id];
        sensor->root_id = model_->body_rootid[sensor->body_id];
        sensor->measures_torque = config.measures_torque;
        needs_dynamics_ = needs_dynamics_ || config.measures_torque;
        sensor->qpos_address = model_->jnt_qposadr[joint_id];
        sensor->dof_address = model_->jnt_dofadr[joint_id];
        sensor->adapter = std::move(adapter);
        sensor->reading.sensor_name = config.name;
        sensor->reading.mount_name = config.mjcf_joint;
        sensors_.push_back(std::move(sensor));
    }
}

MujocoSensorStage::~MujocoSensorStage() = default;

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
        sa::JointSensorTruth truth;
        truth.sim_time_ns = local_time_ns;
        truth.position = data_->qpos[sensor->qpos_address];
        truth.velocity = data_->qvel[sensor->dof_address];
        truth.actuator_torque = data_->qfrc_actuator[sensor->dof_address];
        if (sensor->measures_torque) {
            // The torque the parent side exerts on the child side about the joint
            // axis: the child body's cfrc_int (at the subtree COM of its root)
            // moved to the joint anchor and projected on the joint axis.
            mjtNum spatial[6];
            mju_transformSpatial(spatial, data_->cfrc_int + 6 * sensor->body_id, 1,
                data_->xanchor + 3 * sensor->joint_id,
                data_->subtree_com + 3 * sensor->root_id, nullptr);
            truth.transmitted_torque = mju_dot3(spatial, data_->xaxis + 3 * sensor->joint_id);
            truth.has_transmitted_torque = true;
        } else {
            truth.transmitted_torque = truth.actuator_torque;
            truth.has_transmitted_torque = false;
        }

        sa::JointSensorSample sample;
        std::string error;
        if (!sensor->adapter->step(truth, sample, error)) {
            throw std::runtime_error("sensor " + sensor->name + " failed: " + error);
        }
        sensor->reading.updated = sample.updated;
        if (sample.updated) {
            sensor->reading.has_sample = true;
            sensor->reading.sample_time_usec = first_step_time_usec_ + sample.sample_time_ns / 1000;
            sensor->reading.position = sample.position;
            sensor->reading.velocity = sample.velocity;
            sensor->reading.has_effort = sample.has_effort;
            sensor->reading.effort = sample.effort;
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
        if (!sensor->adapter->reset(error)) {
            throw std::runtime_error("sensor " + sensor->name + " reset failed: " + error);
        }
        sensor->reading.has_sample = false;
        sensor->reading.updated = false;
        sensor->reading.sample_time_usec = 0;
        sensor->reading.position = 0.0;
        sensor->reading.velocity = 0.0;
        sensor->reading.has_effort = false;
        sensor->reading.effort = 0.0;
    }
}

} // namespace hakoniwa::robot_runtime::adapters::mujoco
