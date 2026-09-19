#include "hakoniwa/robot_runtime/adapters/physics/mujoco/mujoco_actuator_plant.hpp"

#include "physics/physics_impl.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace hakoniwa::robot_runtime::adapters::mujoco {
namespace {

int require_named_id(
    const mjModel* model,
    const mjtObj object_type,
    const std::string& name,
    const char* label)
{
    const int id = mj_name2id(model, object_type, name.c_str());
    if (id < 0) {
        throw std::invalid_argument(
            std::string(label) + " not found in MuJoCo model: " + name);
    }
    return id;
}

std::uint64_t model_step_usec(const mjModel* model)
{
    const double usec = model->opt.timestep * 1'000'000.0;
    if (!std::isfinite(usec) || usec < 1.0
        || usec > static_cast<double>(
            std::numeric_limits<std::uint64_t>::max())) {
        throw std::invalid_argument(
            "MuJoCo timestep is not representable in microseconds");
    }
    const auto rounded = static_cast<std::uint64_t>(std::llround(usec));
    if (std::abs(usec - static_cast<double>(rounded)) > 1.0e-6) {
        throw std::invalid_argument(
            "MuJoCo timestep must be an integral number of microseconds");
    }
    return rounded;
}

std::uint64_t model_time_usec(const mjData* data)
{
    const double usec = data->time * 1'000'000.0;
    if (!std::isfinite(usec) || usec < 0.0
        || usec > static_cast<double>(
            std::numeric_limits<std::uint64_t>::max())) {
        throw std::runtime_error(
            "MuJoCo simulation time is not representable in microseconds");
    }
    return static_cast<std::uint64_t>(std::llround(usec));
}

double clamp_value(
    const double value,
    const runtime::ActuatorLimits& limits) noexcept
{
    return std::clamp(value, limits.minimum, limits.maximum);
}

} // namespace

class MujocoActuatorPlant::Impl {
public:
    struct Binding {
        runtime::RuntimeActuatorConfig config;
        std::shared_ptr<hako::robots::actuator::IJointActuator> actuator;
        int qpos_address {-1};
        int qvel_address {-1};
        int actuator_id {-1};
    };

    struct BodyBinding {
        std::string state_id;
        int joint_id {-1};
        int qpos_address {-1};
        int body_id {-1};
    };

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR
    struct LocalContactBody {
        std::string body_id;
        int root_body_id {-1};
    };

    struct MirrorBinding {
        std::string mirror_id;
        int joint_id {-1};
        int qpos_address {-1};
        int qvel_address {-1};
        int root_body_id {-1};
        std::vector<LocalContactBody> contact_bodies;
    };
#endif

    explicit Impl(const runtime::RuntimeDefinition& definition)
        : world(std::make_shared<hako::robots::physics::impl::WorldImpl>())
    {
        world->loadModel(definition.model_path);
        auto* model = world->getModel();
        if (model == nullptr || world->getData() == nullptr) {
            throw std::runtime_error(
                "MuJoCo model did not create model/data state");
        }
        delta_time_usec = model_step_usec(model);
        initial_body_poses = definition.initial_body_poses;
        apply_initial_body_poses();
        if (!initial_body_poses.empty()) {
            mj_forward(model, world->getData());
        }
        bindings.reserve(definition.actuators.size());
        for (const auto& config : definition.actuators) {
            if (binding_by_id.contains(config.component_id)) {
                throw std::invalid_argument(
                    "duplicate actuator component ID: "
                    + config.component_id);
            }
            if (!config.limits.is_valid()) {
                throw std::invalid_argument(
                    "invalid actuator limits: " + config.component_id);
            }
            Binding binding;
            binding.config = config;
            binding.actuator = world->createJointActuator();
            if (!binding.actuator->LoadConfig(config.config_path)) {
                throw std::invalid_argument(
                    "failed to bind joint actuator config: "
                    + config.config_path);
            }
            const int joint_id = require_named_id(
                model,
                mjOBJ_JOINT,
                config.joint_name,
                "joint");
            binding.qpos_address = model->jnt_qposadr[joint_id];
            binding.qvel_address = model->jnt_dofadr[joint_id];
            binding.actuator_id = require_named_id(
                model,
                mjOBJ_ACTUATOR,
                config.mujoco_actuator_name,
                "actuator");
            binding_by_id.emplace(
                config.component_id,
                bindings.size());
            bindings.push_back(std::move(binding));
        }
        std::unordered_set<std::string> body_state_ids;
        for (const auto& output : definition.multi_dof_state_outputs) {
            for (const auto& config : output.bodies) {
                if (!body_state_ids.insert(config.state_id).second) {
                    throw std::invalid_argument(
                        "duplicate body state ID: " + config.state_id);
                }
                const int joint_id = require_named_id(
                    model,
                    mjOBJ_JOINT,
                    config.mjcf_freejoint,
                    "body state freejoint");
                if (model->jnt_type[joint_id] != mjJNT_FREE) {
                    throw std::invalid_argument(
                        "body state binding is not a freejoint: "
                        + config.mjcf_freejoint);
                }
                body_bindings.push_back({
                    config.state_id,
                    joint_id,
                    model->jnt_qposadr[joint_id],
                    model->jnt_bodyid[joint_id],
                });
            }
        }
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR
        std::unordered_set<std::string> mirror_ids;
        for (const auto& config : definition.mirror_bodies) {
            if (!mirror_ids.insert(config.mirror_id).second) {
                throw std::invalid_argument(
                    "duplicate Mirror ID: " + config.mirror_id);
            }
            const int joint_id = require_named_id(
                model, mjOBJ_JOINT, config.mjcf_freejoint,
                "Mirror freejoint");
            if (model->jnt_type[joint_id] != mjJNT_FREE) {
                throw std::invalid_argument(
                    "Mirror binding is not a freejoint: "
                    + config.mjcf_freejoint);
            }
            MirrorBinding mirror {
                config.mirror_id,
                joint_id,
                model->jnt_qposadr[joint_id],
                model->jnt_dofadr[joint_id],
                model->jnt_bodyid[joint_id],
                {},
            };
            mirror.contact_bodies.reserve(config.contact_bodies.size());
            for (const auto& contact : config.contact_bodies) {
                const int local_joint_id = require_named_id(
                    model, mjOBJ_JOINT, contact.mjcf_freejoint,
                    "Mirror contact body freejoint");
                if (model->jnt_type[local_joint_id] != mjJNT_FREE) {
                    throw std::invalid_argument(
                        "Mirror contact body binding is not a freejoint: "
                        + contact.mjcf_freejoint);
                }
                mirror.contact_bodies.push_back({
                    contact.body_id,
                    model->jnt_bodyid[local_joint_id],
                });
            }
            mirror_binding_by_id.emplace(
                mirror.mirror_id, mirror_bindings.size());
            mirror_bindings.push_back(std::move(mirror));
        }
#endif
    }

    void apply_initial_body_poses()
    {
        auto* model = world->getModel();
        auto* data = world->getData();
        for (const auto& pose : initial_body_poses) {
            const int joint_id = require_named_id(
                model, mjOBJ_JOINT, pose.mjcf_freejoint,
                "initial body pose freejoint");
            if (model->jnt_type[joint_id] != mjJNT_FREE) {
                throw std::invalid_argument(
                    "initial body pose binding is not a freejoint: "
                    + pose.mjcf_freejoint);
            }
            const int qpos = model->jnt_qposadr[joint_id];
            data->qpos[qpos] = pose.position.x;
            data->qpos[qpos + 1] = pose.position.y;
            data->qpos[qpos + 2] = pose.position.z;
            mjtNum euler[3] {
                pose.rpy_rad[0], pose.rpy_rad[1], pose.rpy_rad[2],
            };
            mju_euler2Quat(data->qpos + qpos + 3, euler, "XYZ");
        }
    }

    runtime::RobotState state() const
    {
        const auto* data = world->getData();
        const auto sample_time_usec = model_time_usec(data);
        runtime::RobotState state;
        state.sample_time_usec = sample_time_usec;
        state.actuators.reserve(bindings.size());
        for (const auto& binding : bindings) {
            state.actuators.push_back({
                binding.config.component_id,
                data->qpos[binding.qpos_address],
                data->qvel[binding.qvel_address],
                data->actuator_force[binding.actuator_id],
                sample_time_usec,
            });
        }
        state.bodies.reserve(body_bindings.size());
        for (const auto& binding : body_bindings) {
            const int qpos = binding.qpos_address;
            mjtNum velocity[6] {};
            mj_objectVelocity(
                world->getModel(), data, mjOBJ_BODY,
                binding.body_id, velocity, 0);
            state.bodies.push_back({
                binding.state_id,
                {data->qpos[qpos], data->qpos[qpos + 1], data->qpos[qpos + 2]},
                {data->qpos[qpos + 4], data->qpos[qpos + 5],
                    data->qpos[qpos + 6], data->qpos[qpos + 3]},
                {velocity[3], velocity[4], velocity[5]},
                {velocity[0], velocity[1], velocity[2]},
                sample_time_usec,
            });
        }
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR
        append_mirror_contacts(state);
#endif
        return state;
    }

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR
    bool body_is_descendant(int body_id, int ancestor_body_id) const noexcept
    {
        const auto* model = world->getModel();
        int cursor = body_id;
        while (cursor >= 0) {
            if (cursor == ancestor_body_id) {
                return true;
            }
            const int parent = model->body_parentid[cursor];
            if (parent == cursor) {
                break;
            }
            cursor = parent;
        }
        return false;
    }

    std::array<double, 3> body_position(int body_id) const noexcept
    {
        const auto* data = world->getData();
        return {
            data->xipos[3 * body_id],
            data->xipos[3 * body_id + 1],
            data->xipos[3 * body_id + 2],
        };
    }

    std::array<double, 6> body_velocity(int body_id) const noexcept
    {
        mjtNum velocity[6] {};
        mj_objectVelocity(
            world->getModel(), world->getData(), mjOBJ_BODY,
            body_id, velocity, 0);
        return {
            velocity[0], velocity[1], velocity[2],
            velocity[3], velocity[4], velocity[5],
        };
    }

    runtime::EulerState body_euler(int body_id) const noexcept
    {
        const mjtNum* rotation = &world->getData()->xmat[9 * body_id];
        runtime::EulerState euler;
        euler.pitch = std::asin(std::clamp(
            static_cast<double>(rotation[6]), -1.0, 1.0));
        const double cos_pitch = std::cos(euler.pitch);
        if (std::fabs(cos_pitch) > 1.0e-6) {
            euler.roll = std::atan2(-rotation[7], rotation[8]);
            euler.yaw = std::atan2(rotation[3], rotation[0]);
        } else {
            euler.roll = 0.0;
            euler.yaw = std::atan2(-rotation[1], rotation[4]);
        }
        return euler;
    }

    void append_mirror_contacts(runtime::RobotState& state) const
    {
        const auto* model = world->getModel();
        const auto* data = world->getData();
        for (int index = 0; index < data->ncon; ++index) {
            const auto& contact = data->contact[index];
            if (contact.geom1 < 0 || contact.geom2 < 0) {
                continue;
            }
            const int body1 = model->geom_bodyid[contact.geom1];
            const int body2 = model->geom_bodyid[contact.geom2];
            for (const auto& mirror : mirror_bindings) {
                const bool body1_is_mirror =
                    body_is_descendant(body1, mirror.root_body_id);
                const bool body2_is_mirror =
                    body_is_descendant(body2, mirror.root_body_id);
                if (body1_is_mirror == body2_is_mirror) {
                    continue;
                }
                const int other_body = body1_is_mirror ? body2 : body1;
                const LocalContactBody* local = nullptr;
                for (const auto& candidate : mirror.contact_bodies) {
                    if (body_is_descendant(other_body, candidate.root_body_id)) {
                        local = &candidate;
                        break;
                    }
                }
                if (local == nullptr) {
                    continue;
                }

                std::array<double, 3> normal {
                    contact.frame[0], contact.frame[1], contact.frame[2]};
                if (body1_is_mirror) {
                    normal[0] = -normal[0];
                    normal[1] = -normal[1];
                    normal[2] = -normal[2];
                }
                const std::array<double, 3> point {
                    contact.pos[0], contact.pos[1], contact.pos[2]};
                const auto mirror_position = body_position(mirror.root_body_id);
                const auto target_position = body_position(local->root_body_id);
                const auto mirror_velocity = body_velocity(mirror.root_body_id);
                const auto target_velocity = body_velocity(local->root_body_id);
                const double relative_normal_speed = std::fabs(
                    (mirror_velocity[3] - target_velocity[3]) * normal[0]
                    + (mirror_velocity[4] - target_velocity[4]) * normal[1]
                    + (mirror_velocity[5] - target_velocity[5]) * normal[2]);

                state.mirror_contacts.push_back({
                    mirror.mirror_id,
                    local->body_id,
                    contact.dist,
                    relative_normal_speed,
                    {point[0] - mirror_position[0],
                        point[1] - mirror_position[1],
                        point[2] - mirror_position[2]},
                    {normal[0], normal[1], normal[2]},
                    {point[0] - target_position[0],
                        point[1] - target_position[1],
                        point[2] - target_position[2]},
                    {target_velocity[3], target_velocity[4], target_velocity[5]},
                    {target_velocity[0], target_velocity[1], target_velocity[2]},
                    body_euler(local->root_body_id),
                    {model->body_inertia[3 * local->root_body_id],
                        model->body_inertia[3 * local->root_body_id + 1],
                        model->body_inertia[3 * local->root_body_id + 2]},
                    model->body_mass[local->root_body_id],
                });
                break;
            }
        }
    }

    void apply_mirror_commands(
        const std::vector<runtime::MirrorBodyCommand>& commands,
        std::uint64_t simulation_time_usec)
    {
        std::unordered_map<std::string_view, const runtime::MirrorBodyCommand*> first;
        std::unordered_set<std::string_view> duplicates;
        for (const auto& command : commands) {
            if (!mirror_binding_by_id.contains(command.mirror_id)) {
                continue;
            }
            if (!first.emplace(command.mirror_id, &command).second) {
                duplicates.insert(command.mirror_id);
            }
        }
        auto* data = world->getData();
        for (const auto& mirror : mirror_bindings) {
            const auto iterator = first.find(mirror.mirror_id);
            if (iterator == first.end() || duplicates.contains(mirror.mirror_id)) {
                continue;
            }
            const auto& command = *iterator->second;
            if (command.created_at_usec > simulation_time_usec
                || !std::isfinite(command.position.x)
                || !std::isfinite(command.position.y)
                || !std::isfinite(command.position.z)
                || !std::isfinite(command.orientation.roll)
                || !std::isfinite(command.orientation.pitch)
                || !std::isfinite(command.orientation.yaw)
                || !std::isfinite(command.linear_velocity.x)
                || !std::isfinite(command.linear_velocity.y)
                || !std::isfinite(command.linear_velocity.z)
                || !std::isfinite(command.angular_velocity.x)
                || !std::isfinite(command.angular_velocity.y)
                || !std::isfinite(command.angular_velocity.z)) {
                continue;
            }
            const int qpos = mirror.qpos_address;
            const int qvel = mirror.qvel_address;
            data->qpos[qpos] = command.position.x;
            data->qpos[qpos + 1] = command.position.y;
            data->qpos[qpos + 2] = command.position.z;
            mjtNum euler[3] {
                command.orientation.roll,
                command.orientation.pitch,
                command.orientation.yaw,
            };
            mjtNum quaternion[4] {};
            mju_euler2Quat(quaternion, euler, "XYZ");
            for (int offset = 0; offset < 4; ++offset) {
                data->qpos[qpos + 3 + offset] = quaternion[offset];
            }
            data->qvel[qvel] = command.linear_velocity.x;
            data->qvel[qvel + 1] = command.linear_velocity.y;
            data->qvel[qvel + 2] = command.linear_velocity.z;
            data->qvel[qvel + 3] = command.angular_velocity.x;
            data->qvel[qvel + 4] = command.angular_velocity.y;
            data->qvel[qvel + 5] = command.angular_velocity.z;
        }
        mj_forward(world->getModel(), data);
    }
#endif

    double fallback_value(const Binding& binding) const noexcept
    {
        double value =
            binding.config.command_type
                == runtime::ActuatorCommandType::Position
            ? world->getData()->qpos[binding.qpos_address]
            : 0.0;
        if (!std::isfinite(value)) {
            value = 0.0;
        }
        return clamp_value(value, binding.config.limits);
    }

    double guarded_value(
        const Binding& binding,
        const runtime::ActuatorCommand* command,
        const bool duplicate,
        const std::uint64_t simulation_time_usec) const noexcept
    {
        if (command == nullptr
            || duplicate
            || command->type != binding.config.command_type
            || command->created_at_usec > simulation_time_usec
            || (command->expires_at_usec.has_value()
                && simulation_time_usec >= *command->expires_at_usec)
            || !std::isfinite(command->value)) {
            return fallback_value(binding);
        }
        return clamp_value(command->value, binding.config.limits);
    }

    std::shared_ptr<hako::robots::physics::impl::WorldImpl> world;
    std::vector<runtime::RuntimeInitialBodyPoseConfig> initial_body_poses;
    std::vector<Binding> bindings;
    std::vector<BodyBinding> body_bindings;
    std::unordered_map<std::string, std::size_t> binding_by_id;
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR
    std::vector<MirrorBinding> mirror_bindings;
    std::unordered_map<std::string, std::size_t> mirror_binding_by_id;
#endif
    std::uint64_t delta_time_usec {0};
};

MujocoActuatorPlant::MujocoActuatorPlant(
    const runtime::RuntimeDefinition& definition)
    : impl_(std::make_unique<Impl>(definition))
{
}

MujocoActuatorPlant::~MujocoActuatorPlant() = default;
MujocoActuatorPlant::MujocoActuatorPlant(MujocoActuatorPlant&&) noexcept = default;
MujocoActuatorPlant& MujocoActuatorPlant::operator=(
    MujocoActuatorPlant&&) noexcept = default;

std::uint64_t MujocoActuatorPlant::delta_time_usec() const noexcept
{
    return impl_->delta_time_usec;
}

runtime::RobotState MujocoActuatorPlant::read_state() const
{
    return impl_->state();
}

runtime::RobotState MujocoActuatorPlant::step(
    const std::vector<runtime::ActuatorCommand>& commands)
{
    const auto simulation_time_usec = model_time_usec(
        impl_->world->getData());

    std::unordered_map<
        std::string_view,
        const runtime::ActuatorCommand*> first_commands;
    std::unordered_set<std::string_view> duplicate_ids;
    first_commands.reserve(commands.size());
    duplicate_ids.reserve(commands.size());
    for (const auto& command : commands) {
        if (!impl_->binding_by_id.contains(command.actuator_id)) {
            continue;
        }
        if (!first_commands.emplace(
                command.actuator_id,
                &command).second) {
            duplicate_ids.emplace(command.actuator_id);
        }
    }

    std::vector<double> safe_values;
    safe_values.reserve(impl_->bindings.size());
    for (const auto& binding : impl_->bindings) {
        const auto command = first_commands.find(
            binding.config.component_id);
        safe_values.push_back(impl_->guarded_value(
            binding,
            command != first_commands.end() ? command->second : nullptr,
            duplicate_ids.contains(binding.config.component_id),
            simulation_time_usec));
    }

    for (std::size_t index = 0;
         index < impl_->bindings.size();
         ++index) {
        impl_->bindings[index].actuator->SetTarget(safe_values[index]);
    }
    impl_->world->advanceTimeStep();
    return impl_->state();
}

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR
runtime::RobotState MujocoActuatorPlant::step(
    const std::vector<runtime::ActuatorCommand>& commands,
    const std::vector<runtime::MirrorBodyCommand>& mirror_commands)
{
    const auto simulation_time_usec = model_time_usec(
        impl_->world->getData());
    impl_->apply_mirror_commands(mirror_commands, simulation_time_usec);
    return step(commands);
}
#endif

runtime::RobotState MujocoActuatorPlant::reset(
    const std::uint64_t simulation_time_usec)
{
    if (simulation_time_usec % impl_->delta_time_usec != 0) {
        throw std::invalid_argument(
            "MuJoCo reset time must align to the model timestep");
    }
    auto* data = impl_->world->getData();
    mj_resetData(impl_->world->getModel(), data);
    impl_->apply_initial_body_poses();
    data->time =
        static_cast<double>(simulation_time_usec) / 1'000'000.0;
    mj_forward(impl_->world->getModel(), data);
    return impl_->state();
}

mjModel* MujocoActuatorPlant::model() noexcept
{
    return impl_->world->getModel();
}

mjData* MujocoActuatorPlant::data() noexcept
{
    return impl_->world->getData();
}

} // namespace hakoniwa::robot_runtime::adapters::mujoco
