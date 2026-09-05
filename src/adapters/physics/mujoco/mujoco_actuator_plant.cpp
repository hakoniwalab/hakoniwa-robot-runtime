#include "hakoniwa/robot_runtime/adapters/physics/mujoco/mujoco_actuator_plant.hpp"

#include "physics/physics_impl.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
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
        return state;
    }

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
    std::vector<Binding> bindings;
    std::unordered_map<std::string, std::size_t> binding_by_id;
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

runtime::RobotState MujocoActuatorPlant::reset(
    const std::uint64_t simulation_time_usec)
{
    if (simulation_time_usec % impl_->delta_time_usec != 0) {
        throw std::invalid_argument(
            "MuJoCo reset time must align to the model timestep");
    }
    auto* data = impl_->world->getData();
    mj_resetData(impl_->world->getModel(), data);
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
