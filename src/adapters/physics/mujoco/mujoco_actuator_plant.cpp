#include "hakoniwa/robot_runtime/adapters/physics/mujoco/mujoco_actuator_plant.hpp"

#include "physics/physics_impl.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <tuple>
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

double quaternion_yaw(const mjtNum* quaternion) noexcept
{
    const double w = quaternion[0];
    const double x = quaternion[1];
    const double y = quaternion[2];
    const double z = quaternion[3];
    return std::atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z));
}

std::array<double, 4> quaternion_multiply(
    const std::array<double, 4>& a,
    const std::array<double, 4>& b) noexcept
{
    return {
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    };
}

runtime::Vector3State quaternion_rotate(
    const std::array<double, 4>& quaternion,
    const runtime::Vector3State& vector) noexcept
{
    const auto rotated = quaternion_multiply(
        quaternion_multiply(
            quaternion, {0.0, vector.x, vector.y, vector.z}),
        {quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]});
    return {rotated[1], rotated[2], rotated[3]};
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

#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
    struct PersonVehicleBinding {
        std::string name;
        int body_id {-1};
        int qvel_address {-1};
        std::unordered_map<std::string, runtime::Vector3State> seats;
    };

    struct PersonRideBinding {
        std::size_t vehicle_index {0};
        runtime::Vector3State offset;
    };

    struct PersonBinding {
        std::string name;
        int body_id {-1};
        int slide_x_qpos {-1};
        int slide_y_qpos {-1};
        int slide_z_qpos {-1};
        int turn_qpos {-1};
        int slide_x_qvel {-1};
        int slide_y_qvel {-1};
        int slide_z_qvel {-1};
        int turn_qvel {-1};
        int move_x_actuator {-1};
        int move_y_actuator {-1};
        int lift_actuator {-1};
        int turn_actuator {-1};
        int collision_geom {-1};
        int standing_contype {1};
        int standing_conaffinity {1};
        int hip_body_id {-1};
        double hip_height_m {0.0};
        double origin_x_m {0.0};
        double origin_y_m {0.0};
        double initial_yaw_rad {0.0};
        std::array<int, adapters::hakoniwa::person_limb_joint_names.size()>
            limb_qpos {};
        adapters::hakoniwa::PersonStepCommand command;
        std::array<double, adapters::hakoniwa::person_limb_joint_names.size()>
            gait_angles {};
        double gait_phase {0.0};
        double gait_clock {0.0};
        double posture_pitch_rad {0.0};
        double posture_roll_rad {0.0};
        std::optional<double> sit_ground;
        std::optional<PersonRideBinding> requested_ride;
        std::optional<PersonRideBinding> riding;
    };

    using PersonContactKey = std::tuple<std::string, std::string, std::string>;

    struct ActivePersonContact {
        runtime::PersonContactEvent started;
        double last_seen_sec {0.0};
    };

    struct ContactOwner {
        std::string kind {"static"};
        std::string name {"static"};
        std::optional<std::size_t> person_index;
        std::optional<std::size_t> vehicle_index;
    };
#endif

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
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

    struct GeomFrictionBinding {
        int geom_id {-1};
        /** Model friction before any directive; restored by reset(). */
        std::array<mjtNum, 3> original_friction {};
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
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
        person_vehicles.reserve(definition.person_vehicles.size());
        for (const auto& config : definition.person_vehicles) {
            if (person_vehicle_by_name.contains(config.name)) {
                throw std::invalid_argument(
                    "duplicate person vehicle: " + config.name);
            }
            const int joint_id = require_named_id(
                model, mjOBJ_JOINT, config.mjcf_freejoint,
                "person vehicle freejoint");
            if (model->jnt_type[joint_id] != mjJNT_FREE) {
                throw std::invalid_argument(
                    "person vehicle binding is not a freejoint: "
                    + config.mjcf_freejoint);
            }
            PersonVehicleBinding vehicle;
            vehicle.name = config.name;
            vehicle.body_id = model->jnt_bodyid[joint_id];
            vehicle.qvel_address = model->jnt_dofadr[joint_id];
            for (const auto& seat : config.seats) {
                if (!vehicle.seats.emplace(seat.name, seat.offset).second) {
                    throw std::invalid_argument(
                        "duplicate person vehicle seat: "
                        + config.name + "/" + seat.name);
                }
            }
            person_vehicle_by_name.emplace(vehicle.name, person_vehicles.size());
            person_vehicles.push_back(std::move(vehicle));
        }
#endif
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
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
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
        bind_geom_frictions(definition.geom_frictions);
#endif
    }

#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
    static std::string person_model_name(
        const mjModel* model,
        const std::string& person,
        const std::string_view suffix,
        const mjtObj object_type)
    {
        const std::array<std::string, 3> candidates {
            person + "/" + std::string(suffix),
            person + "_" + std::string(suffix),
            std::string(suffix),
        };
        for (const auto& candidate : candidates) {
            if (mj_name2id(model, object_type, candidate.c_str()) >= 0) {
                return candidate;
            }
        }
        throw std::invalid_argument(
            "person " + person + " is missing MuJoCo object "
            + std::string(suffix));
    }

    void bind_person(std::string person_name, const double initial_yaw_rad)
    {
        if (person_name.empty()) {
            throw std::invalid_argument("person name must not be empty");
        }
        if (person_binding_by_name.contains(person_name)) {
            throw std::invalid_argument("duplicate PersonStep person: " + person_name);
        }
        auto* model = world->getModel();
        if (!person_terrain_groups_configured) {
            for (int geom = 0; geom < model->ngeom; ++geom) {
                if (model->geom_type[geom] == mjGEOM_HFIELD
                    || model->geom_type[geom] == mjGEOM_PLANE) {
                    model->geom_group[geom] = 4;
                }
            }
            person_terrain_groups_configured = true;
        }
        auto joint_id = [&](const std::string_view suffix) {
            return require_named_id(model, mjOBJ_JOINT,
                person_model_name(model, person_name, suffix, mjOBJ_JOINT),
                "person joint");
        };
        auto actuator_id = [&](const std::string_view suffix) {
            return require_named_id(model, mjOBJ_ACTUATOR,
                person_model_name(model, person_name, suffix, mjOBJ_ACTUATOR),
                "person actuator");
        };
        const int slide_x = joint_id("slide_x_joint");
        const int slide_y = joint_id("slide_y_joint");
        const int slide_z = joint_id("slide_z_joint");
        const int turn = joint_id("turn_joint");
        PersonBinding binding;
        binding.name = person_name;
        if (!std::isfinite(initial_yaw_rad)) {
            throw std::invalid_argument(
                "person initial yaw must be finite: " + person_name);
        }
        binding.initial_yaw_rad = initial_yaw_rad;
        binding.body_id = require_named_id(model, mjOBJ_BODY,
            person_model_name(model, binding.name, "person", mjOBJ_BODY),
            "person body");
        binding.slide_x_qpos = model->jnt_qposadr[slide_x];
        binding.slide_y_qpos = model->jnt_qposadr[slide_y];
        binding.slide_z_qpos = model->jnt_qposadr[slide_z];
        binding.turn_qpos = model->jnt_qposadr[turn];
        binding.slide_x_qvel = model->jnt_dofadr[slide_x];
        binding.slide_y_qvel = model->jnt_dofadr[slide_y];
        binding.slide_z_qvel = model->jnt_dofadr[slide_z];
        binding.turn_qvel = model->jnt_dofadr[turn];
        binding.move_x_actuator = actuator_id("move_x");
        binding.move_y_actuator = actuator_id("move_y");
        binding.lift_actuator = actuator_id("lift");
        binding.turn_actuator = actuator_id("turn");
        binding.collision_geom = require_named_id(
            model, mjOBJ_GEOM,
            person_model_name(model, binding.name, "person_collision", mjOBJ_GEOM),
            "person collision geom");
        binding.standing_contype = model->geom_contype[binding.collision_geom];
        binding.standing_conaffinity =
            model->geom_conaffinity[binding.collision_geom];
        for (std::size_t index = 0; index < binding.limb_qpos.size(); ++index) {
            const int limb = joint_id(
                adapters::hakoniwa::person_limb_joint_names[index]);
            binding.limb_qpos[index] = model->jnt_qposadr[limb];
            if (adapters::hakoniwa::person_limb_joint_names[index]
                == "hip_left_joint") {
                binding.hip_body_id = model->jnt_bodyid[limb];
            }
        }
        world->getData()->qpos[binding.turn_qpos] = binding.initial_yaw_rad;
        mj_forward(model, world->getData());
        if (binding.hip_body_id < 0) {
            throw std::invalid_argument(
                "person " + binding.name + " is missing its hip body");
        }
        const auto* data = world->getData();
        binding.origin_x_m = data->xpos[3 * binding.body_id]
            - data->qpos[binding.slide_x_qpos];
        binding.origin_y_m = data->xpos[3 * binding.body_id + 1]
            - data->qpos[binding.slide_y_qpos];
        binding.hip_height_m =
            data->xpos[3 * binding.hip_body_id + 2]
            - data->xpos[3 * binding.body_id + 2];
        if (!std::isfinite(binding.hip_height_m)
            || binding.hip_height_m <= 0.0) {
            throw std::invalid_argument(
                "person " + binding.name
                + " has an invalid hip height in its MuJoCo model");
        }
        person_binding_by_name.emplace(binding.name, person_bindings.size());
        person_bindings.push_back(std::move(binding));
        auto& added = person_bindings.back();
        const double ground = ground_under(added);
        world->getData()->qpos[added.slide_z_qpos] = ground;
        world->getData()->ctrl[added.lift_actuator] = ground;
        mj_forward(model, world->getData());
    }

    double ray_down(
        const double x,
        const double y,
        const double z,
        const int body_exclude,
        const bool terrain_only) const
    {
        mjtNum point[3] {x, y, z};
        mjtNum direction[3] {0.0, 0.0, -1.0};
        mjtByte groups[mjNGROUP] {};
        if (terrain_only) {
            groups[4] = 1;
        } else {
            groups[0] = 1;
            groups[4] = 1;
        }
        int geom_id = -1;
        mjtNum normal[3] {};
        const mjtNum distance = mj_ray(
            world->getModel(), world->getData(), point, direction,
            groups, 1, body_exclude, &geom_id, normal);
        return distance >= 0.0
            ? z - static_cast<double>(distance)
            : std::numeric_limits<double>::quiet_NaN();
    }

    double ground_under(const PersonBinding& person) const
    {
        const auto* data = world->getData();
        const double x = data->xpos[3 * person.body_id];
        const double y = data->xpos[3 * person.body_id + 1];
        const double z = data->xpos[3 * person.body_id + 2];
        double terrain = ray_down(x, y, 10'000.0, -1, true);
        if (!std::isfinite(terrain)) terrain = 0.0;
        double hit = ray_down(
            x, y, std::max(z, terrain) + 0.37, person.body_id, false);
        if (!std::isfinite(hit)) hit = terrain;
        return std::max(terrain, hit);
    }

    PersonBinding& person(const std::string& name)
    {
        const auto found = person_binding_by_name.find(name);
        if (found == person_binding_by_name.end()) {
            throw std::invalid_argument("PersonStep person is not bound: " + name);
        }
        return person_bindings[found->second];
    }

    const PersonBinding& person(const std::string& name) const
    {
        const auto found = person_binding_by_name.find(name);
        if (found == person_binding_by_name.end()) {
            throw std::invalid_argument("PersonStep person is not bound: " + name);
        }
        return person_bindings[found->second];
    }

    std::uint32_t apply_person_command(
        const std::string& name,
        const adapters::hakoniwa::PersonStepCommand& command)
    {
        auto& binding = person(name);
        const auto finite = [](const double value) {
            return std::isfinite(value);
        };
        if (!finite(command.linear_velocity.x)
            || !finite(command.linear_velocity.y)
            || !finite(command.linear_velocity.z)
            || !finite(command.angular_velocity.x)
            || !finite(command.angular_velocity.y)
            || !finite(command.angular_velocity.z)) {
            return 2;
        }
        std::optional<PersonRideBinding> requested_ride;
        if (!command.ride.empty()) {
            const auto separator = command.ride.find('/');
            const std::string vehicle_name = command.ride.substr(0, separator);
            const std::string seat_name = separator == std::string::npos
                || separator + 1 == command.ride.size()
                ? "driver"
                : command.ride.substr(separator + 1);
            const auto vehicle = person_vehicle_by_name.find(vehicle_name);
            if (vehicle == person_vehicle_by_name.end()
                || seat_name.find('/') != std::string::npos) {
                return 1;
            }
            const auto seat = person_vehicles[vehicle->second].seats.find(seat_name);
            if (seat == person_vehicles[vehicle->second].seats.end()) {
                return 1;
            }
            requested_ride = PersonRideBinding {
                vehicle->second, seat->second,
            };
        }
        binding.command = command;
        binding.requested_ride = std::move(requested_ride);
        static const std::unordered_set<std::string> animations {
            "auto", "walk", "idle", "wave", "talk", "sit", "sit_talk",
            "sit_drive", "sit_drowsy", "sit_alert", "sit_startled",
        };
        if (!animations.contains(binding.command.animation)) {
            binding.command.animation = "auto";
        }
        return 0;
    }

    static bool person_is_sitting(const std::string& animation) noexcept
    {
        return animation == "sit"
            || animation == "sit_talk"
            || animation == "sit_drive"
            || animation == "sit_drowsy"
            || animation == "sit_alert"
            || animation == "sit_startled";
    }

    void apply_person_controls()
    {
        auto* data = world->getData();
        for (auto& binding : person_bindings) {
            auto* model = world->getModel();
            if (binding.riding.has_value() || binding.requested_ride.has_value()) {
                data->ctrl[binding.move_x_actuator] = 0.0;
                data->ctrl[binding.move_y_actuator] = 0.0;
                data->ctrl[binding.turn_actuator] = 0.0;
                model->geom_contype[binding.collision_geom] = 0;
                model->geom_conaffinity[binding.collision_geom] = 0;
                binding.sit_ground.reset();
                continue;
            }
            const double yaw = data->qpos[binding.turn_qpos];
            const double c = std::cos(yaw);
            const double s = std::sin(yaw);
            const double body_x = binding.command.linear_velocity.x;
            const double body_y = binding.command.linear_velocity.y;
            data->ctrl[binding.move_x_actuator] = c * body_x - s * body_y;
            data->ctrl[binding.move_y_actuator] = s * body_x + c * body_y;
            data->ctrl[binding.turn_actuator] =
                std::clamp(binding.command.angular_velocity.z, -3.0, 3.0);
            const bool sitting = person_is_sitting(binding.command.animation);
            double ground = 0.0;
            if (sitting && binding.sit_ground.has_value()) {
                ground = *binding.sit_ground;
            } else {
                ground = ground_under(binding);
            }
            binding.sit_ground = sitting
                ? std::optional<double> {ground}
                : std::nullopt;
            constexpr double seat_m = 0.44;
            data->ctrl[binding.lift_actuator] =
                sitting ? ground - (binding.hip_height_m - seat_m) : ground;
            model->geom_contype[binding.collision_geom] =
                sitting ? 0 : binding.standing_contype;
            model->geom_conaffinity[binding.collision_geom] =
                sitting ? 0 : binding.standing_conaffinity;
        }
    }

    void place_person(
        PersonBinding& person,
        const double x,
        const double y,
        const double z,
        const double yaw)
    {
        auto* data = world->getData();
        data->qpos[person.slide_x_qpos] = x - person.origin_x_m;
        data->qpos[person.slide_y_qpos] = y - person.origin_y_m;
        data->qpos[person.slide_z_qpos] = z;
        data->qpos[person.turn_qpos] = yaw;
        data->ctrl[person.lift_actuator] = z;
        data->qvel[person.slide_x_qvel] = 0.0;
        data->qvel[person.slide_y_qvel] = 0.0;
        data->qvel[person.slide_z_qvel] = 0.0;
        data->qvel[person.turn_qvel] = 0.0;
    }

    void update_person_rides()
    {
        constexpr double get_off_m = 0.75;
        auto* model = world->getModel();
        auto* data = world->getData();
        bool changed = false;
        for (auto& person : person_bindings) {
            if (person.requested_ride.has_value()) {
                person.riding = person.requested_ride;
                const auto& ride = *person.riding;
                const auto& vehicle = person_vehicles[ride.vehicle_index];
                const mjtNum* position = &data->xpos[3 * vehicle.body_id];
                const mjtNum* quaternion = &data->xquat[4 * vehicle.body_id];
                const double yaw = quaternion_yaw(quaternion);
                const double c = std::cos(yaw);
                const double s = std::sin(yaw);
                place_person(
                    person,
                    position[0] + c * ride.offset.x - s * ride.offset.y,
                    position[1] + s * ride.offset.x + c * ride.offset.y,
                    position[2] + ride.offset.z - person.hip_height_m,
                    yaw);
                model->geom_contype[person.collision_geom] = 0;
                model->geom_conaffinity[person.collision_geom] = 0;
                changed = true;
                continue;
            }
            if (!person.riding.has_value()) {
                continue;
            }
            const auto ride = *person.riding;
            const auto& vehicle = person_vehicles[ride.vehicle_index];
            const mjtNum* position = &data->xpos[3 * vehicle.body_id];
            const mjtNum* quaternion = &data->xquat[4 * vehicle.body_id];
            const double yaw = quaternion_yaw(quaternion);
            const double c = std::cos(yaw);
            const double s = std::sin(yaw);
            const double side = get_off_m + std::fabs(ride.offset.y);
            const double outside = std::copysign(
                side, ride.offset.y == 0.0 ? 1.0 : ride.offset.y);
            place_person(
                person,
                position[0] + c * ride.offset.x - s * outside,
                position[1] + s * ride.offset.x + c * outside,
                0.0,
                yaw);
            person.riding.reset();
            mj_forward(model, data);
            const double ground = ground_under(person);
            data->qpos[person.slide_z_qpos] = ground;
            data->ctrl[person.lift_actuator] = ground;
            model->geom_contype[person.collision_geom] = person.standing_contype;
            model->geom_conaffinity[person.collision_geom] =
                person.standing_conaffinity;
            changed = true;
        }
        if (changed) {
            mj_forward(model, data);
        }
    }

    bool has_person_ride_activity() const noexcept
    {
        return std::any_of(
            person_bindings.begin(), person_bindings.end(),
            [](const PersonBinding& person) {
                return person.riding.has_value()
                    || person.requested_ride.has_value();
            });
    }

    bool body_descends_from(int body, const int root) const noexcept
    {
        const auto* model = world->getModel();
        while (body > 0) {
            if (body == root) return true;
            body = model->body_parentid[body];
        }
        return root == 0;
    }

    ContactOwner contact_owner(const int geom_id) const
    {
        const auto* model = world->getModel();
        if (model->geom_type[geom_id] == mjGEOM_PLANE
            || model->geom_type[geom_id] == mjGEOM_HFIELD) {
            return {"terrain", "terrain", std::nullopt, std::nullopt};
        }
        const int body = model->geom_bodyid[geom_id];
        for (std::size_t index = 0; index < person_bindings.size(); ++index) {
            if (body_descends_from(body, person_bindings[index].body_id)) {
                return {"person", person_bindings[index].name, index, std::nullopt};
            }
        }
        for (std::size_t index = 0; index < person_vehicles.size(); ++index) {
            if (body_descends_from(body, person_vehicles[index].body_id)) {
                return {"vehicle", person_vehicles[index].name, std::nullopt, index};
            }
        }
        return {};
    }

    runtime::Vector3State contact_owner_velocity(
        const ContactOwner& owner) const
    {
        if (owner.person_index.has_value()) {
            return person_contact_velocities[*owner.person_index];
        }
        if (owner.vehicle_index.has_value()) {
            return vehicle_contact_velocities[*owner.vehicle_index];
        }
        return {};
    }

    void capture_person_contact_velocities()
    {
        const auto* data = world->getData();
        person_contact_velocities.clear();
        person_contact_velocities.reserve(person_bindings.size());
        for (const auto& person : person_bindings) {
            person_contact_velocities.push_back({
                data->qvel[person.slide_x_qvel],
                data->qvel[person.slide_y_qvel],
                data->qvel[person.slide_z_qvel],
            });
        }
        vehicle_contact_velocities.clear();
        vehicle_contact_velocities.reserve(person_vehicles.size());
        for (const auto& vehicle : person_vehicles) {
            vehicle_contact_velocities.push_back({
                data->qvel[vehicle.qvel_address],
                data->qvel[vehicle.qvel_address + 1],
                data->qvel[vehicle.qvel_address + 2],
            });
        }
    }

    static double rounded(const double value, const double scale) noexcept
    {
        return std::round(value * scale) / scale;
    }

    runtime::PersonContactEvent make_person_contact_event(
        const PersonContactKey& key,
        const runtime::Vector3State& position_mjcf,
        const double relative_speed,
        const bool started)
    {
        const auto& [self, kind, other] = key;
        runtime::PersonContactEvent event;
        event.id = next_person_contact_id++;
        event.time_sec = rounded(world->getData()->time, 10'000.0);
        event.self_name = self;
        event.other_name = other;
        event.other_kind = kind;
        event.position = {
            rounded(-position_mjcf.y, 1'000.0),
            rounded(position_mjcf.x, 1'000.0),
            rounded(position_mjcf.z, 1'000.0),
        };
        event.relative_speed_mps = rounded(relative_speed, 1'000.0);
        event.started = started;
        pending_person_contacts.push_back(event);
        return event;
    }

    void detect_person_contacts()
    {
        constexpr double contact_gap_sec = 0.3;
        auto* data = world->getData();
        std::map<PersonContactKey, runtime::Vector3State> seen;
        for (int index = 0; index < data->ncon; ++index) {
            const auto& contact = data->contact[index];
            const std::array<ContactOwner, 2> owners {
                contact_owner(contact.geom1), contact_owner(contact.geom2),
            };
            for (int mine_index = 0; mine_index < 2; ++mine_index) {
                const auto& mine = owners[mine_index];
                const auto& other = owners[1 - mine_index];
                if (mine.kind != "person" || !mine.person_index.has_value()) {
                    continue;
                }
                // Supporting terrain is not an impact event. Static city
                // parts such as walls, decks and boxes remain observable.
                if (other.kind == "terrain") {
                    continue;
                }
                const auto& person = person_bindings[*mine.person_index];
                if (other.kind == "vehicle"
                    && other.vehicle_index.has_value()
                    && person.riding.has_value()
                    && person.riding->vehicle_index == *other.vehicle_index) {
                    continue;
                }
                const PersonContactKey key {
                    mine.name, other.kind, other.name,
                };
                seen.try_emplace(key, runtime::Vector3State {
                    contact.pos[0], contact.pos[1], contact.pos[2],
                });
            }
        }

        const double now = data->time;
        for (const auto& [key, position] : seen) {
            const auto active = active_person_contacts.find(key);
            if (active != active_person_contacts.end()) {
                active->second.last_seen_sec = now;
                continue;
            }
            const auto self_index = person_binding_by_name.at(std::get<0>(key));
            const auto& self = person_bindings[self_index];
            const ContactOwner self_owner {
                "person", self.name, self_index, std::nullopt,
            };
            const ContactOwner other_owner = [&]() {
                if (std::get<1>(key) == "person") {
                    const auto index = person_binding_by_name.at(std::get<2>(key));
                    return ContactOwner {
                        "person", std::get<2>(key), index, std::nullopt,
                    };
                }
                if (std::get<1>(key) == "vehicle") {
                    const auto index = person_vehicle_by_name.at(std::get<2>(key));
                    return ContactOwner {
                        "vehicle", std::get<2>(key), std::nullopt, index,
                    };
                }
                return ContactOwner {};
            }();
            const auto self_velocity = contact_owner_velocity(self_owner);
            const auto other_velocity = contact_owner_velocity(other_owner);
            const double relative_speed = std::hypot(
                self_velocity.x - other_velocity.x,
                self_velocity.y - other_velocity.y);
            auto event = make_person_contact_event(
                key, position, relative_speed, true);
            active_person_contacts.emplace(
                key, ActivePersonContact {std::move(event), now});
        }

        for (auto active = active_person_contacts.begin();
             active != active_person_contacts.end();) {
            if (seen.contains(active->first)
                || now - active->second.last_seen_sec <= contact_gap_sec) {
                ++active;
                continue;
            }
            const auto& position = active->second.started.position;
            const runtime::Vector3State position_mjcf {
                position.y, -position.x, position.z,
            };
            make_person_contact_event(
                active->first, position_mjcf, 0.0, false);
            active = active_person_contacts.erase(active);
        }
    }

    std::vector<runtime::PersonContactEvent> take_person_contact_events()
    {
        auto result = std::move(pending_person_contacts);
        pending_person_contacts.clear();
        return result;
    }

    static double approach(
        const double current,
        const double target,
        const double maximum_step) noexcept
    {
        const double difference = target - current;
        if (std::fabs(difference) <= maximum_step) return target;
        return current + std::copysign(maximum_step, difference);
    }

    void update_person_gaits()
    {
        constexpr double pi = 3.14159265358979323846;
        constexpr double moving_m_s = 0.05;
        constexpr double stride_m = 1.0;
        constexpr double max_swing_rad = pi / 6.0;
        const double dt = static_cast<double>(delta_time_usec) / 1'000'000.0;
        auto* data = world->getData();
        for (auto& binding : person_bindings) {
            const double vx = data->qvel[binding.slide_x_qvel];
            const double vy = data->qvel[binding.slide_y_qvel];
            const double speed = std::hypot(vx, vy);
            binding.gait_clock += dt;
            std::string mode = binding.command.animation;
            if (binding.riding.has_value() && !person_is_sitting(mode)) {
                mode = "sit";
            }
            if (mode.empty() || mode == "auto") {
                mode = speed > moving_m_s ? "walk" : "idle";
            }
            std::array<double, adapters::hakoniwa::person_limb_joint_names.size()>
                targets {};
            if (mode == "walk") {
                const double gait_speed = std::max(speed, 0.6);
                binding.gait_phase = std::fmod(
                    binding.gait_phase + 2.0 * pi * gait_speed * dt / stride_m,
                    2.0 * pi);
                const double swing = max_swing_rad
                    * std::min(1.0, gait_speed / 1.4);
                const double phase = std::sin(binding.gait_phase);
                targets = {
                    -swing * phase,
                    swing * phase,
                    swing * phase,
                    -swing * phase,
                    1.3 * swing * std::max(0.0, phase),
                    1.3 * swing * std::max(0.0, -phase),
                };
                binding.gait_angles = targets;
            } else {
                if (mode == "idle") {
                    const double sway = (pi / 90.0)
                        * std::sin(binding.gait_clock * 1.3);
                    targets[0] = sway;
                    targets[1] = -sway;
                } else if (mode == "wave") {
                    targets[1] = -5.0 * pi / 6.0
                        + (pi / 10.0)
                            * std::sin(binding.gait_clock * 2.0 * pi * 1.5);
                } else if (person_is_sitting(mode)) {
                    targets[0] = -5.0 * pi / 36.0;
                    targets[1] = -5.0 * pi / 36.0;
                    targets[2] = -pi / 2.0;
                    targets[3] = -pi / 2.0;
                    targets[4] = pi / 2.0;
                    targets[5] = pi / 2.0;
                }
                if (mode == "sit_drive" || mode == "sit_alert") {
                    targets[0] = -13.0 * pi / 36.0;
                    targets[1] = -13.0 * pi / 36.0;
                } else if (mode == "sit_drowsy") {
                    const double drift = (pi / 45.0)
                        * std::sin(binding.gait_clock * 0.7);
                    targets[0] = -pi / 12.0 + drift;
                    targets[1] = -pi / 12.0 - drift;
                } else if (mode == "sit_startled") {
                    targets[0] = -17.0 * pi / 36.0;
                    targets[1] = -17.0 * pi / 36.0;
                }
                if (mode == "talk" || mode == "sit_talk") {
                    targets[1] = -7.0 * pi / 36.0
                        + (7.0 * pi / 90.0)
                            * std::sin(binding.gait_clock * 2.3);
                    targets[0] = -pi / 9.0
                        + (pi / 20.0)
                            * std::sin(binding.gait_clock * 1.7 + 1.0);
                }
                const double maximum_step = (4.0 * pi / 3.0) * dt;
                for (std::size_t index = 0;
                     index < targets.size(); ++index) {
                    binding.gait_angles[index] = approach(
                        binding.gait_angles[index], targets[index], maximum_step);
                }
            }
            for (std::size_t index = 0;
                 index < binding.limb_qpos.size(); ++index) {
                data->qpos[binding.limb_qpos[index]] =
                    binding.gait_angles[index];
            }
            double target_pitch = 0.0;
            double target_roll = 0.0;
            if (binding.riding.has_value()) {
                if (mode == "sit_drowsy") {
                    target_pitch = 22.0 * pi / 180.0
                        + 6.0 * pi / 180.0
                            * std::sin(binding.gait_clock * 0.9);
                    target_roll = 5.0 * pi / 180.0
                        * std::sin(binding.gait_clock * 0.6);
                } else if (mode == "sit_startled") {
                    target_pitch = -8.0 * pi / 180.0;
                } else if (mode == "sit_drive") {
                    target_pitch = 4.0 * pi / 180.0;
                }
            }
            const double posture_step = 90.0 * pi / 180.0 * dt;
            binding.posture_pitch_rad = approach(
                binding.posture_pitch_rad, target_pitch, posture_step);
            binding.posture_roll_rad = approach(
                binding.posture_roll_rad, target_roll, posture_step);
        }
        if (!person_bindings.empty()) {
            mj_forward(world->getModel(), data);
        }
    }

    adapters::hakoniwa::PersonStepState person_state(
        const std::string& name,
        const std::uint32_t result_code) const
    {
        const auto& binding = person(name);
        const auto* data = world->getData();
        const mjtNum* actual_position = &data->xpos[3 * binding.body_id];
        const mjtNum* actual_quaternion = &data->xquat[4 * binding.body_id];
        runtime::Vector3State position {
            actual_position[0], actual_position[1], actual_position[2],
        };
        std::array<double, 4> quaternion {
            actual_quaternion[0], actual_quaternion[1],
            actual_quaternion[2], actual_quaternion[3],
        };
        if (binding.riding.has_value()
            && (binding.posture_pitch_rad != 0.0
                || binding.posture_roll_rad != 0.0)) {
            const auto pitch = std::array<double, 4> {
                std::cos(binding.posture_pitch_rad / 2.0), 0.0,
                std::sin(binding.posture_pitch_rad / 2.0), 0.0,
            };
            const auto roll = std::array<double, 4> {
                std::cos(binding.posture_roll_rad / 2.0),
                std::sin(binding.posture_roll_rad / 2.0), 0.0, 0.0,
            };
            const auto tilted = quaternion_multiply(
                quaternion, quaternion_multiply(pitch, roll));
            const runtime::Vector3State hip {0.0, 0.0, binding.hip_height_m};
            const auto before = quaternion_rotate(quaternion, hip);
            const auto after = quaternion_rotate(tilted, hip);
            position.x += before.x - after.x;
            position.y += before.y - after.y;
            position.z += before.z - after.z;
            quaternion = tilted;
        }
        return {
            result_code,
            model_time_usec(data),
            position,
            {quaternion[1], quaternion[2], quaternion[3], quaternion[0]},
            {data->qvel[binding.slide_x_qvel],
                data->qvel[binding.slide_y_qvel], 0.0},
            {0.0, 0.0, data->qvel[binding.turn_qvel]},
            binding.gait_angles,
        };
    }

    void reset_people()
    {
        auto* data = world->getData();
        for (auto& binding : person_bindings) {
            binding.command = {};
            binding.gait_angles.fill(0.0);
            binding.gait_phase = 0.0;
            binding.gait_clock = 0.0;
            binding.posture_pitch_rad = 0.0;
            binding.posture_roll_rad = 0.0;
            binding.sit_ground.reset();
            binding.requested_ride.reset();
            binding.riding.reset();
            data->qpos[binding.turn_qpos] = binding.initial_yaw_rad;
            const double ground = ground_under(binding);
            data->qpos[binding.slide_z_qpos] = ground;
            data->ctrl[binding.lift_actuator] = ground;
            auto* model = world->getModel();
            model->geom_contype[binding.collision_geom] =
                binding.standing_contype;
            model->geom_conaffinity[binding.collision_geom] =
                binding.standing_conaffinity;
            for (std::size_t index = 0;
                 index < binding.limb_qpos.size(); ++index) {
                data->qpos[binding.limb_qpos[index]] = 0.0;
            }
        }
        if (!person_bindings.empty()) {
            mj_forward(world->getModel(), data);
        }
        active_person_contacts.clear();
        pending_person_contacts.clear();
        next_person_contact_id = 1;
    }
#endif

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
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
        append_mirror_contacts(state);
#endif
        return state;
    }

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
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

    /**
     * Resolves geom names once at binding and remembers their original
     * friction for reset().
     *
     * The Plant does not touch geom_priority. MuJoCo uses the friction of the
     * higher-priority geom of a contact pair (element-wise maximum for equal
     * priority), so the model / asset composer decides whether these geoms'
     * friction wins, e.g. by giving tires a higher priority than the World.
     */
    void bind_geom_frictions(
        const std::vector<runtime::RuntimeGeomFrictionConfig>& configs)
    {
        auto* model = world->getModel();
        for (const auto& config : configs) {
            for (const auto& name : config.geoms) {
                if (geom_friction_binding_by_name.contains(name)) {
                    throw std::invalid_argument(
                        "duplicate geom friction binding: " + name);
                }
                const int geom_id = require_named_id(
                    model, mjOBJ_GEOM, name, "geom friction geom");
                GeomFrictionBinding binding;
                binding.geom_id = geom_id;
                for (int index = 0; index < 3; ++index) {
                    binding.original_friction[index] =
                        model->geom_friction[3 * geom_id + index];
                }
                geom_friction_binding_by_name.emplace(name, binding);
            }
        }
    }

    void restore_geom_frictions() noexcept
    {
        auto* model = world->getModel();
        for (const auto& [name, binding] : geom_friction_binding_by_name) {
            (void)name;
            for (int index = 0; index < 3; ++index) {
                model->geom_friction[3 * binding.geom_id + index] =
                    binding.original_friction[index];
            }
        }
    }

    /**
     * Sets the sliding friction (geom_friction[3 * id]) of each bound geom.
     * Torsional and rolling friction stay unchanged. Directives are applied in
     * list order, so a later directive for the same geom wins. The value
     * persists in the model until the next directive or reset().
     */
    void apply_geom_friction_directives(
        const std::vector<std::shared_ptr<const runtime::GeomFrictionDirective>>&
            directives,
        std::uint64_t simulation_time_usec)
    {
        auto* model = world->getModel();
        for (const auto& directive : directives) {
            const double friction = directive->sliding_friction();
            if (directive->created_at_usec() > simulation_time_usec
                || !std::isfinite(friction) || friction < 0.0) {
                continue;
            }
            for (const auto& name : directive->geom_names()) {
                const auto binding = geom_friction_binding_by_name.find(name);
                if (binding == geom_friction_binding_by_name.end()) {
                    continue;
                }
                model->geom_friction[3 * binding->second.geom_id] = friction;
            }
        }
    }

    void apply_mirror_directives(
        const std::vector<std::shared_ptr<const runtime::MirrorBodyDirective>>&
            directives,
        std::uint64_t simulation_time_usec)
    {
        // Mirror configurations run a forward pass every step after applying
        // poses, before physics advances. Without Mirror bindings there is
        // nothing to apply, so the forward pass is skipped.
        if (mirror_bindings.empty()) {
            return;
        }
        std::unordered_map<std::string_view, const runtime::MirrorBodyCommand*> first;
        std::unordered_set<std::string_view> duplicates;
        for (const auto& directive : directives) {
            const auto& command = directive->command();
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
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
    std::vector<PersonVehicleBinding> person_vehicles;
    std::unordered_map<std::string, std::size_t> person_vehicle_by_name;
    std::vector<PersonBinding> person_bindings;
    std::unordered_map<std::string, std::size_t> person_binding_by_name;
    std::map<PersonContactKey, ActivePersonContact> active_person_contacts;
    std::vector<runtime::PersonContactEvent> pending_person_contacts;
    std::vector<runtime::Vector3State> person_contact_velocities;
    std::vector<runtime::Vector3State> vehicle_contact_velocities;
    std::uint64_t next_person_contact_id {1};
    bool person_terrain_groups_configured {false};
#endif
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
    std::vector<MirrorBinding> mirror_bindings;
    std::unordered_map<std::string, std::size_t> mirror_binding_by_id;
    std::unordered_map<std::string, GeomFrictionBinding> geom_friction_binding_by_name;
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
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
    impl_->apply_person_controls();
    impl_->capture_person_contact_velocities();
#endif
    impl_->world->advanceTimeStep();
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
    impl_->detect_person_contacts();
    // mj_step leaves position-dependent fields at the state used for the
    // integration. Refresh them before placing riders on the just-integrated
    // vehicle pose returned for this step. Avoid the extra forward pass when
    // no ride is active or pending.
    if (impl_->has_person_ride_activity()) {
        mj_forward(impl_->world->getModel(), impl_->world->getData());
    }
    impl_->update_person_rides();
    impl_->update_person_gaits();
#endif
    return impl_->state();
}

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
void MujocoActuatorPlant::apply_directives(
    const runtime::PlantDirectiveList& directives)
{
    const auto simulation_time_usec = model_time_usec(
        impl_->world->getData());
    // Each applier takes only its own directive kind. Model parameters such as
    // friction are set first; the Mirror applier ends with mj_forward() so the
    // state handed to the physics step is consistent.
    impl_->apply_geom_friction_directives(
        runtime::directives_of<runtime::GeomFrictionDirective>(directives),
        simulation_time_usec);
    impl_->apply_mirror_directives(
        runtime::directives_of<runtime::MirrorBodyDirective>(directives),
        simulation_time_usec);
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
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
    // mj_resetData() does not touch mjModel, so directive-modified model
    // parameters are restored explicitly.
    impl_->restore_geom_frictions();
#endif
    mj_resetData(impl_->world->getModel(), data);
    impl_->apply_initial_body_poses();
    data->time =
        static_cast<double>(simulation_time_usec) / 1'000'000.0;
    mj_forward(impl_->world->getModel(), data);
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
    impl_->reset_people();
#endif
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

#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
void MujocoActuatorPlant::bind_person(
    std::string person_name,
    const double initial_yaw_rad)
{
    impl_->bind_person(std::move(person_name), initial_yaw_rad);
}

std::uint32_t MujocoActuatorPlant::apply_person_command(
    const std::string& person_name,
    const adapters::hakoniwa::PersonStepCommand& command)
{
    return impl_->apply_person_command(person_name, command);
}

adapters::hakoniwa::PersonStepState MujocoActuatorPlant::person_state(
    const std::string& person_name,
    const std::uint32_t result_code) const
{
    return impl_->person_state(person_name, result_code);
}

std::vector<runtime::PersonContactEvent>
MujocoActuatorPlant::take_person_contact_events()
{
    return impl_->take_person_contact_events();
}
#endif

} // namespace hakoniwa::robot_runtime::adapters::mujoco
