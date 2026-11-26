#include "hakoniwa/robot_runtime/factory/manifest_factory.hpp"

#include <algorithm>

#include "factory/manifest_factory_internal.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/float64_pdu_event_reader.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/joint_state_pdu_writer.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/joint_trajectory_pdu_event_reader.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/joy_pdu_event_reader.hpp"
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
#include "hakoniwa/robot_runtime/adapters/endpoint/impulse_collision_pdu_writer.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/mirror_body_twist_pdu_reader.hpp"
#endif
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MOBILE_BASE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MOBILE_BASE
#include "adapters/endpoint/robot_pdu_event_reader.hpp"
#include "plugins/ackermann_step/ackermann_step.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/multi_dof_joint_state_pdu_writer.hpp"
#endif
#include "hakoniwa/robot_runtime/adapters/endpoint/ackermann_drive_rpc_event_reader.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/person_step_rpc_event_reader.hpp"
#include "hakoniwa/robot_runtime/adapters/physics/mujoco/mujoco_actuator_plant.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hako_msgs/pdu_cpptype_ContactEvent.hpp"
#include "hako_msgs/pdu_cpptype_ContactEventArray.hpp"
#include "hako_msgs/pdu_cpptype_conv_ContactEvent.hpp"
#include "hako_msgs/pdu_cpptype_conv_ContactEventArray.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include <nlohmann/json.hpp>
#include "runner/runner_factory.hpp"
#include "runtime/runtime_factory.hpp"
#include "runtime/runtime_factory_manifest.hpp"
#include "runtime/service/builtin_robot_config.hpp"
#include "runner/hakoniwa/core_async_service_server.hpp"
#include "runtime/service/async_service_pipeline.hpp"
#include "runtime/service/robot_plugin_declaration.hpp"
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR
#include "adapters/physics/mujoco/mujoco_sensor_stage.hpp"
#include "runtime/publisher/sensor_joint_state_publisher.hpp"
#include "runtime/publisher/sensor_wrench_publisher.hpp"
#include "runtime/publisher/sensor_imu_publisher.hpp"
#include "runtime/publisher/sensor_laser_scan_publisher.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/laser_scan_pdu_writer.hpp"
#include "runtime/publisher/sensor_point_cloud_publisher.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/point_cloud2_pdu_writer.hpp"
#include "runtime/publisher/sensor_range_publisher.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/range_pdu_writer.hpp"
#include "runtime/publisher/sensor_depth_image_publisher.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/depth_image_pdu_writer.hpp"
#include "runtime/publisher/sensor_compressed_image_publisher.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/compressed_image_pdu_writer.hpp"
#include "runtime/publisher/sensor_magnetic_field_publisher.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/magnetic_field_pdu_writer.hpp"
#include "runtime/publisher/sensor_fluid_pressure_publisher.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/fluid_pressure_pdu_writer.hpp"
#include "runtime/publisher/sensor_nav_sat_fix_publisher.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/nav_sat_fix_pdu_writer.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/imu_pdu_writer.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/wrench_stamped_pdu_writer.hpp"
#include "runtime/sensor/sensor_registry.hpp"
#endif

#include <memory>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::factory {

namespace {

using ContactEndpoint = ::hakoniwa::pdu::TypedEndpoint<
    HakoCpp_ContactEvent,
    ::hako::pdu::msgs::hako_msgs::ContactEvent>;
using ContactArrayEndpoint = ::hakoniwa::pdu::TypedEndpoint<
    HakoCpp_ContactEventArray,
    ::hako::pdu::msgs::hako_msgs::ContactEventArray>;

HakoCpp_ContactEvent contact_pdu(const runtime::PersonContactEvent& event)
{
    HakoCpp_ContactEvent result {};
    const auto nanoseconds = static_cast<std::int64_t>(
        std::llround(event.time_sec * 1'000'000'000.0));
    result.stamp.sec = static_cast<std::int32_t>(
        nanoseconds / 1'000'000'000LL);
    result.stamp.nanosec = static_cast<std::uint32_t>(
        nanoseconds % 1'000'000'000LL);
    result.self_name = event.self_name;
    result.other_name = event.other_name;
    result.other_kind = event.other_kind;
    result.position.x = event.position.x;
    result.position.y = event.position.y;
    result.position.z = event.position.z;
    result.relative_speed = event.relative_speed_mps;
    result.started = event.started;
    return result;
}

class PersonContactOutput final {
public:
    PersonContactOutput(
        ::hakoniwa::pdu::Endpoint& endpoint,
        const std::vector<std::string>& people,
        std::string state_robot,
        const std::string& log_path)
        : array_endpoint_(
            endpoint,
            ::hakoniwa::pdu::PduKey {
                std::move(state_robot), "contact_events"})
    {
        for (const auto& person : people) {
            person_endpoints_.emplace(
                person,
                std::make_unique<ContactEndpoint>(
                    endpoint,
                    ::hakoniwa::pdu::PduKey {person, "contact"}));
        }
        if (!log_path.empty()) {
            const std::filesystem::path path(log_path);
            if (path.has_parent_path()) {
                std::filesystem::create_directories(path.parent_path());
            }
            log_.open(path, std::ios::app);
            if (!log_) {
                throw std::runtime_error(
                    "failed to open person contact log: " + log_path);
            }
        }
    }

    bool publish(const std::vector<runtime::PersonContactEvent>& events)
    {
        if (events.empty()) return true;
        if (events.front().id == 1) latest_.clear();
        for (const auto& event : events) {
            const auto endpoint = person_endpoints_.find(event.self_name);
            if (endpoint == person_endpoints_.end()
                || endpoint->second->send(contact_pdu(event)) != HAKO_PDU_ERR_OK) {
                return false;
            }
            latest_.push_back(event);
            if (latest_.size() > published_events) latest_.pop_front();
            if (log_) {
                const nlohmann::json record {
                    {"id", event.id},
                    {"time_sec", event.time_sec},
                    {"self", event.self_name},
                    {"other", event.other_name},
                    {"kind", event.other_kind},
                    {"started", event.started},
                    {"position", {
                        {"east_m", event.position.x},
                        {"north_m", event.position.y},
                        {"up_m", event.position.z},
                    }},
                    {"relative_speed", event.relative_speed_mps},
                };
                log_ << record.dump() << '\n';
                log_.flush();
                if (!log_) return false;
            }
        }
        HakoCpp_ContactEventArray array {};
        array.events.reserve(latest_.size());
        for (const auto& event : latest_) {
            array.events.push_back(contact_pdu(event));
        }
        return array_endpoint_.send(array) == HAKO_PDU_ERR_OK;
    }

private:
    static constexpr std::size_t published_events = 24;
    std::map<std::string, std::unique_ptr<ContactEndpoint>> person_endpoints_;
    ContactArrayEndpoint array_endpoint_;
    std::deque<runtime::PersonContactEvent> latest_;
    std::ofstream log_;
};

class PersonStatePublisher final : public runtime::IStatePublisher {
public:
    explicit PersonStatePublisher(std::function<bool(const runtime::RobotState&, std::string&)> write)
        : write_(std::move(write)) {}

    std::string_view id() const noexcept override { return "person-state"; }

    runtime::ComponentStatus publish(
        const runtime::RobotState& state, const runtime::RuntimeStepContext&) noexcept override
    {
        const std::string component_id(id());
        try {
            std::string error;
            const bool ok = write_(state, error);
            return {component_id, ok ? runtime::ComponentState::Ready
                                     : runtime::ComponentState::Error, std::move(error)};
        } catch (const std::exception& error) {
            return {component_id, runtime::ComponentState::Error, error.what()};
        } catch (...) {
            return {component_id, runtime::ComponentState::Error, "failed to publish person state"};
        }
    }

    void reset() noexcept override {}

private:
    std::function<bool(const runtime::RobotState&, std::string&)> write_;
};

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN) && HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN
struct ServicePluginRobot {
    std::shared_ptr<plugin::PluginLibrary> library;
    std::vector<std::size_t> service_indices;
};

std::unordered_map<std::string, std::string> read_rpc_service_types(const std::string& path)
{
    std::ifstream stream(path);
    if (!stream) {
        throw std::invalid_argument("failed to open RPC service config: " + path);
    }
    try {
        const auto config = nlohmann::json::parse(stream);
        std::unordered_map<std::string, std::string> types;
        for (const auto& service : config.at("services")) {
            types.emplace(service.at("name").get<std::string>(), service.at("type").get<std::string>());
        }
        return types;
    } catch (const nlohmann::json::exception& error) {
        throw std::invalid_argument("failed to read RPC service types from " + path + ": " + error.what());
    }
}
#endif

} // namespace

std::unique_ptr<runner::IRunner> ManifestFactory::create(
    const std::string& manifest_path,
    ManifestFactoryConfig config)
{
    if (config.endpoint_name.empty()) {
        throw std::invalid_argument(
            "Endpoint instance name must not be empty");
    }

    detail::ResolvedManifest manifest;
    std::string manifest_error;
    if (!detail::resolve_manifest(
            manifest_path,
            manifest,
            &manifest_error)) {
        throw std::invalid_argument(manifest_error);
    }

    runtime::RuntimeDefinition runtime_definition;
    std::string runtime_error;
    if (!runtime::resolve_runtime_definition(
            manifest.runtime_input,
            runtime_definition,
            &runtime_error)) {
        throw std::invalid_argument(runtime_error);
    }

    const std::string asset_name = config.asset_name.empty()
        ? manifest.name
        : std::move(config.asset_name);


    std::unordered_set<std::string> rpc_requested;
    std::vector<std::string> people;
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN) && HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN
    std::vector<ServicePluginRobot> plugin_robots;
    std::unordered_map<std::string, std::size_t> plugin_robot_indices;
#endif
    for (std::size_t index = 0; index < runtime_definition.services.size(); ++index) {
        const auto& service = runtime_definition.services[index];
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN) && HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN
        if (service.type == runtime::RuntimeServiceType::Plugin) {
            const auto [robot, inserted] = plugin_robot_indices.emplace(service.pdu_robot, plugin_robots.size());
            if (inserted) {
                plugin_robots.push_back({
                    plugin::PluginLibrary::open(service.plugin_library, HAKO_PLUGIN_KIND_SERVICE), {}});
            }
            plugin_robots[robot->second].service_indices.push_back(index);
            continue;
        }
#endif
        const auto& robot = service.pdu_robot;
        const bool ackermann = service.type == runtime::RuntimeServiceType::AckermannStep;
        const auto expected_name = ackermann
            ? adapters::hakoniwa::ackermann_step_service_name(robot)
            : adapters::hakoniwa::person_step_service_name(robot);
        if (service.name != expected_name) {
            throw std::invalid_argument("service " + service.name
                + " must use the name " + expected_name + " for " + robot);
        }
        if (!ackermann) {
            people.push_back(robot);
            continue;
        }
        const auto count = std::ranges::count(runtime_definition.ackermann_controllers, robot,
            &runtime::RuntimeAckermannControllerConfig::pdu_robot);
        if (count == 0) {
            throw std::invalid_argument("Ackermann RPC vehicle has no controller: " + robot);
        }
        if (count > 1) {
            throw std::invalid_argument("Ackermann RPC vehicle has multiple controllers: " + robot);
        }
        rpc_requested.insert(robot);
    }
    const bool has_services = !runtime_definition.services.empty();
    const bool has_service_definition = !config.async_rpc_service_definition_path.empty();
    if (has_services != has_service_definition) {
        throw std::invalid_argument("Manifest services must be paired with --rpc-service-config");
    }
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN) && HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN
    std::unordered_map<std::string, std::string> rpc_service_types;
    if (!plugin_robots.empty()) {
        rpc_service_types = read_rpc_service_types(config.async_rpc_service_definition_path);
        for (const auto& robot : plugin_robots) {
            for (const auto index : robot.service_indices) {
                const auto& name = runtime_definition.services[index].name;
                const auto type = rpc_service_types.find(name);
                if (type == rpc_service_types.end() || type->second.empty()) {
                    throw std::invalid_argument("RPC service config must provide a type for plugin service " + name);
                }
            }
        }
    }
#endif
    if (people.empty() && !config.person_contact_log_path.empty()) {
        throw std::invalid_argument("--person-contact-log requires a PersonStep input");
    }
    auto resources_factory =
        [definition = std::move(runtime_definition),
         endpoint_path = manifest.endpoint_path,
         endpoint_name = std::move(config.endpoint_name),
         lockstep_keys = config.lockstep_inputs,
         rpc_requested = std::move(rpc_requested),
         people = std::move(people),
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN) && HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN
         plugin_robots = std::move(plugin_robots),
         rpc_service_types = std::move(rpc_service_types),
#endif
         person_state_robot = config.person_state_robot,
         person_contact_log_path = config.person_contact_log_path,
         asset_name]() mutable
        -> runner::HakoniwaRunnerResources {
        auto endpoint = std::make_unique<::hakoniwa::pdu::Endpoint>(
            endpoint_name,
            HAKO_PDU_ENDPOINT_DIRECTION_INOUT);
        if (endpoint->open(endpoint_path, asset_name.c_str()) != HAKO_PDU_ERR_OK) {
            throw std::runtime_error(
                "failed to open Runtime endpoint: " + endpoint_path);
        }

        std::vector<runner::LockstepGate::Input> lockstep_inputs;
        std::vector<std::string> lockstep_matched;
        try {
            std::vector<runtime::RobotServiceComponents> additional_services;
            std::shared_ptr<runtime::IStatePublisher> person_state_publisher;
            std::shared_ptr<runner::hakoniwa::CoreAsyncServiceServer> service_server;
            auto plant = std::make_shared<
                adapters::mujoco::MujocoActuatorPlant>(definition);
            std::unordered_map<std::string, std::vector<std::string>> service_friction;
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
            for (const auto& friction : definition.geom_frictions) {
                if (friction.ackermann_step) {
                    service_friction.emplace(friction.pdu_robot, friction.geoms);
                }
            }
#endif
            std::unordered_map<std::string, std::string> state_id_by_vehicle;
            for (const auto& output : definition.multi_dof_state_outputs) {
                for (const auto& body : output.bodies) {
                    if (!state_id_by_vehicle.try_emplace(body.name, body.state_id).second
                        && rpc_requested.contains(body.name)) {
                        throw std::invalid_argument(
                            "Ackermann RPC vehicle has multiple state bindings: " + body.name);
                    }
                }
            }
            for (const auto& vehicle : rpc_requested) {
                if (!state_id_by_vehicle.contains(vehicle)) {
                    throw std::invalid_argument(
                        "Ackermann RPC vehicle has no MultiDOF state binding: " + vehicle);
                }
            }
            std::unordered_map<std::string, std::size_t> ackermann_service_ids;
            std::vector<std::string> service_names;
            for (std::size_t index = 0; index < definition.services.size(); ++index) {
                const auto& service = definition.services[index];
                service_names.push_back(service.name);
                if (service.type == runtime::RuntimeServiceType::AckermannStep) {
                    ackermann_service_ids.emplace(service.pdu_robot, index);
                }
            }
            if (!service_names.empty()) {
                service_server = std::make_shared<runner::hakoniwa::CoreAsyncServiceServer>(
                    asset_name, std::move(service_names));
            }
            const auto make_service_source = [&](std::size_t index,
                const std::shared_ptr<runtime::IRobotServices>& robot, std::size_t local_index, std::string id) {
                const auto& config = definition.services[index];
                auto service_source = std::make_shared<runtime::AsyncServiceCommandSource>(
                    std::move(id), static_cast<int>(index), service_server, robot, local_index, config.lockstep);
                if (config.lockstep) {
                    lockstep_inputs.push_back({config.name, service_source});
                }
                return service_source;
            };
            const auto make_service_components = [&](
                std::size_t index, std::shared_ptr<runtime::IRobotServices> robot,
                std::string controller_id = {}, std::string source_id = {}) {
                const auto& config = definition.services[index];
                auto service_source = make_service_source(index, robot, 0, config.name + "/0");
                auto source = std::make_shared<runtime::RobotServiceSource>(
                    source_id.empty() ? config.name : std::move(source_id), std::vector {service_source});
                return runtime::RobotServiceComponents {
                    source,
                    std::make_shared<runtime::RobotServiceController>(
                        controller_id.empty() ? config.name + ":controller" : std::move(controller_id),
                        source, robot),
                    {std::make_shared<runtime::AsyncServiceResponsePublisher>(
                        config.name + ":response", service_source)},
                };
            };
            for (const auto& person : people) {
                const auto initial = std::find_if(
                    definition.person_initial_poses.begin(),
                    definition.person_initial_poses.end(),
                    [&person](const auto& pose) {
                        return pose.name == person;
                    });
                plant->bind_person(
                    person,
                    initial == definition.person_initial_poses.end()
                        ? 0.0
                        : initial->yaw_rad);
            }
            std::function<bool(const runtime::RobotState&, std::string&)> publish_people;
            if (!people.empty()) {
                auto person_joint_writer = std::make_shared<
                    adapters::hakoniwa::JointStatePduWriter>(
                        *endpoint, person_state_robot, "joint_states");
                auto person_body_writer = std::make_shared<
                    adapters::hakoniwa::MultiDofJointStatePduWriter>(
                        *endpoint, person_state_robot, "vehicle_states");
                auto person_contact_output = std::make_shared<PersonContactOutput>(
                    *endpoint,
                    people,
                    person_state_robot,
                    person_contact_log_path);
                publish_people = [plant, person_joint_writer, person_body_writer, person_contact_output](
                    const runtime::RobotState& state, std::string& error) {
                    runtime::JointStateOutput joints;
                    runtime::MultiDofJointStateOutput bodies;
                    joints.simulation_time_usec = state.sample_time_usec;
                    bodies.simulation_time_usec = state.sample_time_usec;
                    bodies.frame_id = "city_map";
                    for (const auto& person : state.humanoids) {
                        for (std::size_t index = 0;
                             index < adapters::hakoniwa::person_limb_joint_names.size();
                             ++index) {
                            joints.names.push_back(
                                person.name + "/" + std::string(
                                    adapters::hakoniwa::person_limb_joint_names[index]));
                            joints.position.push_back(
                                person.joint_positions[index]);
                            joints.velocity.push_back(0.0);
                            joints.effort.push_back(0.0);
                        }
                        bodies.bodies.push_back({
                            person.name,
                            person.position,
                            person.orientation,
                            person.linear_velocity,
                            person.angular_velocity,
                        });
                    }
                    if (!person_joint_writer->send(joints)
                        || !person_body_writer->send(bodies)) {
                        error = "failed to publish viewer-compatible person state";
                        return false;
                    }
                    if (!person_contact_output->publish(
                            plant->take_person_contact_events())) {
                        error = "failed to publish person contact events";
                        return false;
                    }
                    return true;
                };
            }
            for (std::size_t index = 0; index < definition.services.size(); ++index) {
                const auto& service = definition.services[index];
                if (service.type != runtime::RuntimeServiceType::PersonStep) {
                    continue;
                }
                const auto& person = service.pdu_robot;
                auto library = plugin::PluginLibrary::open_builtin("person_step", HAKO_PLUGIN_KIND_SERVICE);
                auto person_config = runtime::person_plugin_config(person,
                    {service.name, "hako_srv_msgs/PersonStep", service.lockstep}, plant->delta_time_usec() * 1000);
                runtime::require_robot_plugin_declaration(*library, person_config, plant->capabilities());
                additional_services.push_back(make_service_components(index,
                    std::make_shared<runtime::ServicePlugin>(std::move(library), std::move(person_config))));
            }
            if (!additional_services.empty()) {
                // An asynchronous person may have no reply on this tick.
                // Publish viewer state and contacts once independently of replies.
                person_state_publisher = std::make_shared<PersonStatePublisher>(
                    std::exchange(publish_people, {}));
            }
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN) && HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN
            for (const auto& robot : plugin_robots) {
                const auto& first = definition.services[robot.service_indices.front()];
                runtime::ServicePluginConfig plugin_config;
                plugin_config.robot = first.pdu_robot;
                plugin_config.config_json = first.plugin_config_json;
                plugin_config.physics_step_ns = plant->delta_time_usec() * 1000;
                for (const auto& actuator : definition.actuators) {
                    if (actuator.pdu_robot == first.pdu_robot) {
                        plugin_config.actuators.push_back({
                            actuator.component_id, actuator.command_type, actuator.limits});
                    }
                }
                for (const auto index : robot.service_indices) {
                    const auto& service = definition.services[index];
                    plugin_config.services.push_back({
                        service.name, rpc_service_types.at(service.name), service.lockstep});
                }
                runtime::require_robot_plugin_declaration(*robot.library, plugin_config, plant->capabilities());
                const std::shared_ptr<runtime::IRobotServices> instance = std::make_shared<runtime::ServicePlugin>(
                    robot.library, std::move(plugin_config));
                std::vector<std::shared_ptr<runtime::AsyncServiceCommandSource>> sources;
                std::vector<std::shared_ptr<runtime::IStatePublisher>> publishers;
                for (std::size_t local = 0; local < robot.service_indices.size(); ++local) {
                    const auto index = robot.service_indices[local];
                    const auto& service = definition.services[index];
                    auto service_source = make_service_source(index, instance, local, service.name);
                    publishers.push_back(std::make_shared<runtime::AsyncServiceResponsePublisher>(
                        service.name + ":response", service_source));
                    sources.push_back(std::move(service_source));
                }
                auto source = std::make_shared<runtime::RobotServiceSource>(first.pdu_robot, std::move(sources));
                auto controller = std::make_shared<runtime::RobotServiceController>(
                    first.pdu_robot + ":controller", source, instance);
                additional_services.push_back({std::move(source), std::move(controller),
                    std::move(publishers), first.priority});
            }
            // Sensor PDUs of the controller plugins: the plugins read the
            // bytes the sensor writers encode for the endpoint.
            const auto plugin_pdu_channels = runtime::make_plugin_pdu_channels(definition);
#endif
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MOBILE_BASE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MOBILE_BASE
            const auto bind_robot_pdu = [&](const std::string& robot, const std::string& pdu,
                adapters::hakoniwa::RobotPduEventReader::Validator validate = nullptr) -> runtime::RobotPduInput {
                auto reader = std::make_shared<adapters::hakoniwa::RobotPduEventReader>(
                    *endpoint, robot, pdu, validate);
                const auto key = robot + "/" + pdu;
                if (std::find(lockstep_keys.begin(), lockstep_keys.end(), key) != lockstep_keys.end()) {
                    reader->subscribe_lockstep();
                    lockstep_inputs.push_back({key, reader});
                    lockstep_matched.push_back(key);
                } else {
                    reader->subscribe();
                }
                return {pdu, std::move(reader)};
            };
#endif
            auto runtime_instance = runtime::build_runtime(
                definition,
                plant,
                [&](const runtime::RuntimeActuatorConfig& actuator) {
                    auto reader = std::make_shared<
                        adapters::hakoniwa::Float64PduEventReader>(
                        *endpoint,
                        actuator.pdu_robot,
                        actuator.pdu_name);
                    reader->subscribe();
                    return reader;
                },
                [&](const runtime::RuntimeStateOutputConfig& output) {
                    return std::make_shared<
                        adapters::hakoniwa::JointStatePduWriter>(
                        *endpoint,
                        output.pdu_robot,
                        output.pdu_name);
                },
                [&](const runtime::RuntimeTrajectoryControllerConfig& controller) {
                    auto reader = std::make_shared<
                        adapters::hakoniwa::JointTrajectoryPduEventReader>(
                        *endpoint,
                        controller.pdu_robot,
                        controller.pdu_name);
                    reader->subscribe();
                    return reader;
                },
                [&](const runtime::RuntimeManualControllerConfig& controller) {
                    auto reader = std::make_shared<
                        adapters::hakoniwa::JoyPduEventReader>(
                        *endpoint,
                        controller.pdu_robot,
                        controller.pdu_name);
                    reader->subscribe();
                    return reader;
                }
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MOBILE_BASE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MOBILE_BASE
                ,
                [&](const runtime::RuntimeAckermannControllerConfig& drive) {
                    const auto& vehicle = drive.pdu_robot;
                    const auto service = ackermann_service_ids.find(vehicle);
                    const bool rpc = service != ackermann_service_ids.end();
                    const auto friction = service_friction.find(vehicle);
                    auto config = runtime::ackermann_plugin_config(drive, definition.actuators, rpc,
                        friction == service_friction.end() ? std::vector<std::string> {} : friction->second);
                    config.physics_step_ns = plant->delta_time_usec() * 1000;
                    if (const auto body = state_id_by_vehicle.find(vehicle); body != state_id_by_vehicle.end()) {
                        config.bodies.push_back(body->second);
                    }
                    const std::string drive_source = "ackermann-pdu:" + drive.component_id;
                    if (rpc) {
                        const auto& row = definition.services[service->second];
                        config.services.push_back({row.name, "hako_srv_msgs/AckermannStep", row.lockstep});
                    } else {
                        config.pdus.push_back(bind_robot_pdu(vehicle, drive.pdu_name,
                            plugins::ackermann_step::valid_topic));
                    }
                    auto library = plugin::PluginLibrary::open_builtin("ackermann_step", HAKO_PLUGIN_KIND_SERVICE);
                    runtime::require_robot_plugin_declaration(*library, config, plant->capabilities());
                    auto robot = std::make_shared<runtime::ServicePlugin>(std::move(library), std::move(config));
                    if (rpc) {
                        return make_service_components(service->second, robot, drive.component_id, drive_source);
                    }
                    auto source = std::make_shared<runtime::RobotServiceSource>(drive_source,
                        std::vector<std::shared_ptr<runtime::AsyncServiceCommandSource>> {},
                        [robot](const auto& context) { return robot->poll_pdus(context); });
                    return runtime::RobotServiceComponents {
                        source,
                        std::make_shared<runtime::RobotServiceController>(drive.component_id, source, robot),
                        {},
                    };
                },
                [&](const runtime::RuntimeMultiDofStateOutputConfig& output) {
                    return std::make_shared<
                        adapters::hakoniwa::MultiDofJointStatePduWriter>(
                        *endpoint,
                        output.pdu_robot,
                        output.pdu_name);
                }
#elif (defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) \
    || (defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN) && HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN)
                , {}, {}
#endif
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
                ,
                [&](const runtime::RuntimeMirrorBodyConfig& mirror) {
                    return std::make_shared<
                        adapters::hakoniwa::MirrorBodyTwistPduReader>(
                        *endpoint,
                        mirror.pdu_robot,
                        mirror.pose_pdu_name,
                        mirror.velocity_pdu_name);
                },
                [&](const runtime::RuntimeImpulseCollisionOutputConfig& output) {
                    return std::make_shared<
                        adapters::hakoniwa::ImpulseCollisionPduWriter>(
                        *endpoint,
                        output.pdu_robot,
                        output.pdu_name);
                },
                [&](const runtime::RuntimeGeomFrictionConfig& friction)
                    -> std::shared_ptr<runtime::IFloat64EventReader> {
                    auto reader = std::make_shared<
                        adapters::hakoniwa::Float64PduEventReader>(
                        *endpoint,
                        friction.pdu_robot,
                        friction.pdu_name);
                    const auto key = friction.pdu_robot + "/" + friction.pdu_name;
                    if (std::find(lockstep_keys.begin(), lockstep_keys.end(), key)
                        != lockstep_keys.end()) {
                        reader->subscribe_lockstep();
                        lockstep_inputs.push_back({key, reader});
                        lockstep_matched.push_back(key);
                    } else {
                        reader->subscribe();
                    }
                    return reader;
                }
#endif
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN) && HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN
                , &plugin_pdu_channels
#endif
                , std::move(additional_services)
                , std::move(person_state_publisher)
                );

            for (const auto& key : lockstep_keys) {
                if (std::find(lockstep_matched.begin(), lockstep_matched.end(), key)
                    == lockstep_matched.end()) {
                    throw std::invalid_argument(
                        "lockstep input " + key + " is not an Ackermann command or a geom friction input");
                }
            }
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR
            if (!definition.sensors.empty()) {
                auto stage = std::make_shared<adapters::mujoco::MujocoSensorStage>(
                    plant->model(),
                    plant->data(),
                    definition.sensors,
                    definition.sensor_seed,
                    plant->delta_time_usec(),
                    runtime::sensor::sensor_implementation_registry(),
                    // Called only while the stage is built; the plant owns the world.
                    [plant_ptr = plant.get()](
                        const adapters::mujoco::MujocoSensorStage::RayCasterMount& mount) {
                        return plant_ptr->create_ray_caster(mount.body, mount.position_m,
                            mount.orientation_wxyz, mount.exclude_root_body);
                    },
                    definition.sensor_world,
                    [plant_ptr = plant.get()]() { return plant_ptr->create_image_renderer(); });
                std::vector<std::shared_ptr<runtime::IStatePublisher>> sensor_publishers;
                for (const auto& output : definition.sensor_outputs) {
                    // The controller plugin channel of this PDU, or null.
                    std::shared_ptr<runtime::IEncodedPduSink> plugin_sink;
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN) && HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN
                    plugin_sink = plugin_pdu_channels.find(output.pdu_name);
#endif
                    if (output.message_type == "sensor_msgs/Imu") {
                        sensor_publishers.push_back(
                            std::make_shared<runtime::SensorImuPublisher>(
                                output.component_id,
                                output.sensors.front(),
                                output.update_rate_hz,
                                std::make_shared<adapters::hakoniwa::ImuPduWriter>(
                                    *endpoint,
                                    output.pdu_robot,
                                    output.pdu_name)));
                        continue;
                    }
                    if (output.message_type == "sensor_msgs/PointCloud2") {
                        sensor_publishers.push_back(
                            std::make_shared<runtime::SensorPointCloudPublisher>(
                                output.component_id,
                                output.sensors.front(),
                                output.update_rate_hz,
                                std::make_shared<adapters::hakoniwa::PointCloud2PduWriter>(
                                    *endpoint,
                                    output.pdu_robot,
                                    output.pdu_name,
                                    plugin_sink)));
                        continue;
                    }
                    if (output.message_type == "sensor_msgs/MagneticField") {
                        sensor_publishers.push_back(
                            std::make_shared<runtime::SensorMagneticFieldPublisher>(
                                output.component_id,
                                output.sensors.front(),
                                output.update_rate_hz,
                                std::make_shared<adapters::hakoniwa::MagneticFieldPduWriter>(
                                    *endpoint,
                                    output.pdu_robot,
                                    output.pdu_name)));
                        continue;
                    }
                    if (output.message_type == "sensor_msgs/FluidPressure") {
                        sensor_publishers.push_back(
                            std::make_shared<runtime::SensorFluidPressurePublisher>(
                                output.component_id,
                                output.sensors.front(),
                                output.update_rate_hz,
                                std::make_shared<adapters::hakoniwa::FluidPressurePduWriter>(
                                    *endpoint,
                                    output.pdu_robot,
                                    output.pdu_name)));
                        continue;
                    }
                    if (output.message_type == "sensor_msgs/NavSatFix") {
                        sensor_publishers.push_back(
                            std::make_shared<runtime::SensorNavSatFixPublisher>(
                                output.component_id,
                                output.sensors.front(),
                                output.update_rate_hz,
                                std::make_shared<adapters::hakoniwa::NavSatFixPduWriter>(
                                    *endpoint,
                                    output.pdu_robot,
                                    output.pdu_name)));
                        continue;
                    }
                    if (output.message_type == "sensor_msgs/CompressedImage") {
                        sensor_publishers.push_back(
                            std::make_shared<runtime::SensorCompressedImagePublisher>(
                                output.component_id,
                                output.sensors.front(),
                                output.update_rate_hz,
                                std::make_shared<adapters::hakoniwa::CompressedImagePduWriter>(
                                    *endpoint,
                                    output.pdu_robot,
                                    output.cameras,
                                    output.jpeg_quality)));
                        continue;
                    }
                    if (output.message_type == "sensor_msgs/Image") {
                        sensor_publishers.push_back(
                            std::make_shared<runtime::SensorDepthImagePublisher>(
                                output.component_id,
                                output.sensors.front(),
                                output.update_rate_hz,
                                std::make_shared<adapters::hakoniwa::DepthImagePduWriter>(
                                    *endpoint,
                                    output.pdu_robot,
                                    output.pdu_name,
                                    output.camera_info_pdu_name)));
                        continue;
                    }
                    if (output.message_type == "sensor_msgs/Range") {
                        sensor_publishers.push_back(
                            std::make_shared<runtime::SensorRangePublisher>(
                                output.component_id,
                                output.sensors.front(),
                                output.update_rate_hz,
                                std::make_shared<adapters::hakoniwa::RangePduWriter>(
                                    *endpoint,
                                    output.pdu_robot,
                                    output.pdu_name)));
                        continue;
                    }
                    if (output.message_type == "sensor_msgs/LaserScan") {
                        sensor_publishers.push_back(
                            std::make_shared<runtime::SensorLaserScanPublisher>(
                                output.component_id,
                                output.sensors.front(),
                                output.update_rate_hz,
                                std::make_shared<adapters::hakoniwa::LaserScanPduWriter>(
                                    *endpoint,
                                    output.pdu_robot,
                                    output.pdu_name)));
                        continue;
                    }
                    if (output.message_type == "geometry_msgs/WrenchStamped") {
                        sensor_publishers.push_back(
                            std::make_shared<runtime::SensorWrenchPublisher>(
                                output.component_id,
                                output.sensors.front(),
                                output.update_rate_hz,
                                std::make_shared<adapters::hakoniwa::WrenchStampedPduWriter>(
                                    *endpoint,
                                    output.pdu_robot,
                                    output.pdu_name,
                                    plugin_sink)));
                        continue;
                    }
                    sensor_publishers.push_back(
                        std::make_shared<runtime::SensorJointStatePublisher>(
                            output.component_id,
                            output.sensors,
                            output.update_rate_hz,
                            std::make_shared<adapters::hakoniwa::JointStatePduWriter>(
                                *endpoint,
                                output.pdu_robot,
                                output.pdu_name)));
                }
                runtime_instance->attach_sensors(
                    std::move(stage), std::move(sensor_publishers));
            }
#endif
            return {
                std::move(runtime_instance),
                std::move(plant),
                std::move(endpoint),
                std::move(lockstep_inputs),
                service_server ? std::function<void()>([service_server] { service_server->prepare(); })
                    : std::function<void()>(),
            };
        } catch (...) {
            endpoint->close();
            throw;
        }
    };

    return runner::create_hakoniwa_runner(
        std::move(resources_factory),
        {
            asset_name,
            manifest.pdu_definition_path,
            config.realtime_sync_cycle_msec,
            config.owns_conductor,
            config.conductor_max_delay_usec,
            config.physics_substeps,
            config.async_rpc_service_definition_path,
        });
}

} // namespace hakoniwa::robot_runtime::factory
