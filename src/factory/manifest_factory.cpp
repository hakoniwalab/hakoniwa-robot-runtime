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
#include "hakoniwa/robot_runtime/adapters/endpoint/ackermann_drive_pdu_event_reader.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/multi_dof_joint_state_pdu_writer.hpp"
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
#include "hakoniwa/robot_runtime/adapters/endpoint/ackermann_drive_rpc_event_reader.hpp"
#endif
#endif
#include "hakoniwa/robot_runtime/adapters/physics/mujoco/mujoco_actuator_plant.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "runner/runner_factory.hpp"
#include "runtime/runtime_factory.hpp"
#include "runtime/runtime_factory_manifest.hpp"

#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hakoniwa::robot_runtime::factory {

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

#if !defined(HAKO_ENABLE_ASYNC_RPC_INPUT) || !HAKO_ENABLE_ASYNC_RPC_INPUT
    if (!config.ackermann_rpc_inputs.empty()) {
        throw std::invalid_argument(
            "Ackermann RPC input support is not enabled in this build");
    }
#endif

    auto resources_factory =
        [definition = std::move(runtime_definition),
         endpoint_path = manifest.endpoint_path,
         endpoint_name = std::move(config.endpoint_name),
         lockstep_keys = config.lockstep_inputs,
         rpc_vehicles = config.ackermann_rpc_inputs,
         asset_name]() mutable
        -> runner::HakoniwaRunnerResources {
        auto endpoint = std::make_unique<::hakoniwa::pdu::Endpoint>(
            endpoint_name,
            HAKO_PDU_ENDPOINT_DIRECTION_INOUT);
        if (endpoint->open(endpoint_path, asset_name.c_str()) != HAKO_PDU_ERR_OK) {
            throw std::runtime_error(
                "failed to open Runtime endpoint: " + endpoint_path);
        }

        std::vector<std::shared_ptr<runner::ILockstepInput>> lockstep_inputs;
        std::vector<std::string> lockstep_matched;
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
        std::unordered_map<std::string,
            std::shared_ptr<adapters::hakoniwa::AckermannDriveRpcEventReader>>
            rpc_readers;
        std::unordered_set<std::string> rpc_requested(
            rpc_vehicles.begin(), rpc_vehicles.end());
        if (rpc_requested.size() != rpc_vehicles.size()) {
            throw std::invalid_argument("duplicate Ackermann RPC vehicle");
        }
#endif
        try {
            auto plant = std::make_shared<
                adapters::mujoco::MujocoActuatorPlant>(definition);
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
                [&](const runtime::RuntimeAckermannControllerConfig& controller)
                    -> std::shared_ptr<runtime::IAckermannDriveEventReader> {
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
                    if (rpc_requested.contains(controller.pdu_robot)) {
                        auto reader = std::make_shared<
                            adapters::hakoniwa::AckermannDriveRpcEventReader>();
                        if (!rpc_readers.emplace(controller.pdu_robot, reader).second) {
                            throw std::invalid_argument(
                                "Ackermann RPC vehicle has multiple controllers: "
                                + controller.pdu_robot);
                        }
                        return std::shared_ptr<runtime::IAckermannDriveEventReader>(reader);
                    }
#endif
                    auto reader = std::make_shared<
                        adapters::hakoniwa::AckermannDrivePduEventReader>(
                        *endpoint,
                        controller.pdu_robot,
                        controller.pdu_name);
                    const auto key = controller.pdu_robot + "/" + controller.pdu_name;
                    if (std::find(lockstep_keys.begin(), lockstep_keys.end(), key)
                        != lockstep_keys.end()) {
                        reader->subscribe_lockstep();
                        lockstep_inputs.push_back(reader);
                        lockstep_matched.push_back(key);
                    } else {
                        reader->subscribe();
                    }
                    return reader;
                },
                [&](const runtime::RuntimeMultiDofStateOutputConfig& output) {
                    return std::make_shared<
                        adapters::hakoniwa::MultiDofJointStatePduWriter>(
                        *endpoint,
                        output.pdu_robot,
                        output.pdu_name);
                }
#elif defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
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
                [&](const runtime::RuntimeGeomFrictionConfig& friction) {
                    auto reader = std::make_shared<
                        adapters::hakoniwa::Float64PduEventReader>(
                        *endpoint,
                        friction.pdu_robot,
                        friction.pdu_name);
                    const auto key = friction.pdu_robot + "/" + friction.pdu_name;
                    if (std::find(lockstep_keys.begin(), lockstep_keys.end(), key)
                        != lockstep_keys.end()) {
                        reader->subscribe_lockstep();
                        lockstep_inputs.push_back(reader);
                        lockstep_matched.push_back(key);
                    } else {
                        reader->subscribe();
                    }
                    return reader;
                }
#endif
                );

            for (const auto& key : lockstep_keys) {
                if (std::find(lockstep_matched.begin(), lockstep_matched.end(), key)
                    == lockstep_matched.end()) {
                    throw std::invalid_argument(
                        "lockstep input " + key + " is not an Ackermann command or a geom friction input");
                }
            }
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
            std::vector<std::shared_ptr<runner::AsyncRpcInput>> async_rpc_inputs;
            if (!rpc_requested.empty()) {
                if (rpc_readers.size() != rpc_requested.size()) {
                    for (const auto& vehicle : rpc_requested) {
                        if (!rpc_readers.contains(vehicle)) {
                            throw std::invalid_argument(
                                "Ackermann RPC vehicle has no controller: " + vehicle);
                        }
                    }
                }
                std::unordered_map<std::string, std::string> state_id_by_vehicle;
                for (const auto& output : definition.multi_dof_state_outputs) {
                    for (const auto& body : output.bodies) {
                        if (rpc_requested.contains(body.name)
                            && !state_id_by_vehicle.emplace(body.name, body.state_id).second) {
                            throw std::invalid_argument(
                                "Ackermann RPC vehicle has multiple state bindings: "
                                + body.name);
                        }
                    }
                }
                std::vector<std::shared_ptr<runner::IAsyncRpcChannel>> channels;
                channels.reserve(rpc_vehicles.size());
                for (const auto& vehicle : rpc_vehicles) {
                    if (!state_id_by_vehicle.contains(vehicle)) {
                        throw std::invalid_argument(
                            "Ackermann RPC vehicle has no MultiDOF state binding: "
                            + vehicle);
                    }
                    channels.push_back(
                        adapters::hakoniwa::make_ackermann_step_service_channel(
                            vehicle, asset_name));
                }
                auto input = std::make_shared<runner::AsyncRpcInput>(
                    std::move(channels),
                    runner::AsyncRpcInputConfig {},
                    [rpc_readers](
                        std::span<const runner::AsyncRpcRequest> requests,
                        std::string& error) {
                        for (const auto& request : requests) {
                            runtime::AckermannDriveCommand command;
                            const auto reader = rpc_readers.find(request.channel);
                            if (reader == rpc_readers.end()
                                || !adapters::hakoniwa::decode_ackermann_command(
                                    request.payload, command)) {
                                error = "invalid Ackermann command for " + request.channel;
                                return false;
                            }
                            reader->second->apply(command);
                        }
                        return true;
                    },
                    [plant, state_id_by_vehicle](
                        const runner::AsyncRpcRequest& request,
                        runner::AsyncRpcResponse& response,
                        std::string& error) {
                        const auto binding = state_id_by_vehicle.find(request.channel);
                        if (binding == state_id_by_vehicle.end()) {
                            error = "missing Ackermann state binding for " + request.channel;
                            return false;
                        }
                        const auto state = plant->read_state();
                        const auto body = std::find_if(
                            state.bodies.begin(), state.bodies.end(),
                            [&](const auto& candidate) {
                                return candidate.body_id == binding->second;
                            });
                        if (body == state.bodies.end()) {
                            error = "plant state is missing body " + binding->second;
                            return false;
                        }
                        response.step = request.step;
                        response.payload = adapters::hakoniwa::encode_ackermann_state({
                            state.sample_time_usec,
                            body->position,
                            body->orientation,
                            body->linear_velocity,
                            body->angular_velocity,
                        });
                        return true;
                    });
                async_rpc_inputs.push_back(std::move(input));
            }
#endif
            return {
                std::move(runtime_instance),
                std::move(plant),
                std::move(endpoint),
                std::move(lockstep_inputs),
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
                std::move(async_rpc_inputs),
#endif
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
