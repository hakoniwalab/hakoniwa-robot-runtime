#include "hakoniwa/robot_runtime/factory/manifest_factory.hpp"

#include "factory/manifest_factory_internal.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/float64_pdu_event_reader.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/joint_state_pdu_writer.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/joint_trajectory_pdu_event_reader.hpp"
#include "hakoniwa/robot_runtime/adapters/endpoint/joy_pdu_event_reader.hpp"
#include "hakoniwa/robot_runtime/adapters/physics/mujoco/mujoco_actuator_plant.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "runner/runner_factory.hpp"
#include "runtime/runtime_factory.hpp"
#include "runtime/runtime_factory_manifest.hpp"

#include <memory>
#include <stdexcept>
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

    auto resources_factory =
        [definition = std::move(runtime_definition),
         endpoint_path = manifest.endpoint_path,
         endpoint_name = std::move(config.endpoint_name),
         asset_name]() mutable
        -> runner::HakoniwaRunnerResources {
        auto endpoint = std::make_unique<::hakoniwa::pdu::Endpoint>(
            endpoint_name,
            HAKO_PDU_ENDPOINT_DIRECTION_INOUT);
        if (endpoint->open(endpoint_path, asset_name.c_str()) != HAKO_PDU_ERR_OK) {
            throw std::runtime_error(
                "failed to open Runtime endpoint: " + endpoint_path);
        }

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
                });

            return {
                std::move(runtime_instance),
                std::move(plant),
                std::move(endpoint),
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
        });
}

} // namespace hakoniwa::robot_runtime::factory
