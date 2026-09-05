#include "runtime/runtime_factory.hpp"

#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runtime/controller/hold_controller.hpp"
#include "runtime/controller/joint_trajectory_controller.hpp"
#include "runtime/controller/manual_controller.hpp"
#include "runtime/controller/scalar_pdu_command_controller.hpp"

#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

std::unique_ptr<ActuatorRuntime> build_runtime(
    const RuntimeDefinition& definition,
    std::shared_ptr<IActuatorPlant> plant,
    const Float64EventReaderFactory& reader_factory,
    const JointStateWriterFactory& writer_factory,
    const JointTrajectoryEventReaderFactory& trajectory_reader_factory,
    const JoyEventReaderFactory& joy_reader_factory)
{
    if (plant == nullptr) {
        throw std::invalid_argument("Actuator Plant must be provided");
    }
    if (!reader_factory) {
        throw std::invalid_argument(
            "Float64 event reader factory must be provided");
    }
    if (!definition.state_outputs.empty() && !writer_factory) {
        throw std::invalid_argument(
            "joint state writer factory is required by the Runtime definition");
    }
    if (!definition.trajectory_controllers.empty()
        && !trajectory_reader_factory) {
        throw std::invalid_argument(
            "trajectory event reader factory is required by the Runtime definition");
    }
    if (!definition.manual_controllers.empty() && !joy_reader_factory) {
        throw std::invalid_argument(
            "Joy event reader factory is required by the Runtime definition");
    }
    if (definition.manual_controllers.size() > 1) {
        throw std::invalid_argument(
            "only one arm Manual Controller is supported");
    }

    std::vector<std::shared_ptr<ICommandSource>> sources;
    std::vector<std::shared_ptr<IController>> controllers;
    std::vector<ControllerPriority> priorities;
    sources.reserve(definition.actuators.size());
    controllers.reserve(
        definition.actuators.size()
        + definition.manual_controllers.size()
        + definition.trajectory_controllers.size()
        + 1);
    priorities.reserve(controllers.capacity());

    constexpr const char* scalar_control_group = "scalar-pdu-direct";

    for (const auto& actuator_config : definition.actuators) {
        auto reader = reader_factory(actuator_config);
        if (reader == nullptr) {
            throw std::invalid_argument(
                "reader factory returned null for actuator: "
                + actuator_config.component_id);
        }
        const std::string source_id =
            "plant-pdu:" + actuator_config.component_id;
        const std::string controller_id =
            "scalar-command:" + actuator_config.component_id;
        sources.push_back(std::make_shared<ScalarPduCommandSource>(
            source_id,
            actuator_config.command_timeout_usec,
            std::move(reader)));
        controllers.push_back(std::make_shared<ScalarPduCommandController>(
            controller_id,
            source_id,
            actuator_config.component_id,
            actuator_config.command_type));
        priorities.push_back({controller_id, 0, scalar_control_group});
    }

    std::unordered_map<std::string, const RuntimeActuatorConfig*> actuator_by_id;
    for (const auto& actuator : definition.actuators) {
        actuator_by_id.emplace(actuator.component_id, &actuator);
    }

    if (!definition.manual_controllers.empty()
        || !definition.trajectory_controllers.empty()) {
        std::vector<HoldActuatorBinding> hold_actuators;
        for (const auto& actuator : definition.actuators) {
            if (actuator.command_type == ActuatorCommandType::Position) {
                hold_actuators.push_back({actuator.component_id});
            }
        }
        if (!hold_actuators.empty()) {
            constexpr const char* hold_controller_id = "runtime-hold";
            controllers.push_back(std::make_shared<HoldController>(
                hold_controller_id, std::move(hold_actuators)));
            priorities.push_back({hold_controller_id, -10, {}});
        }
    }

    for (const auto& manual_config : definition.manual_controllers) {
        auto reader = joy_reader_factory(manual_config);
        if (reader == nullptr) {
            throw std::invalid_argument(
                "Joy reader factory returned null: "
                + manual_config.component_id);
        }
        const std::string source_id =
            "joy-pdu:" + manual_config.component_id;
        sources.push_back(std::make_shared<JoyCommandSource>(
            source_id,
            manual_config.input_timeout_usec,
            std::move(reader)));

        std::unordered_map<std::string, bool> referenced_actuators;
        std::vector<ManualBank> banks;
        banks.reserve(manual_config.banks.size());
        for (const auto& bank : manual_config.banks) {
            ManualBank parsed {bank.id, bank.select_button_index, {}};
            parsed.bindings.reserve(bank.bindings.size());
            for (const auto& binding : bank.bindings) {
                referenced_actuators.emplace(binding.actuator_id, true);
                parsed.bindings.push_back({
                    binding.axis_index,
                    binding.actuator_id,
                    binding.velocity_per_sec,
                    binding.invert,
                });
            }
            banks.push_back(std::move(parsed));
        }

        std::vector<ManualActuatorBinding> manual_actuators;
        for (const auto& actuator : definition.actuators) {
            if (!referenced_actuators.contains(actuator.component_id)) {
                continue;
            }
            if (actuator.command_type != ActuatorCommandType::Position) {
                throw std::invalid_argument(
                    "Manual Controller requires position actuator: "
                    + actuator.component_id);
            }
            manual_actuators.push_back({
                actuator.component_id,
                actuator.limits,
            });
        }
        if (manual_actuators.size() != referenced_actuators.size()) {
            throw std::invalid_argument(
                "Manual Controller references an unknown actuator");
        }

        controllers.push_back(std::make_shared<ManualController>(
            manual_config.component_id,
            source_id,
            manual_config.manual_enable_button_index,
            manual_config.joy_axis_count,
            manual_config.joy_button_count,
            manual_config.deadzone,
            manual_config.expo,
            std::move(manual_actuators),
            std::move(banks)));
        priorities.push_back({manual_config.component_id, 20, {}});
    }

    for (const auto& trajectory_config : definition.trajectory_controllers) {
        auto reader = trajectory_reader_factory(trajectory_config);
        if (reader == nullptr) {
            throw std::invalid_argument(
                "trajectory reader factory returned null: "
                + trajectory_config.component_id);
        }
        const std::string source_id =
            "trajectory-pdu:" + trajectory_config.component_id;
        sources.push_back(std::make_shared<JointTrajectoryCommandSource>(
            source_id, std::move(reader)));

        std::vector<TrajectoryActuatorBinding> bindings;
        bindings.reserve(trajectory_config.joints.size());
        for (const auto& joint : trajectory_config.joints) {
            const auto actuator = actuator_by_id.find(joint.actuator_id);
            if (actuator == actuator_by_id.end()) {
                throw std::invalid_argument(
                    "trajectory controller references unknown actuator: "
                    + joint.actuator_id);
            }
            bindings.push_back({
                joint.joint_name,
                joint.actuator_id,
                actuator->second->command_type,
            });
        }
        controllers.push_back(std::make_shared<JointTrajectoryController>(
            trajectory_config.component_id,
            source_id,
            std::move(bindings)));
        priorities.push_back({trajectory_config.component_id, 10, {}});
    }

    std::vector<std::shared_ptr<IStatePublisher>> publishers;
    publishers.reserve(definition.state_outputs.size());
    for (const auto& output : definition.state_outputs) {
        auto writer = writer_factory(output);
        if (writer == nullptr) {
            throw std::invalid_argument(
                "writer factory returned null for state output: "
                + output.component_id);
        }
        std::vector<JointStateOutputBinding> bindings;
        bindings.reserve(output.joints.size());
        for (const auto& joint : output.joints) {
            bindings.push_back({
                joint.joint_name,
                joint.actuator_id,
            });
        }
        publishers.push_back(std::make_shared<JointStatePublisher>(
            output.component_id,
            std::move(bindings),
            output.update_rate_hz,
            std::move(writer)));
    }

    auto arbiter = std::make_shared<PriorityCommandArbiter>(
        std::move(priorities));
    return std::make_unique<ActuatorRuntime>(
        std::move(sources),
        std::move(controllers),
        std::move(arbiter),
        std::move(plant),
        std::move(publishers));
}

} // namespace hakoniwa::robot_runtime::runtime
