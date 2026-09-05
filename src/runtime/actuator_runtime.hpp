#pragma once

#include "runtime/arbiter/command_arbiter.hpp"
#include "runtime/controller/controller.hpp"
#include "runtime/plant/actuator_plant.hpp"
#include "runtime/publisher/state_publisher.hpp"
#include "runtime/source/command_source.hpp"

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

struct RuntimeStepReport {
    RuntimeStepContext completed_step;
    RuntimeStepContext next_step;
    std::vector<ComponentStatus> source_statuses;
    std::vector<ComponentStatus> controller_statuses;
    ArbitrationResult arbitration;
    std::vector<ComponentStatus> publisher_statuses;
};

/**
 * Orchestrates the five Runtime responsibilities for exactly one simulation
 * step: CommandSource -> Controller -> Arbiter -> Plant -> Publisher.
 */
class ActuatorRuntime final {
public:
    ActuatorRuntime(
        std::vector<std::shared_ptr<ICommandSource>> sources,
        std::vector<std::shared_ptr<IController>> controllers,
        std::shared_ptr<ICommandArbiter> arbiter,
        std::shared_ptr<IActuatorPlant> plant,
        std::vector<std::shared_ptr<IStatePublisher>> publishers);

    /** Runs exactly one Source -> Controller -> Arbiter -> Plant -> Publish step. */
    [[nodiscard]] RuntimeStepReport step();

    /**
     * Resets component/physical state and sets the Plant's actual simulation time.
     * Hakoniwa Core reset uses zero; local Viewer reset can preserve Asset time.
     */
    [[nodiscard]] RobotState reset(std::uint64_t simulation_time_usec = 0);

private:
    using ControllerInputMap = std::unordered_map<
        std::string_view,
        std::shared_ptr<const IControllerInput>>;

    [[nodiscard]] RobotState read_current_state() const;
    [[nodiscard]] RuntimeStepContext prepare_step_context(
        const RobotState& current_state) const;
    [[nodiscard]] ControllerInputMap poll_sources(
        const RuntimeStepContext& context,
        std::vector<ComponentStatus>& statuses);
    [[nodiscard]] std::vector<ControllerOutput> update_controllers(
        const ControllerInputMap& inputs,
        const RobotState& current_state,
        const RuntimeStepContext& context,
        std::vector<ComponentStatus>& statuses);
    [[nodiscard]] RobotState step_plant(
        const std::vector<ActuatorCommand>& commands);
    [[nodiscard]] RuntimeStepContext complete_step(
        const RobotState& next_state,
        const ArbitrationResult& arbitration);
    void publish_state(
        const RobotState& state,
        const RuntimeStepContext& context,
        std::vector<ComponentStatus>& statuses);

    std::vector<std::shared_ptr<ICommandSource>> sources_;
    std::vector<std::shared_ptr<IController>> controllers_;
    std::shared_ptr<ICommandArbiter> arbiter_;
    std::shared_ptr<IActuatorPlant> plant_;
    std::vector<std::shared_ptr<IStatePublisher>> publishers_;
    std::optional<std::string> previous_selected_control_id_;
    std::vector<ActuatorCommand> previous_selected_commands_;
};

} // namespace hakoniwa::robot_runtime::runtime
