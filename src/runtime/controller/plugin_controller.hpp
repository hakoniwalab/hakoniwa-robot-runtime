#pragma once

#include "plugin/plugin_library.hpp"
#include "runtime/controller/controller.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

/** Where a controller plugin's joint states come from. */
enum class PluginJointStateSource {
    /** RobotState.actuators: the Plant's true joint states. */
    Truth,
    /**
     * Joint sensor readings (encoder values with noise and sample rate).
     * TODO: not implemented yet. It needs the sensor stage's joint readings
     * mapped to the actuators; PluginController refuses it for now.
     */
    Observed,
};

struct PluginControllerOptions {
    std::string controller_id;
    std::shared_ptr<plugin::PluginLibrary> library;
    /** The plugin's own configuration file; may be empty. */
    std::string plugin_config_path;
    /** Actuator IDs passed as joint_names, in this order. */
    std::vector<std::string> actuator_ids;
    std::uint64_t physics_step_ns {0};
    PluginJointStateSource joint_state_source {PluginJointStateSource::Truth};
};

/**
 * Controller backed by a controller plugin (controller_plugin.h).
 *
 * It has no CommandSource (empty source_id): the plugin receives its own
 * input, for example through ROS inside the plugin. The plugin's update runs
 * only on the steps whose step_index is a multiple of steps_per_update()
 * (the plugin's control period divided by the physics step). On the other
 * steps the commands of the last update are returned again, restamped for
 * the current step.
 *
 * A failing plugin call yields ComponentState::Error with the plugin's
 * message; the last commands are kept (the Arbiter ignores them while the
 * status is Error) and the next successful update clears the error.
 *
 * The instance is created in the constructor and destroyed in the
 * destructor; the PluginLibrary stays loaded while this object exists.
 */
class PluginController final : public IController {
public:
    /** @throws std::invalid_argument or std::runtime_error when create fails. */
    explicit PluginController(PluginControllerOptions options);
    ~PluginController() override;

    PluginController(const PluginController&) = delete;
    PluginController& operator=(const PluginController&) = delete;

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] std::string_view source_id() const noexcept override;
    [[nodiscard]] bool accepts(const IControllerInput& input) const noexcept override;
    [[nodiscard]] ControllerOutput update(
        std::shared_ptr<const IControllerInput> input,
        const RobotState& current_state,
        const RuntimeStepContext& context) override;
    void reset(const RobotState& current_state) override;

    /**
     * Start and stop events of the plugin lifecycle.
     * TODO: IController has no start/stop yet; the runner must call these
     * (design 5.2 (4)). Until then on_start/on_stop are not called by the
     * Runtime. Failures are returned as a status.
     */
    [[nodiscard]] ComponentStatus start();
    [[nodiscard]] ComponentStatus stop();

    /** Physics steps per plugin update (1 = every step). */
    [[nodiscard]] std::uint64_t steps_per_update() const noexcept;

private:
    static void log_to_runtime(void* user, std::int32_t level, const char* message);

    [[nodiscard]] bool copy_joint_states(
        const RobotState& current_state,
        std::string& error);
    [[nodiscard]] ControllerOutput repeat_last_output(
        const RuntimeStepContext& context) const;
    [[nodiscard]] ControllerOutput fail(
        const RuntimeStepContext& context,
        const std::string& detail);
    [[nodiscard]] ActuatorCommand stamped(
        const ActuatorCommand& command,
        const RuntimeStepContext& context) const;

    std::string controller_id_;
    std::shared_ptr<plugin::PluginLibrary> library_;
    const HakoControllerPluginV1* table_ {nullptr};
    void* instance_ {nullptr};
    std::vector<std::string> actuator_ids_;
    std::uint64_t physics_step_ns_ {0};
    std::uint64_t steps_per_update_ {1};
    PluginJointStateSource joint_state_source_ {PluginJointStateSource::Truth};
    bool just_reset_ {true};

    // Buffers lent to the plugin on each update (one element per joint).
    std::vector<double> position_;
    std::vector<double> velocity_;
    std::vector<double> effort_;
    std::vector<std::uint8_t> has_command_;
    std::vector<std::uint8_t> command_type_;
    std::vector<double> command_value_;

    // Result of the last update, repeated on the steps in between.
    ComponentStatus last_status_;
    std::vector<ActuatorCommand> last_commands_;
};

} // namespace hakoniwa::robot_runtime::runtime
