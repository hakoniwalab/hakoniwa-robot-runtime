#pragma once

#include "plugin/plugin_library.hpp"
#include "runtime/service/robot_services.hpp"

namespace hakoniwa::robot_runtime::runtime {

/** Adapts a single-service C ABI v1 plugin to the robot interface. */
class ServicePlugin final : public IRobotServices {
public:
    ServicePlugin(std::shared_ptr<plugin::PluginLibrary> library, std::string service_name,
        const std::string& config_json, std::vector<std::string> actuator_ids,
        std::uint64_t physics_step_ns, std::size_t metadata_size, std::size_t header_size);
    ~ServicePlugin() override;

    ServicePlugin(const ServicePlugin&) = delete;
    ServicePlugin& operator=(const ServicePlugin&) = delete;

    bool on_request(std::size_t, std::span<const std::byte> request) override;
    RobotStepResult on_step(const RobotState& state, const RuntimeStepContext& context,
        std::span<RobotServiceSlot> slots) override;
    bool on_response(std::size_t, const RobotState& state, const RuntimeStepContext& context,
        std::span<std::byte> packet) override;
    void on_cancel(std::size_t) override;
    void reset() override;

private:
    bool copy_states(const RobotState& state);
    HakoJointCommandsV1 prepare_commands();
    bool collect_commands(const RuntimeStepContext& context, std::vector<ActuatorCommand>& output) const;
    HakoTickV1 tick(const RuntimeStepContext& context, bool& first_call) const;
    std::string append_periodic_commands(const RobotState& state, const RuntimeStepContext& context,
        std::vector<ActuatorCommand>& output);

    std::shared_ptr<plugin::PluginLibrary> library_;
    const HakoServicePluginV1& table_;
    std::string service_name_;
    std::string controller_id_;
    std::vector<std::string> actuator_ids_;
    std::size_t metadata_size_;
    std::size_t header_size_;
    std::vector<double> position_, velocity_, effort_, command_value_;
    std::vector<std::uint8_t> has_command_, command_type_;
    bool first_step_ {true};
    bool first_response_ {true};
    bool first_update_ {true};
    void* instance_ {nullptr};
};

} // namespace hakoniwa::robot_runtime::runtime
