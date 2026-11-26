#pragma once

#include "plugin/plugin_library.hpp"
#include "runtime/service/robot_services.hpp"

namespace hakoniwa::robot_runtime::runtime {

struct RobotActuatorSpec {
    std::string id;
    ActuatorCommandType command_type;
    ActuatorLimits limits;
};

struct RobotServiceSpec {
    std::string name;
    std::string type;
    bool lockstep {false};
};

struct ServicePluginConfig {
    std::string robot;
    std::vector<RobotActuatorSpec> actuators;
    std::vector<std::string> bodies;
    std::vector<std::string> humanoids;
    std::vector<std::string> pdus;
    std::vector<RobotServiceSpec> services;
    std::string config_json {"{}"};
    std::uint64_t physics_step_ns {0};
};

/** Adapts one C ABI v1 robot instance and its services to the runtime. */
class ServicePlugin final : public IRobotServices {
public:
    ServicePlugin(std::shared_ptr<plugin::PluginLibrary> library, ServicePluginConfig config);
    ~ServicePlugin() override;

    ServicePlugin(const ServicePlugin&) = delete;
    ServicePlugin& operator=(const ServicePlugin&) = delete;

    bool on_request(std::size_t service, std::span<const std::byte> request) override;
    RobotStepResult on_step(const RobotState& state, const RuntimeStepContext& context,
        std::span<RobotServiceSlot> slots) override;
    bool on_response(std::size_t service, const RobotState& state, const RuntimeStepContext& context,
        std::span<std::byte> packet) override;
    void on_cancel(std::size_t service) override;
    void reset() override;

private:
    std::string copy_states(const RobotState& state);
    HakoRobotStateV1 state_view() const;
    HakoJointCommandsV1 prepare_commands();
    bool collect_commands(const RuntimeStepContext& context, std::vector<ActuatorCommand>& output) const;
    HakoTickV1 tick(const RuntimeStepContext& context, bool& first_call) const;

    std::shared_ptr<plugin::PluginLibrary> library_;
    const HakoServicePluginV1& table_;
    ServicePluginConfig config_;
    std::string controller_id_;
    std::vector<HakoBodyStateV1> bodies_;
    std::vector<HakoHumanoidStateV1> humanoids_;
    std::vector<HakoServiceSlotV1> slots_;
    std::vector<double> position_, velocity_, effort_, command_value_;
    std::vector<std::uint8_t> has_command_, command_type_;
    bool first_step_ {true};
    bool first_response_ {true};
    HakoRobotPlugin* instance_ {nullptr};
};

} // namespace hakoniwa::robot_runtime::runtime
