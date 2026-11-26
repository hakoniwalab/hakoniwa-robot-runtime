#pragma once

#include "plugin/plugin_library.hpp"
#include "runtime/controller/controller.hpp"
#include "runtime/service/async_service.hpp"

namespace hakoniwa::robot_runtime::runtime {

/** The service and optional controller share this single plugin instance. */
class ServicePlugin final : public IAsyncService {
public:
    ServicePlugin(std::shared_ptr<plugin::PluginLibrary> library, std::string service_name,
        const std::string& config_json, std::vector<std::string> actuator_ids,
        std::uint64_t physics_step_ns, std::size_t metadata_size, std::size_t header_size);
    ~ServicePlugin() override;

    ServicePlugin(const ServicePlugin&) = delete;
    ServicePlugin& operator=(const ServicePlugin&) = delete;

    bool on_request(std::span<const std::byte> request) override;
    AsyncServiceStepResult on_step(const RobotState& state, AsyncServiceState server_state,
        const RuntimeStepContext& context) override;
    bool on_response(const RobotState& state, const RuntimeStepContext& context,
        std::span<std::byte> packet) override;
    void on_cancel() override;
    void reset() override;

    bool has_update() const noexcept;
    ControllerOutput update(const RobotState& state, const RuntimeStepContext& context);

private:
    bool copy_states(const RobotState& state);
    HakoJointCommandsV1 prepare_commands();
    bool collect_commands(const RuntimeStepContext& context, std::vector<ActuatorCommand>& output) const;
    HakoTickV1 tick(const RuntimeStepContext& context, bool& first_call) const;

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

/** Adds the periodic update to a service's existing controller candidate. */
class ServicePluginController final : public IController {
public:
    ServicePluginController(std::shared_ptr<IController> rpc, std::shared_ptr<ServicePlugin> plugin);
    std::string_view id() const noexcept override { return rpc_->id(); }
    std::string_view source_id() const noexcept override { return rpc_->source_id(); }
    bool accepts(const IControllerInput& input) const noexcept override { return rpc_->accepts(input); }
    bool error_stops_run() const noexcept override { return rpc_->error_stops_run(); }
    void reset(const RobotState& state) override { rpc_->reset(state); }
    ControllerOutput update(std::shared_ptr<const IControllerInput> input, const RobotState& state,
        const RuntimeStepContext& context) override;

private:
    std::shared_ptr<IController> rpc_;
    std::shared_ptr<ServicePlugin> plugin_;
};

} // namespace hakoniwa::robot_runtime::runtime
