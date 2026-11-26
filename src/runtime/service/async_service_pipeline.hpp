#pragma once


#include "hakoniwa/robot_runtime/runner/lockstep_input.hpp"
#include "runtime/controller/controller.hpp"
#include "runtime/publisher/state_publisher.hpp"
#include "runtime/service/robot_services.hpp"
#include "runtime/service/async_service_server.hpp"

#include <optional>

namespace hakoniwa::robot_runtime::runtime {

class AsyncServiceCommandSource final : public ICommandSource, public runner::ILockstepInput {
public:
    AsyncServiceCommandSource(
        std::string id, int service_id, std::shared_ptr<IAsyncServiceServer> server,
        std::shared_ptr<IRobotServices> robot, std::size_t service_index, bool lockstep = false);

    std::string_view id() const noexcept override;
    CommandSourcePollResult poll(const RuntimeStepContext& context) override;
    void reset() noexcept override;
    bool lockstep_ready() noexcept override;
    void lockstep_poll_while_paused() noexcept override;
    void lockstep_prepare() noexcept override;
    void lockstep_reset() noexcept override { lockstep_prepare(); }
    std::string_view lockstep_failure_reason() const noexcept override { return lockstep_failure_; }
    bool lockstep_uses_deadline() const noexcept override { return lockstep_; }
    bool lockstep_needs_initial_sync() const noexcept override { return false; }

private:
    friend class RobotServiceController;
    friend class AsyncServiceResponsePublisher;

    std::string id_;
    int service_id_;
    std::shared_ptr<IAsyncServiceServer> server_;
    std::shared_ptr<IRobotServices> robot_;
    std::size_t service_index_;
    bool lockstep_;
    std::optional<std::vector<std::byte>> staged_request_;
    std::string lockstep_failure_;
    AsyncServiceState expected_state_ {AsyncServiceState::Idle};
    std::optional<AsyncServiceResultCode> completion_;
    std::uint64_t sequence_ {0};
};

class RobotServiceSource final : public ICommandSource {
public:
    RobotServiceSource(std::string id, std::vector<std::shared_ptr<AsyncServiceCommandSource>> services);

    std::string_view id() const noexcept override;
    CommandSourcePollResult poll(const RuntimeStepContext& context) override;
    void reset() noexcept override;

private:
    friend class RobotServiceController;

    std::string id_;
    std::vector<std::shared_ptr<AsyncServiceCommandSource>> services_;
    std::uint64_t sequence_ {0};
};

class RobotServiceController final : public IController {
public:
    RobotServiceController(
        std::string id, std::shared_ptr<RobotServiceSource> source, std::shared_ptr<IRobotServices> robot);

    std::string_view id() const noexcept override;
    std::string_view source_id() const noexcept override;
    bool accepts(const IControllerInput& input) const noexcept override;
    bool error_stops_run() const noexcept override;
    ControllerOutput update(
        std::shared_ptr<const IControllerInput> input, const RobotState& state,
        const RuntimeStepContext& context) override;
    void reset(const RobotState& state) override;

private:
    std::string id_;
    std::shared_ptr<RobotServiceSource> source_;
    std::shared_ptr<IRobotServices> robot_;
    std::vector<RobotServiceSlot> slots_;
};

class AsyncServiceResponsePublisher final : public IStatePublisher {
public:
    AsyncServiceResponsePublisher(
        std::string id, std::shared_ptr<AsyncServiceCommandSource> source);

    std::string_view id() const noexcept override;
    ComponentStatus publish(
        const RobotState& state, const RuntimeStepContext& context) noexcept override;
    void reset() noexcept override;

private:
    std::string id_;
    std::shared_ptr<AsyncServiceCommandSource> source_;
};

} // namespace hakoniwa::robot_runtime::runtime
