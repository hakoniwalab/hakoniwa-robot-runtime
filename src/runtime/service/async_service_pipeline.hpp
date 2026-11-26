#pragma once

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE

#include "runtime/controller/controller.hpp"
#include "runtime/publisher/state_publisher.hpp"
#include "runtime/service/async_service.hpp"
#include "runtime/service/async_service_server.hpp"

#include <optional>

namespace hakoniwa::robot_runtime::runtime {

class AsyncServiceCommandSource final : public ICommandSource {
public:
    AsyncServiceCommandSource(
        std::string id, int service_id, std::shared_ptr<IAsyncServiceServer> server,
        std::shared_ptr<IAsyncService> service,
        bool lockstep = false);

    std::string_view id() const noexcept override;
    CommandSourcePollResult poll(const RuntimeStepContext& context) override;
    void reset() noexcept override;

private:
    friend class AsyncServiceLockstepGate;
    friend class AsyncServiceController;
    friend class AsyncServiceResponsePublisher;

    std::string id_;
    int service_id_;
    std::shared_ptr<IAsyncServiceServer> server_;
    std::shared_ptr<IAsyncService> service_;
    bool lockstep_;
    std::optional<std::vector<std::byte>> staged_request_;
    AsyncServiceState expected_state_ {AsyncServiceState::Idle};
    std::optional<AsyncServiceResultCode> completion_;
    std::uint64_t sequence_ {0};
};

class AsyncServiceController final : public IController {
public:
    AsyncServiceController(
        std::string id, std::shared_ptr<AsyncServiceCommandSource> source);

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
    std::shared_ptr<AsyncServiceCommandSource> source_;
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

#endif
