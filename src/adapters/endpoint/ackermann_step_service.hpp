#pragma once

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE

#include "hakoniwa/robot_runtime/adapters/endpoint/ackermann_drive_rpc_event_reader.hpp"
#include "runtime/service/async_service.hpp"

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class AckermannStepService final : public runtime::IAsyncService {
public:
    AckermannStepService(
        std::shared_ptr<AckermannDriveRpcEventReader> reader, std::string body_id);

    bool on_request(std::span<const std::byte> request) override;
    runtime::AsyncServiceStepResult on_step(
        const runtime::RobotState&, runtime::AsyncServiceState,
        const runtime::RuntimeStepContext&) override;
    bool on_response(
        const runtime::RobotState& state, const runtime::RuntimeStepContext&,
        std::span<std::byte> packet) override;
    void reset() override;

private:
    std::shared_ptr<AckermannDriveRpcEventReader> reader_;
    std::string body_id_;
    std::uint64_t step_ {0};
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
