#pragma once

// Test-only: runs a one-service fake written against the old per-service hooks
// as a robot with exactly one service. The existing pipeline tests keep their
// fakes and assertions; only their fixtures change. Not product code.

#include "runtime/service/robot_services.hpp"

#include <memory>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace hakoniwa::robot_runtime::test_support {

struct OneServiceStepResult {
    runtime::AsyncServiceStepOutcome outcome {runtime::AsyncServiceStepOutcome::Pending};
    std::vector<runtime::ActuatorCommand> commands;
};

/** The old per-service hooks, kept only for the ported tests. */
class OneService {
public:
    virtual ~OneService() = default;
    virtual bool on_request(std::span<const std::byte> request) = 0;
    virtual OneServiceStepResult on_step(const runtime::RobotState& state,
        runtime::AsyncServiceState server_state, const runtime::RuntimeStepContext& context) = 0;
    virtual bool on_response(const runtime::RobotState& state,
        const runtime::RuntimeStepContext& context, std::span<std::byte> packet) = 0;
    virtual void on_cancel() {}
    virtual void reset() = 0;
};

/**
 * Calls the service's on_step only while its slot is not Idle, which is when
 * the old per-service controller called it. An Idle robot gives no commands.
 */
class SingleServiceRobot final : public runtime::IRobotServices {
public:
    explicit SingleServiceRobot(std::shared_ptr<OneService> service) : service_(std::move(service))
    {
        if (!service_) {
            throw std::invalid_argument("single service robot needs a service");
        }
    }

    bool on_request(std::size_t index, std::span<const std::byte> packet) override
    {
        check(index);
        return service_->on_request(packet);
    }

    void on_cancel(std::size_t index) override
    {
        check(index);
        service_->on_cancel();
    }

    runtime::RobotStepResult on_step(const runtime::RobotState& state,
        const runtime::RuntimeStepContext& context, std::span<runtime::RobotServiceSlot> slots) override
    {
        if (slots.size() != 1) {
            throw std::logic_error("single service robot got another slot count");
        }
        if (slots[0].state == runtime::AsyncServiceState::Idle) {
            return {};
        }
        auto result = service_->on_step(state, slots[0].state, context);
        slots[0].result = result.outcome;
        return {std::move(result.commands)};
    }

    bool on_response(std::size_t index, const runtime::RobotState& state,
        const runtime::RuntimeStepContext& context, std::span<std::byte> packet) override
    {
        check(index);
        return service_->on_response(state, context, packet);
    }

    void reset() override { service_->reset(); }

private:
    static void check(std::size_t index)
    {
        if (index != 0) {
            throw std::logic_error("single service robot got another service index");
        }
    }

    std::shared_ptr<OneService> service_;
};

} // namespace hakoniwa::robot_runtime::test_support
