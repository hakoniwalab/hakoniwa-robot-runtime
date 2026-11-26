#pragma once

#include "single_service_robot.hpp"

namespace hakoniwa::robot_runtime::test_support {

/** Keeps the direct packet assertions for the built-in single-service robots. */
class SingleRobotServiceCalls {
public:
    explicit SingleRobotServiceCalls(runtime::IRobotServices& robot) : robot_(robot) {}

    bool on_request(std::span<const std::byte> request)
    {
        return robot_.on_request(0, request);
    }

    OneServiceStepResult on_step(const runtime::RobotState& state, runtime::AsyncServiceState service_state,
        const runtime::RuntimeStepContext& context)
    {
        runtime::RobotServiceSlot slot {service_state};
        auto result = robot_.on_step(state, context, std::span(&slot, 1));
        return {slot.result, std::move(result.commands)};
    }

    bool on_response(const runtime::RobotState& state, const runtime::RuntimeStepContext& context,
        std::span<std::byte> packet)
    {
        return robot_.on_response(0, state, context, packet);
    }

private:
    runtime::IRobotServices& robot_;
};

} // namespace hakoniwa::robot_runtime::test_support
