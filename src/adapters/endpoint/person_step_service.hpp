#pragma once

#include "runtime/service/robot_services.hpp"

#include <optional>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class PersonStepService final : public runtime::IRobotServices {
public:
    explicit PersonStepService(std::string person);

    bool on_request(std::size_t, std::span<const std::byte> request) override;
    runtime::RobotStepResult on_step(
        const runtime::RobotState&, const runtime::RuntimeStepContext&,
        std::span<runtime::RobotServiceSlot> slots) override;
    bool on_response(std::size_t, const runtime::RobotState&, const runtime::RuntimeStepContext&,
        std::span<std::byte> packet) override;
    void reset() override;

private:
    std::string person_;
    std::optional<runtime::HumanoidMotionOperation> motion_;
    std::uint64_t step_ {0};
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
