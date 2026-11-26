#pragma once

#include "hakoniwa/robot_runtime/adapters/endpoint/person_step_rpc_event_reader.hpp"
#include "runtime/service/robot_services.hpp"

#include <functional>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class PersonStepService final : public runtime::IRobotServices {
public:
    using ApplyCommand = std::function<std::uint32_t(const PersonStepCommand&)>;
    using ReadState = std::function<PersonStepState(std::uint32_t)>;
    PersonStepService(std::string person, ApplyCommand apply_command, ReadState read_state);

    bool on_request(std::size_t, std::span<const std::byte> request) override;
    runtime::RobotStepResult on_step(
        const runtime::RobotState&, const runtime::RuntimeStepContext&,
        std::span<runtime::RobotServiceSlot> slots) override;
    bool on_response(std::size_t, const runtime::RobotState&, const runtime::RuntimeStepContext&,
        std::span<std::byte> packet) override;
    void reset() override;

private:
    std::string person_;
    ApplyCommand apply_command_;
    ReadState read_state_;
    std::uint64_t step_ {0};
    std::uint32_t result_code_ {0};
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
