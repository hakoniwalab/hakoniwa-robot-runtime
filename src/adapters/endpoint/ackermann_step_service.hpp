#pragma once


#include "hakoniwa/robot_runtime/adapters/endpoint/ackermann_drive_rpc_event_reader.hpp"
#include "runtime/service/robot_services.hpp"
#include "runtime/source/scalar_pdu_command_source.hpp"

#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

/** A scalar input staged by the car service, consumed by its friction controller. */
class AckermannStepFrictionInput final : public runtime::IFloat64EventReader {
public:
    void stage(double value) noexcept { pending_ = value; }
    std::optional<double> take_latest() override { return std::exchange(pending_, std::nullopt); }
    void reset() noexcept override { pending_.reset(); }

private:
    std::optional<double> pending_;
};

class AckermannStepService final : public runtime::IRobotServices {
public:
    AckermannStepService(
        std::shared_ptr<AckermannDriveRpcEventReader> reader, std::string body_id,
        std::shared_ptr<AckermannStepFrictionInput> friction = {});

    bool on_request(std::size_t service, std::span<const std::byte> request) override;
    bool on_request(std::span<const std::byte> request);
    runtime::RobotStepResult on_step(
        const runtime::RobotState&, const runtime::RuntimeStepContext&,
        std::span<runtime::RobotServiceSlot> slots) override;
    bool on_response(
        std::size_t, const runtime::RobotState& state, const runtime::RuntimeStepContext&,
        std::span<std::byte> packet) override;
    void reset() override;

private:
    std::shared_ptr<AckermannDriveRpcEventReader> reader_;
    std::string body_id_;
    std::shared_ptr<AckermannStepFrictionInput> friction_;
    std::uint64_t step_ {0};
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
