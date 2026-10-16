#pragma once

#include "runtime/source/ackermann_drive_command_source.hpp"
#include "hakoniwa/robot_runtime/runner/lockstep_input.hpp"

#include <memory>
#include <optional>
#include <string>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class AckermannDrivePduEventReader final
    : public runtime::IAckermannDriveEventReader
    , public runner::ILockstepInput {
public:
    AckermannDrivePduEventReader(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name);
    ~AckermannDrivePduEventReader() override;

    void subscribe();
    /**
     * Lockstep mode instead of subscribe(): no receive callback; the receive
     * flag is checked and the PDU read synchronously (runner::ILockstepInput).
     */
    void subscribe_lockstep();
    [[nodiscard]] bool lockstep_ready() noexcept override;
    void lockstep_prepare() noexcept override;
    [[nodiscard]] std::optional<runtime::AckermannDriveCommand>
        take_latest() override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
