#pragma once

#include "hakoniwa/robot_runtime/runner/lockstep_input.hpp"
#include "runtime/service/robot_pdu_reader.hpp"

#include <memory>
#include <optional>
#include <span>
#include <string>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

/** Latest-wins receive-event adapter for an input PDU of a robot plugin. */
class RobotPduEventReader final
    : public runtime::IRobotPduReader
    , public runner::ILockstepInput {
public:
    using Validator = bool (*)(std::span<const std::byte>);

    RobotPduEventReader(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name,
        Validator validate = nullptr);
    ~RobotPduEventReader() override;

    void subscribe();
    /**
     * Lockstep mode instead of subscribe(): no receive callback; the receive
     * flag is checked and the PDU read synchronously (runner::ILockstepInput).
     */
    void subscribe_lockstep();
    [[nodiscard]] bool lockstep_ready() noexcept override;
    void lockstep_prepare() noexcept override;
    [[nodiscard]] std::optional<runtime::RobotPduSample>
        take_latest(std::uint64_t poll_time_usec) override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
