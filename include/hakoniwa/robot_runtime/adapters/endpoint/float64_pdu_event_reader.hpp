#pragma once

#include "runtime/source/scalar_pdu_command_source.hpp"
#include "hakoniwa/robot_runtime/runner/lockstep_input.hpp"

#include <memory>
#include <optional>
#include <string>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

/** Latest-wins receive-event adapter for a std_msgs/Float64 PDU channel. */
class Float64PduEventReader final
    : public runtime::IFloat64EventReader
    , public runner::ILockstepInput {
public:
    Float64PduEventReader(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name);
    ~Float64PduEventReader() override;

    void subscribe();
    /** Lockstep mode instead of subscribe() (runner::ILockstepInput). */
    void subscribe_lockstep();
    [[nodiscard]] bool lockstep_ready() noexcept override;
    void lockstep_prepare() noexcept override;
    [[nodiscard]] std::optional<double> take_latest() override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
