#pragma once

#include "runtime/source/ackermann_drive_command_source.hpp"

#include <memory>
#include <optional>
#include <string>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class AckermannDrivePduEventReader final
    : public runtime::IAckermannDriveEventReader {
public:
    AckermannDrivePduEventReader(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name);
    ~AckermannDrivePduEventReader() override;

    void subscribe();
    [[nodiscard]] std::optional<runtime::AckermannDriveCommand>
        take_latest() override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
