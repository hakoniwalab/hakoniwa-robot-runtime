#pragma once

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR

#include "runtime/publisher/sensor_nav_sat_fix_publisher.hpp"

#include <memory>
#include <string>

namespace hakoniwa::pdu {
class Endpoint;
}

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

/**
 * Writes a NavSatFixOutput as sensor_msgs/NavSatFix: WGS-84 latitude and
 * longitude [deg], height above the ellipsoid [m]; status FIX with service
 * GPS, NO_FIX when the receiver has no fix; a diagonal covariance from the
 * reported accuracies, or unknown when the receiver reports none.
 */
class NavSatFixPduWriter final : public runtime::INavSatFixWriter {
public:
    NavSatFixPduWriter(
        ::hakoniwa::pdu::Endpoint& endpoint,
        std::string pdu_robot,
        std::string pdu_name);
    ~NavSatFixPduWriter() override;

    [[nodiscard]] bool send(const runtime::NavSatFixOutput& output) noexcept override;
    void reset() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa

#endif
