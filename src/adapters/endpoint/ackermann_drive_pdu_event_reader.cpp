#include "hakoniwa/robot_runtime/adapters/endpoint/ackermann_drive_pdu_event_reader.hpp"

#include "ackermann_msgs/pdu_cpptype_AckermannDrive.hpp"
#include "ackermann_msgs/pdu_cpptype_conv_AckermannDrive.hpp"
#include "hakoniwa/pdu/endpoint.hpp"

#include <cstddef>
#include <mutex>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class AckermannDrivePduEventReader::Impl {
public:
    Impl(
        ::hakoniwa::pdu::Endpoint& endpoint_value,
        std::string pdu_robot,
        std::string pdu_name)
        : endpoint(endpoint_value), key {std::move(pdu_robot), std::move(pdu_name)}
    {
    }

    ::hakoniwa::pdu::Endpoint& endpoint;
    ::hakoniwa::pdu::PduKey key;
    std::mutex mutex;
    std::optional<runtime::AckermannDriveCommand> pending;
    bool subscribed {false};
};

AckermannDrivePduEventReader::AckermannDrivePduEventReader(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name)))
{
}

AckermannDrivePduEventReader::~AckermannDrivePduEventReader() = default;

void AckermannDrivePduEventReader::subscribe()
{
    if (impl_->subscribed) {
        throw std::logic_error(
            "AckermannDrive PDU event reader is already subscribed");
    }
    const auto resolved = ::hakoniwa::pdu::PduResolvedKey {
        impl_->key.robot,
        impl_->endpoint.get_pdu_channel_id(impl_->key),
    };
    if (resolved.channel_id < 0) {
        throw std::runtime_error(
            "failed to resolve AckermannDrive PDU event channel: "
            + impl_->key.robot + "/" + impl_->key.pdu);
    }
    if (impl_->endpoint.set_recv_event(resolved) != HAKO_PDU_ERR_OK) {
        throw std::runtime_error(
            "failed to register AckermannDrive PDU receive event");
    }
    impl_->endpoint.subscribe_on_recv_callback(
        resolved,
        [this](const ::hakoniwa::pdu::PduResolvedKey&,
               const std::span<const std::byte> payload) {
            HakoCpp_AckermannDrive pdu {};
            ::hako::pdu::msgs::ackermann_msgs::AckermannDrive converter;
            std::vector<std::byte> copy(payload.begin(), payload.end());
            if (!converter.pdu2cpp(
                    reinterpret_cast<char*>(copy.data()), pdu)) {
                return;
            }
            runtime::AckermannDriveCommand command {
                pdu.steering_angle,
                pdu.steering_angle_velocity,
                pdu.speed,
                pdu.acceleration,
                pdu.jerk,
            };
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->pending = command;
        });
    impl_->subscribed = true;
}

std::optional<runtime::AckermannDriveCommand>
AckermannDrivePduEventReader::take_latest()
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto value = impl_->pending;
    impl_->pending.reset();
    return value;
}

void AckermannDrivePduEventReader::reset() noexcept
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->pending.reset();
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
