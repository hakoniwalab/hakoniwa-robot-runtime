#include "adapters/endpoint/robot_pdu_event_reader.hpp"

#include "hakoniwa/pdu/endpoint.hpp"
#include "lockstep_pdu_channel.hpp"

#include <cstddef>
#include <mutex>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class RobotPduEventReader::Impl {
public:
    Impl(
        ::hakoniwa::pdu::Endpoint& endpoint_value,
        std::string pdu_robot,
        std::string pdu_name, Validator validator)
        : endpoint(endpoint_value), key {std::move(pdu_robot), std::move(pdu_name)}, validate(validator)
    {
    }

    ::hakoniwa::pdu::Endpoint& endpoint;
    ::hakoniwa::pdu::PduKey key;
    std::mutex mutex;
    Validator validate;
    std::optional<std::vector<std::byte>> pending;
    bool subscribed {false};

    bool stage(std::span<const std::byte> payload)
    {
        if (validate && !validate(payload)) {
            return false;
        }
        pending.emplace(payload.begin(), payload.end());
        return true;
    }

    LockstepPduChannel lockstep {endpoint, key};
};

RobotPduEventReader::RobotPduEventReader(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name, Validator validate)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name), validate))
{
}

RobotPduEventReader::~RobotPduEventReader() = default;

void RobotPduEventReader::subscribe()
{
    if (impl_->subscribed) {
        throw std::logic_error(
            "robot PDU event reader is already subscribed");
    }
    const auto resolved = ::hakoniwa::pdu::PduResolvedKey {
        impl_->key.robot,
        impl_->endpoint.get_pdu_channel_id(impl_->key),
    };
    if (resolved.channel_id < 0) {
        throw std::runtime_error(
            "failed to resolve robot PDU event channel: "
            + impl_->key.robot + "/" + impl_->key.pdu);
    }
    if (impl_->endpoint.set_recv_event(resolved) != HAKO_PDU_ERR_OK) {
        throw std::runtime_error(
            "failed to register robot PDU receive event");
    }
    impl_->endpoint.subscribe_on_recv_callback(
        resolved,
        [this](const ::hakoniwa::pdu::PduResolvedKey&,
               const std::span<const std::byte> payload) {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            (void)impl_->stage(payload);
        });
    impl_->subscribed = true;
}

void RobotPduEventReader::subscribe_lockstep()
{
    if (impl_->subscribed) {
        throw std::logic_error(
            "robot PDU event reader is already subscribed");
    }
    impl_->lockstep.enable();
    impl_->subscribed = true;
}

void RobotPduEventReader::lockstep_prepare() noexcept
{
    impl_->lockstep.prepare();
}

bool RobotPduEventReader::lockstep_ready() noexcept
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->lockstep.ready([this](std::span<const std::byte> payload) {
        return impl_->stage(payload);
    });
}

std::optional<runtime::RobotPduSample>
RobotPduEventReader::take_latest(std::uint64_t poll_time_usec)
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto value = std::exchange(impl_->pending, std::nullopt);
    impl_->lockstep.consumed();  // lockstep: the next step waits for the next value
    if (!value) {
        return std::nullopt;
    }
    return runtime::RobotPduSample {std::move(*value), poll_time_usec};
}

void RobotPduEventReader::reset() noexcept
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->pending.reset();
    impl_->lockstep.consumed();
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
