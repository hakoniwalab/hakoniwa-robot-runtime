#include "hakoniwa/robot_runtime/adapters/endpoint/joy_pdu_event_reader.hpp"

#include "hakoniwa/pdu/endpoint.hpp"
#include "sensor_msgs/pdu_cpptype_Joy.hpp"
#include "sensor_msgs/pdu_cpptype_conv_Joy.hpp"

#include <cstddef>
#include <mutex>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class JoyPduEventReader::Impl {
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
    std::optional<runtime::JoyState> pending;
    bool subscribed {false};
};

JoyPduEventReader::JoyPduEventReader(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name)))
{
}

JoyPduEventReader::~JoyPduEventReader() = default;

void JoyPduEventReader::subscribe()
{
    if (impl_->subscribed) {
        throw std::logic_error("Joy PDU event reader is already subscribed");
    }
    const auto resolved = ::hakoniwa::pdu::PduResolvedKey {
        impl_->key.robot,
        impl_->endpoint.get_pdu_channel_id(impl_->key),
    };
    if (resolved.channel_id < 0) {
        throw std::runtime_error("failed to resolve Joy PDU event channel: "
            + impl_->key.robot + "/" + impl_->key.pdu);
    }
    if (impl_->endpoint.set_recv_event(resolved) != HAKO_PDU_ERR_OK) {
        throw std::runtime_error("failed to register Joy PDU receive event");
    }
    impl_->endpoint.subscribe_on_recv_callback(
        resolved,
        [this](const ::hakoniwa::pdu::PduResolvedKey&,
               const std::span<const std::byte> payload) {
            HakoCpp_Joy pdu {};
            ::hako::pdu::msgs::sensor_msgs::Joy converter;
            std::vector<std::byte> copy(payload.begin(), payload.end());
            if (!converter.pdu2cpp(reinterpret_cast<char*>(copy.data()), pdu)) {
                return;
            }
            runtime::JoyState state;
            state.axes.assign(pdu.axes.begin(), pdu.axes.end());
            state.buttons.assign(pdu.buttons.begin(), pdu.buttons.end());
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->pending = std::move(state);
        });
    impl_->subscribed = true;
}

std::optional<runtime::JoyState> JoyPduEventReader::take_latest()
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto state = std::move(impl_->pending);
    impl_->pending.reset();
    return state;
}

void JoyPduEventReader::reset() noexcept
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->pending.reset();
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
