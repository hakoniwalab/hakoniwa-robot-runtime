#include "hakoniwa/robot_runtime/adapters/endpoint/float64_pdu_event_reader.hpp"

#include "hakoniwa/pdu/converter/std_msgs/float64.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "std_msgs/pdu_cpptype_Float64.hpp"
#include "std_msgs/pdu_cpptype_conv_Float64.hpp"

#include <cstddef>
#include <mutex>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

class Float64PduEventReader::Impl {
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
    std::optional<double> pending;
    bool subscribed {false};
};

Float64PduEventReader::Float64PduEventReader(
    ::hakoniwa::pdu::Endpoint& endpoint,
    std::string pdu_robot,
    std::string pdu_name)
    : impl_(std::make_unique<Impl>(
        endpoint, std::move(pdu_robot), std::move(pdu_name)))
{
}

Float64PduEventReader::~Float64PduEventReader() = default;

void Float64PduEventReader::subscribe()
{
    if (impl_->subscribed) {
        throw std::logic_error("Float64 PDU event reader is already subscribed");
    }
    const auto resolved = ::hakoniwa::pdu::PduResolvedKey {
        impl_->key.robot,
        impl_->endpoint.get_pdu_channel_id(impl_->key),
    };
    if (resolved.channel_id < 0) {
        throw std::runtime_error("failed to resolve Float64 PDU event channel: "
            + impl_->key.robot + "/" + impl_->key.pdu);
    }
    if (impl_->endpoint.set_recv_event(resolved) != HAKO_PDU_ERR_OK) {
        throw std::runtime_error("failed to register Float64 PDU receive event");
    }
    impl_->endpoint.subscribe_on_recv_callback(
        resolved,
        [this](const ::hakoniwa::pdu::PduResolvedKey&,
               const std::span<const std::byte> payload) {
            HakoCpp_Float64 pdu {};
            ::hako::pdu::msgs::std_msgs::Float64 converter;
            std::vector<std::byte> copy(payload.begin(), payload.end());
            if (!converter.pdu2cpp(reinterpret_cast<char*>(copy.data()), pdu)) {
                return;
            }
            const double value =
                ::hako::robots::pdu::converter::std_msgs::ToDouble(pdu);
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->pending = value;
        });
    impl_->subscribed = true;
}

std::optional<double> Float64PduEventReader::take_latest()
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto value = impl_->pending;
    impl_->pending.reset();
    return value;
}

void Float64PduEventReader::reset() noexcept
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->pending.reset();
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
