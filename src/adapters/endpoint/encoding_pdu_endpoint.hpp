#pragma once

#include "runtime/publisher/encoded_pdu_sink.hpp"

#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"

#include <cstddef>
#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

/**
 * Sends one typed PDU exactly as ::hakoniwa::pdu::TypedEndpoint::send does
 * (the type's Convertor encodes into a buffer of the PDU definition's size,
 * then Endpoint::send), and hands the same encoded bytes to an optional
 * IEncodedPduSink, so an in-process consumer gets them without encoding twice.
 */
template <typename CppType, typename Convertor>
class EncodingPduEndpoint final {
public:
    EncodingPduEndpoint(
        ::hakoniwa::pdu::Endpoint& endpoint,
        ::hakoniwa::pdu::PduKey key,
        std::shared_ptr<runtime::IEncodedPduSink> sink)
        : endpoint_(endpoint)
        , key_(std::move(key))
        , sink_(std::move(sink))
    {
    }

    /** sample_time_usec is passed to the sink with the bytes. */
    HakoPduErrorType send(const CppType& value, const std::uint64_t sample_time_usec) noexcept
    {
        try {
            ensure_capacity();
            if (buffer_.empty()) {
                return HAKO_PDU_ERR_UNSUPPORTED;
            }
            CppType copy = value;
            const int written = convertor_.cpp2pdu(
                copy,
                reinterpret_cast<char*>(buffer_.data()),
                static_cast<int>(buffer_.size()));
            if (written <= 0) {
                return HAKO_PDU_ERR_INVALID_CONFIG;
            }
            const std::span<const std::byte> encoded(
                buffer_.data(), static_cast<std::size_t>(written));
            if (sink_ != nullptr) {
                // The sink gets the sample even when the transport fails.
                sink_->store(encoded, sample_time_usec);
            }
            return endpoint_.send(key_, encoded);
        } catch (...) {
            return HAKO_PDU_ERR_INVALID_CONFIG;
        }
    }

    void reset() noexcept
    {
        if (sink_ != nullptr) {
            sink_->clear();
        }
    }

private:
    void ensure_capacity()
    {
        if (!buffer_.empty()) {
            return;
        }
        const std::size_t capacity = endpoint_.get_pdu_size(key_);
        if (capacity == 0) {
            return;
        }
        buffer_.assign(capacity, std::byte {0});
    }

    ::hakoniwa::pdu::Endpoint& endpoint_;
    ::hakoniwa::pdu::PduKey key_;
    std::shared_ptr<runtime::IEncodedPduSink> sink_;
    std::vector<std::byte> buffer_;
    ::hako::pdu::PduConvertor<CppType, Convertor> convertor_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
