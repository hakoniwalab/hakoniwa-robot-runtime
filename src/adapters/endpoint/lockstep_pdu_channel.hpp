#pragma once

#include "hakoniwa/pdu/endpoint.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP) && HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP
#include "hako_asset.h"
#endif

#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

/**
 * The receive side of one lockstep input channel (runner::ILockstepInput),
 * shared by the PDU event readers.
 *
 * No Endpoint receive callback (it runs at the next step boundary, and its
 * registration races the writer's first write): a flag-type receive event,
 * registered at the asset's initialization, and a synchronous read when the
 * flag is set. The first step takes the value the writer wrote at its
 * initialization; every later step waits for a new value.
 */
class LockstepPduChannel {
public:
    LockstepPduChannel(::hakoniwa::pdu::Endpoint& endpoint, ::hakoniwa::pdu::PduKey key)
        : endpoint_(endpoint), key_(std::move(key))
    {
    }

    void enable()
    {
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP) && HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP
        resolved_ = {key_.robot, endpoint_.get_pdu_channel_id(key_)};
        if (resolved_.channel_id < 0) {
            throw std::runtime_error(
                "failed to resolve lockstep channel: " + key_.robot + "/" + key_.pdu);
        }
        enabled_ = true;
#else
        throw std::logic_error(
            "lockstep inputs are not compiled in (HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP)");
#endif
    }

    [[nodiscard]] bool enabled() const noexcept { return enabled_; }

    /** At the asset's initialization. */
    void prepare() noexcept
    {
        if (enabled_ && register_event()) {
            first_ = true;
        }
    }

    /**
     * True when the next step may run. A new value is handed to on_payload
     * (which returns false when it cannot decode it). Never blocks.
     */
    template <typename OnPayload>
    [[nodiscard]] bool ready(OnPayload&& on_payload) noexcept
    {
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP) && HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP
        if (!enabled_) {
            return true;
        }
        if (!event_registered_ && !waiting_) {
            if (!register_event()) {
                return true;  // not prepared at initialization and not yet possible
            }
            first_ = true;
        }
        if (first_) {
            (void)check_flag();
            if (read_now(on_payload)) {
                arrived_ = true;
            }
            first_ = false;
            waiting_ = true;
            return true;
        }
        if (arrived_ || !waiting_) {
            return true;
        }
        if (check_flag() && read_now(on_payload)) {
            arrived_ = true;
            return true;
        }
        return false;
#else
        (void)on_payload;
        return true;
#endif
    }

    /** The step took the value: the next step waits for a new one. */
    void consumed() noexcept { arrived_ = false; }

private:
    bool register_event() noexcept
    {
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP) && HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP
        if (!event_registered_) {
            int event_id = -1;
            event_registered_ = hako_asset_register_data_recv_event(
                key_.robot.c_str(), static_cast<HakoPduChannelIdType>(resolved_.channel_id),
                nullptr, &event_id) == 0;
        }
#endif
        return event_registered_;
    }

    bool check_flag() noexcept
    {
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP) && HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP
        // The flag is set when the writer finishes writing the PDU.
        return hako_asset_check_data_recv_event(
            key_.robot.c_str(), static_cast<HakoPduChannelIdType>(resolved_.channel_id)) == 0;
#else
        return false;
#endif
    }

    template <typename OnPayload>
    bool read_now(OnPayload& on_payload) noexcept
    {
        std::vector<std::byte> buffer(endpoint_.get_pdu_size(key_));
        size_t received = 0;
        if (endpoint_.recv(resolved_, buffer, received) != HAKO_PDU_ERR_OK) {
            return false;
        }
        return on_payload(std::span<const std::byte>(buffer.data(), received));
    }

    ::hakoniwa::pdu::Endpoint& endpoint_;
    ::hakoniwa::pdu::PduKey key_;
    ::hakoniwa::pdu::PduResolvedKey resolved_ {};
    bool enabled_ {false};
    bool event_registered_ {false};
    bool first_ {false};    // the next step is the first: take the initial value
    bool waiting_ {false};  // every step waits for a new value
    bool arrived_ {false};  // a new value has been read for the next step
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
