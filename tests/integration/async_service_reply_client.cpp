#include "hakoniwa_asset_polling.h"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsResponsePacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_ServiceResponseHeader.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <span>
#include <string_view>
#include <thread>
#include <vector>

namespace {

constexpr hako_time_t kDeltaUsec = 1'000;
constexpr int kStateRunning = 2;
constexpr auto kAsset = "AsyncServiceReplyClient";
constexpr auto kService = "Service/AsyncA";
constexpr auto kClient = "AsyncServiceReplyClientRpc";

struct Call {
    std::int64_t target;
    std::int64_t addend;
    int expected_code;
    std::int64_t expected_sum;
};

constexpr std::array kCalls {
    Call {10, 1, HAKO_SERVICE_API_RESULT_CODE_OK, 12},
    Call {20, 2, HAKO_SERVICE_API_RESULT_CODE_OK, 23},
    Call {30, 3, HAKO_SERVICE_API_RESULT_CODE_OK, 34},
    Call {-1, 4, HAKO_SERVICE_API_RESULT_CODE_INVALID, 0},
};
constexpr std::array kLongCall {Call {10, 1, HAKO_SERVICE_API_RESULT_CODE_OK, 16}};
constexpr std::array kCancelCall {Call {10, 1, HAKO_SERVICE_API_RESULT_CODE_CANCELED, 10}};
constexpr Call kBusyProbe {80, 9, HAKO_SERVICE_API_RESULT_CODE_BUSY, 0};

bool send(HakoServiceHandleType& handle, const Call& call, std::uint32_t& request_id)
{
    char* packet = nullptr;
    std::size_t size = 0;
    auto converter = hako::pdu::msgs::hako_srv_msgs::AddTwoIntsRequestPacket {};
    HakoCpp_AddTwoIntsRequestPacket request {};
    if (hako_asset_service_client_get_request_buffer(
            &handle, &packet, &size, HAKO_SERVICE_CLIENT_API_OPCODE_REQUEST, -1) != 0
        || !converter.pdu2cpp(packet, request)) {
        return false;
    }
    request.body.a = call.target;
    request.body.b = call.addend;
    request_id = request.header.request_id;
    return converter.cpp2pdu(request, packet, static_cast<int>(size)) >= 0
        && hako_asset_service_client_call_request(&handle, packet, size, 30000) == 0;
}

bool check_reply(HakoServiceHandleType& handle, const Call& call, std::uint32_t request_id)
{
    char* packet = nullptr;
    std::size_t size = 0;
    HakoCpp_ServiceResponseHeader header {};
    if (hako_asset_service_client_get_response(&handle, &packet, &size, 0) != 0
        || !hako::pdu::msgs::hako_srv_msgs::ServiceResponseHeader {}.pdu2cpp(packet, header)) {
        std::cerr << "CLIENT ERROR: response header could not be decoded\n";
        return false;
    }
    std::cout << "CLIENT reply id=" << header.request_id << " status=" << int(header.status)
              << " code=" << header.result_code << std::endl;
    if (header.request_id != request_id || header.service_name != kService
        || header.client_name != kClient || header.status != HAKO_SERVICE_API_STATUS_DONE
        || header.result_code != call.expected_code) {
        return false;
    }
    if (header.result_code == HAKO_SERVICE_API_RESULT_CODE_INVALID) {
        return true; // Core may reuse the preceding reply's body.
    }
    HakoCpp_AddTwoIntsResponsePacket response {};
    if (!hako::pdu::msgs::hako_srv_msgs::AddTwoIntsResponsePacket {}.pdu2cpp(packet, response)) {
        return false;
    }
    std::cout << "CLIENT body id=" << header.request_id << " sum=" << response.body.sum << std::endl;
    return response.body.sum == call.expected_sum;
}

bool observe_busy(HakoServiceHandleType& handle, std::uint32_t request_id)
{
    // The high-level poll consumes DOING/BUSY as EVENT_NONE. Inspect its
    // response channel without consuming the event used by that poll.
    int request_channel = -1;
    int response_channel = -1;
    char* ignored = nullptr;
    std::size_t size = 0;
    if (hako_asset_service_client_get_channel_id_by_handle(
            &handle, &request_channel, &response_channel) != 0
        || hako_asset_service_client_get_response(&handle, &ignored, &size, 0) != 0) {
        return false;
    }
    std::vector<char> packet(size);
    HakoCpp_ServiceResponseHeader header {};
    if (hakoniwa_asset_read_pdu_nolock(kService, response_channel, packet.data(), size) != 0
        || hako_get_base_ptr_pdu(packet.data()) == nullptr
        || !hako::pdu::msgs::hako_srv_msgs::ServiceResponseHeader {}.pdu2cpp(
            packet.data(), header)
        || header.status != HAKO_SERVICE_API_STATUS_DOING
        || header.result_code != HAKO_SERVICE_API_RESULT_CODE_BUSY
        || header.request_id != request_id || header.service_name != kService
        || header.client_name != kClient) {
        return false;
    }
    std::cout << "CLIENT BUSY id=" << header.request_id << std::endl;
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <service.json> <zero|long|busy|cancel>\n";
        return 2;
    }
    const std::string_view mode = argv[2];
    const bool busy_case = mode == "busy";
    const bool cancel_case = mode == "cancel";
    std::span<const Call> calls = kCalls;
    if (cancel_case) {
        calls = kCancelCall;
    } else if (busy_case || mode == "long") {
        calls = kLongCall;
    } else if (mode != "zero") {
        return 2;
    }
    if (hakoniwa_asset_init() != 0 || hakoniwa_simevent_init() != 0
        || hakoniwa_asset_register_polling(kAsset) != 0
        || hako_asset_service_initialize(argv[1]) != 0) {
        return 1;
    }
    std::cout << "CLIENT registered" << std::endl;

    HakoServiceHandleType handle {-1, -1};
    bool active = false;
    bool waiting = false;
    std::size_t next = 0;
    std::uint32_t request_id = 0;
    hako_time_t local_time = 0;
    hako_time_t sent_at = 0;
    bool followup_sent = false;
    bool saw_busy = false;
    for (;;) {
        if (busy_case && followup_sent && !saw_busy) {
            saw_busy = observe_busy(handle, request_id);
        }
        const int event = hakoniwa_asset_get_event(kAsset);
        if (event == 1) {
            if (!hakoniwa_asset_is_pdu_created()
                || hako_asset_service_client_create(kAsset, kService, kClient, &handle) != 0) {
                return 1;
            }
            active = true;
            hakoniwa_asset_notify_write_pdu_done(kAsset);
            if (hakoniwa_asset_start_feedback_ok(kAsset) != 0) {
                return 1;
            }
        } else if (event == 2) {
            return hakoniwa_asset_stop_feedback_ok(kAsset) == 0 && next == calls.size() ? 0 : 1;
        } else if (event == 3 || event == 4) {
            return 1;
        }

        if (active && hakoniwa_simevent_get_state() == kStateRunning) {
            const auto world_time = hakoniwa_asset_get_worldtime();
            if (waiting) {
                const int service_event = hako_asset_service_client_poll(&handle);
                const auto expected_event = cancel_case
                    ? HAKO_SERVICE_CLIENT_API_REQUEST_CANCEL_DONE
                    : HAKO_SERVICE_CLIENT_API_RESPONSE_IN;
                if (service_event == expected_event) {
                    if (!check_reply(handle, calls[next], request_id) || (busy_case && !saw_busy)) {
                        std::cerr << "CLIENT ERROR: unexpected reply for request "
                                  << request_id << std::endl;
                        return 1;
                    }
                    waiting = false;
                    if (++next == calls.size()) {
                        std::cout << "CLIENT done" << std::endl;
                    }
                } else if (service_event != HAKO_SERVICE_CLIENT_API_EVENT_NONE) {
                    std::cerr << "CLIENT ERROR: unexpected poll event " << service_event << '\n';
                    return 1;
                }
            }
            if (waiting && (busy_case || cancel_case) && !followup_sent
                && world_time >= sent_at + 2 * kDeltaUsec) {
                std::uint32_t ignored_id = 0;
                const bool sent = busy_case ? send(handle, kBusyProbe, ignored_id)
                    : hako_asset_service_client_cancel_request(&handle) == 0;
                if (!sent) {
                    return 1;
                }
                followup_sent = true;
                std::cout << "CLIENT sent " << mode << " followup" << std::endl;
            }
            if (!waiting && next < calls.size()) {
                if (!send(handle, calls[next], request_id)) {
                    return 1;
                }
                waiting = true;
                sent_at = world_time;
            }
            if (local_time <= hakoniwa_asset_get_worldtime()) {
                local_time += kDeltaUsec;
                hakoniwa_asset_notify_simtime(kAsset, local_time);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
