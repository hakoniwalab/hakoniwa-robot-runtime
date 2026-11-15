// Stage A of docs/design-async-rpc.md: the Core client side of the cancel test.
//
// 1. request a=1 b=7 (held by the server), wait kHoldSteps steps, cancel it,
//    expect REQUEST_CANCEL_DONE;
// 2. request a=0 b=9, expect RESPONSE_IN with result OK and sum = 9, which
//    shows the server is idle again after the cancel.
#include "hakoniwa_asset_polling.h"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsResponsePacket.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

namespace {
constexpr hako_time_t kDeltaUsec = 1000;
constexpr int kStateRunning = 2;
constexpr int kHoldSteps = 20;
constexpr const char* kAsset = "AsyncRpcCancelClient";
constexpr const char* kService = "Service/AsyncA";

enum class Phase { send_held, hold, canceling, send_ok, wait_ok, done };
}

int main(int argc, char** argv)
{
    std::cout << std::unitbuf;
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <service.json>" << std::endl;
        return 2;
    }
    if (hakoniwa_asset_init() != 0 || hakoniwa_simevent_init() != 0
        || hakoniwa_asset_register_polling(kAsset) != 0
        || hako_asset_service_initialize(argv[1]) != 0) return 1;
    std::cout << "CLIENT registered" << std::endl;

    auto request_converter = hako::pdu::msgs::hako_srv_msgs::AddTwoIntsRequestPacket {};
    auto response_converter = hako::pdu::msgs::hako_srv_msgs::AddTwoIntsResponsePacket {};
    auto send = [&](HakoServiceHandleType& handle, std::int64_t a, std::int64_t b) {
        char* packet = nullptr;
        std::size_t packet_len = 0;
        HakoCpp_AddTwoIntsRequestPacket request {};
        if (hako_asset_service_client_get_request_buffer(
                &handle, &packet, &packet_len, HAKO_SERVICE_CLIENT_API_OPCODE_REQUEST, -1) != 0
            || !request_converter.pdu2cpp(packet, request)) return false;
        request.body.a = a;
        request.body.b = b;
        if (request_converter.cpp2pdu(request, packet, static_cast<int>(packet_len)) < 0
            || hako_asset_service_client_call_request(&handle, packet, packet_len, 30000) != 0) {
            return false;
        }
        std::cout << "CLIENT sent a=" << a << " b=" << b << std::endl;
        return true;
    };

    HakoServiceHandleType handle {-1, -1};
    bool valid = false;
    Phase phase = Phase::send_held;
    int held_steps = 0;
    hako_time_t local_time = 0;
    for (;;) {
        const int event = hakoniwa_asset_get_event(kAsset);
        if (event == 1) {
            if (!hakoniwa_asset_is_pdu_created()
                || hako_asset_service_client_create(
                    kAsset, kService, "AsyncRpcCancelClientRpc", &handle) != 0) return 1;
            valid = true;
            hakoniwa_asset_notify_write_pdu_done(kAsset);
            if (hakoniwa_asset_start_feedback_ok(kAsset) != 0) return 1;
        } else if (event == 2) {
            if (hakoniwa_asset_stop_feedback_ok(kAsset) != 0) return 1;
            return phase == Phase::done ? 0 : 1;
        } else if (event == 3) {
            if (hakoniwa_asset_reset_feedback_ok(kAsset) != 0) return 1;
            return 1;
        } else if (event == 4) return 1;

        if (valid && hakoniwa_simevent_get_state() == kStateRunning
            && hakoniwa_asset_is_pdu_created()) {
            const int service_event = phase == Phase::send_held || phase == Phase::send_ok
                    || phase == Phase::done
                ? HAKO_SERVICE_CLIENT_API_EVENT_NONE
                : hako_asset_service_client_poll(&handle);
            if (service_event == HAKO_SERVICE_CLIENT_API_REQUEST_TIMEOUT || service_event < 0) {
                std::cerr << "ERROR: client poll event=" << service_event << std::endl;
                return 1;
            }
            switch (phase) {
            case Phase::send_held:
                if (!send(handle, 1, 7)) return 1;
                phase = Phase::hold;
                break;
            case Phase::hold:
                if (service_event != HAKO_SERVICE_CLIENT_API_EVENT_NONE) {
                    std::cerr << "ERROR: held request answered, event=" << service_event
                              << std::endl;
                    return 1;
                }
                if (++held_steps >= kHoldSteps) {
                    if (hako_asset_service_client_cancel_request(&handle) != 0) return 1;
                    std::cout << "CLIENT cancel sent" << std::endl;
                    phase = Phase::canceling;
                }
                break;
            case Phase::canceling:
                if (service_event == HAKO_SERVICE_CLIENT_API_REQUEST_CANCEL_DONE) {
                    std::cout << "CLIENT cancel done" << std::endl;
                    phase = Phase::send_ok;
                } else if (service_event != HAKO_SERVICE_CLIENT_API_EVENT_NONE) {
                    std::cerr << "ERROR: cancel answered with event=" << service_event
                              << std::endl;
                    return 1;
                }
                break;
            case Phase::send_ok:
                if (!send(handle, 0, 9)) return 1;
                phase = Phase::wait_ok;
                break;
            case Phase::wait_ok:
                if (service_event == HAKO_SERVICE_CLIENT_API_RESPONSE_IN) {
                    char* packet = nullptr;
                    std::size_t packet_len = 0;
                    HakoCpp_AddTwoIntsResponsePacket response {};
                    if (hako_asset_service_client_get_response(
                            &handle, &packet, &packet_len, 0) != 0
                        || !response_converter.pdu2cpp(packet, response)) return 1;
                    std::cout << "CLIENT response result=" << int(response.header.result_code)
                              << " sum=" << response.body.sum << std::endl;
                    if (response.header.result_code != HAKO_SERVICE_API_RESULT_CODE_OK
                        || response.body.sum != 9) return 1;
                    std::cout << "CLIENT done" << std::endl;
                    phase = Phase::done;
                } else if (service_event != HAKO_SERVICE_CLIENT_API_EVENT_NONE) {
                    std::cerr << "ERROR: unexpected event=" << service_event << std::endl;
                    return 1;
                }
                break;
            case Phase::done:
                break;
            }
            const auto world_time = hakoniwa_asset_get_worldtime();
            if (local_time <= world_time) {
                local_time += kDeltaUsec;
                hakoniwa_asset_notify_simtime(kAsset, local_time);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
