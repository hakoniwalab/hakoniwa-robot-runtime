#include "async_service_lockstep_fixture.hpp"

#include "hakoniwa_asset_polling.h"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsResponsePacket.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string_view>
#include <thread>

using namespace async_service_lockstep_fixture;

namespace {

bool send(
    HakoServiceHandleType& handle, unsigned channel, unsigned step, std::uint32_t& request_id)
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
    request.body.a = target(channel, step);
    request.body.b = 0;
    request_id = request.header.request_id;
    return converter.cpp2pdu(request, packet, static_cast<int>(size)) >= 0
        && hako_asset_service_client_call_request(&handle, packet, size, 30'000) == 0;
}

bool record_reply(
    HakoServiceHandleType& handle, const char* service, const char* client,
    unsigned channel, unsigned step, std::uint32_t request_id, std::ofstream& transcript)
{
    char* packet = nullptr;
    std::size_t size = 0;
    HakoCpp_AddTwoIntsResponsePacket response {};
    if (hako_asset_service_client_get_response(&handle, &packet, &size, 0) != 0
        || !hako::pdu::msgs::hako_srv_msgs::AddTwoIntsResponsePacket {}.pdu2cpp(packet, response)
        || response.header.request_id != request_id || response.header.service_name != service
        || response.header.client_name != client
        || response.header.status != HAKO_SERVICE_API_STATUS_DONE
        || response.header.result_code != HAKO_SERVICE_API_RESULT_CODE_OK) {
        return false;
    }
    const std::int64_t payload = response.body.sum;
    const auto position = payload / kTimeScale;
    const auto world_time = payload % kTimeScale;
    std::cout << "RESPONSE step=" << step << " position=" << position
              << " time_usec=" << world_time << std::endl;
    if (position != target(channel, step) + kPositionIncrement
        || world_time != static_cast<std::int64_t>(step * kDeltaUsec)) {
        return false;
    }
    transcript.write(reinterpret_cast<const char*>(&payload), sizeof(payload)).flush();
    return static_cast<bool>(transcript);
}

int fail(std::string_view message)
{
    std::cerr << "CLIENT ERROR: " << message << std::endl;
    return 1;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 6) {
        std::cerr << "Usage: " << argv[0]
                  << " <service.json> <A|B> <complete|stop|hold> <transcript.bin> <release-file>\n";
        return 2;
    }
    const std::string_view channel_name = argv[2];
    const std::string_view mode = argv[3];
    if ((channel_name != "A" && channel_name != "B")
        || (mode != "complete" && mode != "stop" && mode != "hold")) {
        return 2;
    }
    const unsigned channel = channel_name == "A" ? 0 : 1;
    const auto* asset = channel == 0 ? "AsyncServiceLockstepClientA" : "AsyncServiceLockstepClientB";
    const auto* service = channel == 0 ? "Service/AsyncA" : "Service/AsyncB";
    const unsigned last_step = mode == "stop" ? 5 : 20;
    std::ofstream transcript(argv[4], std::ios::binary | std::ios::trunc);
    if (!transcript || hakoniwa_asset_init() != 0 || hakoniwa_simevent_init() != 0
        || hakoniwa_asset_register_polling(asset) != 0
        || hako_asset_service_initialize(argv[1]) != 0) {
        return fail("registration failed");
    }
    std::cout << "CLIENT registered" << std::endl;

    HakoServiceHandleType handle {-1, -1};
    bool active = false;
    bool waiting = false;
    unsigned step = 1;
    std::uint32_t request_id = 0;
    hako_time_t local_time = 0;
    constexpr int kStateRunning = 2;
    for (;;) {
        const int event = hakoniwa_asset_get_event(asset);
        if (event == 1) {
            if (!hakoniwa_asset_is_pdu_created()
                || hako_asset_service_client_create(asset, service, asset, &handle) != 0) {
                return fail("service client creation failed");
            }
            active = true;
            hakoniwa_asset_notify_write_pdu_done(asset);
            if (hakoniwa_asset_start_feedback_ok(asset) != 0) {
                return fail("start feedback failed");
            }
        } else if (event == 2) {
            return hakoniwa_asset_stop_feedback_ok(asset) == 0 && step > last_step ? 0 : 1;
        } else if (event == 3 || event == 4) {
            return fail("unexpected Core event");
        }

        if (active && hakoniwa_simevent_get_state() == kStateRunning) {
            if (waiting) {
                const int service_event = hako_asset_service_client_poll(&handle);
                if (service_event == HAKO_SERVICE_CLIENT_API_RESPONSE_IN) {
                    if (!record_reply(handle, service, asset, channel, step, request_id, transcript)) {
                        return fail("unexpected response");
                    }
                    waiting = false;
                    if (++step > last_step) {
                        std::cout << "CLIENT complete steps=" << last_step << std::endl;
                    }
                } else if (service_event != HAKO_SERVICE_CLIENT_API_EVENT_NONE) {
                    return fail("unexpected service event");
                }
            }
            const bool held = mode == "hold" && step == 6 && !std::filesystem::exists(argv[5]);
            if (!waiting && step <= last_step && !held) {
                if (!send(handle, channel, step, request_id)) {
                    return fail("request failed");
                }
                waiting = true;
                std::cout << "SENT step=" << step << std::endl;
            }
            // Even when no request is being sent, this asset is ready for the next
            // Core tick. The server's gate is what prevents a tick without both requests.
            if (local_time <= hakoniwa_asset_get_worldtime()) {
                local_time += kDeltaUsec;
                hakoniwa_asset_notify_simtime(asset, local_time);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
