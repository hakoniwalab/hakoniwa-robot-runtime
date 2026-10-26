#include "hakoniwa_asset_polling.h"
#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepResponsePacket.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr hako_time_t kDeltaUsec = 1000;
constexpr int kStateRunning = 2;
struct Item { std::uint64_t step; double speed; };
bool parse(const std::string& text, std::vector<Item>& result)
{
    std::stringstream stream(text);
    std::string item;
    while (std::getline(stream, item, ',')) {
        const auto separator = item.find(':');
        if (separator == std::string::npos) return false;
        result.push_back({std::stoull(item.substr(0, separator)),
                          std::stod(item.substr(separator + 1))});
    }
    return !result.empty();
}
}

int main(int argc, char** argv)
{
    std::cout << std::unitbuf;
    if (argc != 6) return 2;
    const std::string asset = argv[2];
    const std::string service = argv[3];
    const std::string channel = argv[4];
    std::vector<Item> plan;
    if (!parse(argv[5], plan)) return 2;
    if (hakoniwa_asset_init() != 0 || hakoniwa_simevent_init() != 0
        || hakoniwa_asset_register_polling(asset.c_str()) != 0
        || hako_asset_service_initialize(argv[1]) != 0) return 1;
    std::cout << "CLIENT registered channel=" << channel << std::endl;
    HakoServiceHandleType handle {-1, -1};
    bool valid = false;
    bool waiting = false;
    std::size_t next = 0;
    hako_time_t local_time = 0;
    const std::string client_name = asset + "Rpc";
    auto request_converter = hako::pdu::msgs::hako_srv_msgs::AckermannStepRequestPacket {};
    auto response_converter = hako::pdu::msgs::hako_srv_msgs::AckermannStepResponsePacket {};
    for (;;) {
        const int event = hakoniwa_asset_get_event(asset.c_str());
        if (event == 1) {
            if (!hakoniwa_asset_is_pdu_created()
                || hako_asset_service_client_create(
                    asset.c_str(), service.c_str(), client_name.c_str(), &handle) != 0) return 1;
            valid = true;
            hakoniwa_asset_notify_write_pdu_done(asset.c_str());
            if (hakoniwa_asset_start_feedback_ok(asset.c_str()) != 0) return 1;
        } else if (event == 2) {
            if (hakoniwa_asset_stop_feedback_ok(asset.c_str()) != 0) return 1;
            return 0;
        } else if (event == 3) {
            valid = false;
            waiting = false;
            local_time = 0;
            if (hakoniwa_asset_reset_feedback_ok(asset.c_str()) != 0) return 1;
        } else if (event == 4) return 1;

        if (valid && hakoniwa_simevent_get_state() == kStateRunning
            && hakoniwa_asset_is_pdu_created()) {
            if (!waiting && next < plan.size()) {
                char* packet = nullptr;
                std::size_t packet_len = 0;
                HakoCpp_AckermannStepRequestPacket request {};
                if (hako_asset_service_client_get_request_buffer(
                        &handle, &packet, &packet_len,
                        HAKO_SERVICE_CLIENT_API_OPCODE_REQUEST, -1) != 0
                    || !request_converter.pdu2cpp(packet, request)) return 1;
                request.body.step = plan[next].step;
                request.body.drive.speed = plan[next].speed;
                request.body.drive.steering_angle = channel == "A" ? 0.1 : -0.1;
                if (request_converter.cpp2pdu(
                        request, packet, static_cast<int>(packet_len)) < 0
                    || hako_asset_service_client_call_request(
                        &handle, packet, packet_len, 30000) != 0) return 1;
                waiting = true;
            } else if (waiting) {
                const int service_event = hako_asset_service_client_poll(&handle);
                if (service_event == HAKO_SERVICE_CLIENT_API_RESPONSE_IN) {
                    char* packet = nullptr;
                    std::size_t packet_len = 0;
                    HakoCpp_AckermannStepResponsePacket response {};
                    if (hako_asset_service_client_get_response(
                            &handle, &packet, &packet_len, 0) != 0
                        || !response_converter.pdu2cpp(packet, response)) return 1;
                    const auto& expected = plan[next];
                    if (response.body.step != expected.step
                        || response.body.result_code != 0
                        || std::abs(response.body.twist.linear.x - expected.speed) > 1e-12) {
                        std::cerr << "ERROR response channel=" << channel
                                  << " k=" << expected.step << std::endl;
                        return 1;
                    }
                    std::cout << "RESPONSE channel=" << channel
                              << " k=" << response.body.step
                              << " speed=" << response.body.twist.linear.x << std::endl;
                    ++next;
                    waiting = false;
                    if (next == plan.size()) {
                        std::cout << "DONE channel=" << channel << " count=" << next << std::endl;
                    }
                } else if (service_event == HAKO_SERVICE_CLIENT_API_REQUEST_TIMEOUT
                           || service_event < 0) return 1;
            }
            const auto world_time = hakoniwa_asset_get_worldtime();
            if (local_time <= world_time) {
                local_time += kDeltaUsec;
                hakoniwa_asset_notify_simtime(asset.c_str(), local_time);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
