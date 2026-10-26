#include "hakoniwa_asset_polling.h"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsResponsePacket.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {
constexpr hako_time_t kDeltaUsec = 1000;
constexpr int kStateRunning = 2;
constexpr int kEventStart = 1;
constexpr int kEventStop = 2;
constexpr int kEventReset = 3;
constexpr int kEventError = 4;
constexpr std::int64_t kStepPackingBase = 1'000'000;

struct Item {
    std::int64_t step;
    std::int64_t value;
};

bool parse_plan(const std::string& text, std::vector<Item>& result)
{
    std::stringstream entries(text);
    std::string entry;
    while (std::getline(entries, entry, ',')) {
        const auto separator = entry.find(':');
        if (separator == std::string::npos) return false;
        try {
            std::size_t used_step = 0;
            std::size_t used_value = 0;
            const auto step = std::stoll(entry.substr(0, separator), &used_step);
            const auto value = std::stoll(entry.substr(separator + 1), &used_value);
            if (used_step != separator || used_value != entry.size() - separator - 1
                || step < 0) {
                return false;
            }
            result.push_back({step, value});
        } catch (...) {
            return false;
        }
    }
    return !result.empty();
}
} // namespace

int main(int argc, char** argv)
{
    std::cout << std::unitbuf;
    if (argc != 6 && argc != 8) {
        std::cerr << "Usage: " << argv[0]
                  << " <service.json> <asset> <service> <channel> <k:value,...>"
                     " [<gate-step> <gate-file>]" << std::endl;
        return 2;
    }
    const std::string asset = argv[2];
    const std::string service = argv[3];
    const std::string channel = argv[4];
    std::vector<Item> plan;
    if (!parse_plan(argv[5], plan)) {
        std::cerr << "ERROR: invalid request plan" << std::endl;
        return 2;
    }
    std::int64_t gate_step = -1;
    std::filesystem::path gate_file;
    if (argc == 8) {
        gate_step = std::stoll(argv[6]);
        gate_file = argv[7];
    }

    if (hakoniwa_asset_init() != 0 || hakoniwa_simevent_init() != 0
        || hakoniwa_asset_register_polling(asset.c_str()) != 0
        || hako_asset_service_initialize(argv[1]) != 0) {
        std::cerr << "ERROR: client initialization failed channel=" << channel << std::endl;
        return 1;
    }
    std::cout << "CLIENT registered channel=" << channel << std::endl;

    HakoServiceHandleType handle {-1, -1};
    bool valid = false;
    bool waiting = false;
    std::size_t next = 0;
    hako_time_t local_time = 0;
    const std::string client_name = asset + "Rpc";
    auto request_converter = hako::pdu::msgs::hako_srv_msgs::AddTwoIntsRequestPacket {};
    auto response_converter = hako::pdu::msgs::hako_srv_msgs::AddTwoIntsResponsePacket {};

    for (;;) {
        const int event = hakoniwa_asset_get_event(asset.c_str());
        if (event == kEventStart) {
            if (!hakoniwa_asset_is_pdu_created()
                || hako_asset_service_client_create(
                       asset.c_str(), service.c_str(), client_name.c_str(), &handle) != 0) {
                std::cerr << "ERROR: client_create failed channel=" << channel << std::endl;
                return 1;
            }
            valid = true;
            waiting = false;
            local_time = 0;
            hakoniwa_asset_notify_write_pdu_done(asset.c_str());
            if (hakoniwa_asset_start_feedback_ok(asset.c_str()) != 0) return 1;
            std::cout << "CLIENT started channel=" << channel << std::endl;
        } else if (event == kEventStop) {
            if (hakoniwa_asset_stop_feedback_ok(asset.c_str()) != 0) return 1;
            std::cout << "CLIENT stopped channel=" << channel
                      << " responses=" << next << std::endl;
            return 0;
        } else if (event == kEventReset) {
            valid = false;
            waiting = false;
            local_time = 0;
            if (hakoniwa_asset_reset_feedback_ok(asset.c_str()) != 0) return 1;
        } else if (event == kEventError) {
            return 1;
        }

        if (valid && hakoniwa_simevent_get_state() == kStateRunning
            && hakoniwa_asset_is_pdu_created()) {
            if (!waiting && next < plan.size()) {
                const auto& item = plan[next];
                if (item.step != gate_step || std::filesystem::exists(gate_file)) {
                    char* packet = nullptr;
                    std::size_t packet_len = 0;
                    HakoCpp_AddTwoIntsRequestPacket request {};
                    if (hako_asset_service_client_get_request_buffer(
                            &handle, &packet, &packet_len,
                            HAKO_SERVICE_CLIENT_API_OPCODE_REQUEST, -1) != 0
                        || !request_converter.pdu2cpp(packet, request)) {
                        return 1;
                    }
                    request.body.a = item.step;
                    request.body.b = item.value;
                    if (request_converter.cpp2pdu(
                            request, packet, static_cast<int>(packet_len)) < 0
                        || hako_asset_service_client_call_request(
                               &handle, packet, packet_len, 30000) != 0) {
                        std::cerr << "ERROR: request send failed channel=" << channel
                                  << " k=" << item.step << std::endl;
                        return 1;
                    }
                    waiting = true;
                    std::cout << "SENT channel=" << channel << " k=" << item.step
                              << " value=" << item.value << std::endl;
                }
            } else if (waiting) {
                const int service_event = hako_asset_service_client_poll(&handle);
                if (service_event == HAKO_SERVICE_CLIENT_API_RESPONSE_IN) {
                    char* packet = nullptr;
                    std::size_t packet_len = 0;
                    HakoCpp_AddTwoIntsResponsePacket response {};
                    if (hako_asset_service_client_get_response(
                            &handle, &packet, &packet_len, 0) != 0
                        || !response_converter.pdu2cpp(packet, response)) {
                        return 1;
                    }
                    const auto& item = plan[next];
                    const std::int64_t expected = item.step * kStepPackingBase + item.value;
                    if (response.body.sum != expected) {
                        std::cerr << "ERROR: response mismatch channel=" << channel
                                  << " k=" << item.step << " expected=" << expected
                                  << " actual=" << response.body.sum << std::endl;
                        return 1;
                    }
                    std::cout << "RESPONSE channel=" << channel << " k=" << item.step
                              << " value=" << item.value << " packed=" << response.body.sum
                              << std::endl;
                    ++next;
                    waiting = false;
                    if (next == plan.size()) {
                        std::cout << "DONE channel=" << channel
                                  << " count=" << next << std::endl;
                    }
                } else if (service_event == HAKO_SERVICE_CLIENT_API_REQUEST_TIMEOUT
                           || service_event < 0) {
                    std::cerr << "ERROR: client poll failed channel=" << channel << std::endl;
                    return 1;
                }
            }

            const hako_time_t world_time = hakoniwa_asset_get_worldtime();
            if (local_time <= world_time) {
                local_time += kDeltaUsec;
                hakoniwa_asset_notify_simtime(asset.c_str(), local_time);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
