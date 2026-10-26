#include "hakoniwa_asset_polling.h"
#include "hako_srv_msgs/pdu_cpptype_conv_PersonStepRequestPacket.hpp"
#define hako_convert_pdu2cpp_array_string_varray hako_convert_pdu2ros_array_string_varray
#define hako_convert_cpp2pdu_array_string_varray hako_convert_ros2pdu_array_string_varray
#include "hako_srv_msgs/pdu_cpptype_conv_PersonStepResponsePacket.hpp"
#undef hako_convert_pdu2cpp_array_string_varray
#undef hako_convert_cpp2pdu_array_string_varray

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

namespace {
constexpr hako_time_t kDeltaUsec = 1000;
constexpr int kStateRunning = 2;
}

int main(int argc, char** argv)
{
    std::cout << std::unitbuf;
    if (argc != 5) return 2;
    const std::string config = argv[1];
    const std::string asset = argv[2];
    const std::string person = argv[3];
    const std::string service = "PersonService/PersonStep/" + person;
    const bool walking = person == "A";
    if (hakoniwa_asset_init() != 0 || hakoniwa_simevent_init() != 0
        || hakoniwa_asset_register_polling(asset.c_str()) != 0
        || hako_asset_service_initialize(config.c_str()) != 0) return 1;
    std::cout << "CLIENT registered channel=" << person << std::endl;
    HakoServiceHandleType handle {-1, -1};
    bool valid = false;
    bool waiting = false;
    std::uint64_t step = 0;
    hako_time_t local_time = 0;
    const std::string client_name = asset + "Rpc";
    auto request_converter =
        hako::pdu::msgs::hako_srv_msgs::PersonStepRequestPacket {};
    auto response_converter =
        hako::pdu::msgs::hako_srv_msgs::PersonStepResponsePacket {};
    for (;;) {
        const int event = hakoniwa_asset_get_event(asset.c_str());
        if (event == 1) {
            if (!hakoniwa_asset_is_pdu_created()
                || hako_asset_service_client_create(
                    asset.c_str(), service.c_str(), client_name.c_str(),
                    &handle) != 0) return 1;
            valid = true;
            hakoniwa_asset_notify_write_pdu_done(asset.c_str());
            if (hakoniwa_asset_start_feedback_ok(asset.c_str()) != 0) return 1;
        } else if (event == 2) {
            if (hakoniwa_asset_stop_feedback_ok(asset.c_str()) != 0) return 1;
            return 0;
        } else if (event == 3) {
            valid = false;
            waiting = false;
            step = 0;
            local_time = 0;
            if (hakoniwa_asset_reset_feedback_ok(asset.c_str()) != 0) return 1;
        } else if (event == 4) return 1;

        if (valid && hakoniwa_simevent_get_state() == kStateRunning
            && hakoniwa_asset_is_pdu_created()) {
            if (!waiting && step < 3) {
                char* packet = nullptr;
                std::size_t packet_len = 0;
                HakoCpp_PersonStepRequestPacket request {};
                if (hako_asset_service_client_get_request_buffer(
                        &handle, &packet, &packet_len,
                        HAKO_SERVICE_CLIENT_API_OPCODE_REQUEST, -1) != 0
                    || !request_converter.pdu2cpp(packet, request)) return 1;
                request.body.step = step;
                request.body.cmd_vel.linear.x = walking ? 1.0 : 0.0;
                request.body.cmd_vel.angular.z = walking ? 0.0 : 0.5;
                request.body.animation = "auto";
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
                    HakoCpp_PersonStepResponsePacket response {};
                    if (hako_asset_service_client_get_response(
                            &handle, &packet, &packet_len, 0) != 0
                        || !response_converter.pdu2cpp(packet, response)) return 1;
                    const double expected = 0.001 * static_cast<double>(step + 1);
                    const bool pose_ok = walking
                        ? std::fabs(response.body.pose.translation.x - expected) < 1.0e-9
                        : std::fabs(2.0 * std::atan2(
                              response.body.pose.rotation.z,
                              response.body.pose.rotation.w)
                              - 0.5 * expected) < 1.0e-9;
                    if (response.body.step != step
                        || response.body.result_code != 0
                        || response.body.joints.name.size() != 6
                        || !pose_ok) return 1;
                    std::cout << "RESPONSE channel=" << person
                              << " k=" << step
                              << " x=" << response.body.pose.translation.x
                              << std::endl;
                    waiting = false;
                    if (++step == 3) {
                        std::cout << "DONE channel=" << person
                                  << " count=3" << std::endl;
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
