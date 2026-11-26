#include "hakoniwa_asset_polling.h"
#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepResponsePacket.hpp"

#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

namespace {

constexpr hako_time_t kDeltaUsec = 1000;
constexpr std::array kSpeeds {1.0f, 4.0f, 2.0f, -1.0f};
constexpr std::array kSecondSpeeds {3.0f, -2.0f, 1.0f, 5.0f};
constexpr std::uint64_t kSteps = kSpeeds.size() * 20;

bool send(HakoServiceHandleType& handle, std::uint64_t step, bool second_car)
{
    char* packet = nullptr;
    std::size_t size = 0;
    HakoCpp_AckermannStepRequestPacket request {};
    auto converter = hako::pdu::msgs::hako_srv_msgs::AckermannStepRequestPacket {};
    if (hako_asset_service_client_get_request_buffer(
            &handle, &packet, &size, HAKO_SERVICE_CLIENT_API_OPCODE_REQUEST, -1) != 0
        || !converter.pdu2cpp(packet, request)) {
        return false;
    }
    request.body.step = step;
    request.body.drive.speed = (second_car ? kSecondSpeeds : kSpeeds).at(step / 20);
    request.body.drive.steering_angle = second_car ? -0.2f : 0.1f;
    return converter.cpp2pdu(request, packet, static_cast<int>(size)) >= 0
        && hako_asset_service_client_call_request(&handle, packet, size, 30000) == 0;
}

bool record(HakoServiceHandleType& handle, std::uint64_t step, std::ofstream& trace)
{
    char* packet = nullptr;
    std::size_t size = 0;
    HakoCpp_AckermannStepResponsePacket response {};
    if (hako_asset_service_client_get_response(&handle, &packet, &size, 0) != 0
        || !hako::pdu::msgs::hako_srv_msgs::AckermannStepResponsePacket {}.pdu2cpp(packet, response)
        || response.header.result_code != HAKO_SERVICE_API_RESULT_CODE_OK
        || response.body.result_code != 0 || response.body.step != step
        || response.body.sim_time_usec != (step + 1) * kDeltaUsec) {
        std::cerr << "ERROR response at k=" << step << std::endl;
        return false;
    }
    const auto& body = response.body;
    const auto& p = body.pose.translation;
    const auto& q = body.pose.rotation;
    const auto& v = body.twist.linear;
    const auto& w = body.twist.angular;
    const std::array<double, 13> state {
        p.x, p.y, p.z, q.x, q.y, q.z, q.w,
        v.x, v.y, v.z, w.x, w.y, w.z,
    };
    trace.write(reinterpret_cast<const char*>(&body.sim_time_usec), sizeof(body.sim_time_usec));
    trace.write(reinterpret_cast<const char*>(state.data()), sizeof(state)).flush();
    return static_cast<bool>(trace);
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 3 && argc != 4) {
        return 2;
    }
    const std::string vehicle = argc == 4 ? argv[3] : "Car-1";
    if (vehicle != "Car-1" && vehicle != "Car-2") {
        return 2;
    }
    const auto asset = "AckermannUrbanClient-" + vehicle;
    const auto service = "CarService/AckermannStep/" + vehicle;
    std::ofstream trace(argv[2], std::ios::binary | std::ios::trunc);
    if (!trace || hakoniwa_asset_init() != 0 || hakoniwa_simevent_init() != 0
        || hakoniwa_asset_register_polling(asset.c_str()) != 0
        || hako_asset_service_initialize(argv[1]) != 0) {
        return 1;
    }
    std::cout << "CLIENT registered channel=" << vehicle << std::endl;
    HakoServiceHandleType handle {-1, -1};
    bool active = false;
    bool waiting = false;
    std::uint64_t step = 0;
    hako_time_t local_time = 0;
    constexpr int kStateRunning = 2;

    for (;;) {
        const int event = hakoniwa_asset_get_event(asset.c_str());
        if (event == 1) {
            if (!hakoniwa_asset_is_pdu_created()
                || hako_asset_service_client_create(
                    asset.c_str(), service.c_str(), asset.c_str(), &handle) != 0) {
                return 1;
            }
            active = true;
            hakoniwa_asset_notify_write_pdu_done(asset.c_str());
            if (hakoniwa_asset_start_feedback_ok(asset.c_str()) != 0) {
                return 1;
            }
        } else if (event == 2) {
            return hakoniwa_asset_stop_feedback_ok(asset.c_str()) == 0 && step == kSteps ? 0 : 1;
        } else if (event == 3 || event == 4) {
            return 1;
        }

        if (active && hakoniwa_simevent_get_state() == kStateRunning) {
            if (waiting) {
                const int result = hako_asset_service_client_poll(&handle);
                if (result == HAKO_SERVICE_CLIENT_API_RESPONSE_IN) {
                    if (!record(handle, step, trace)) {
                        return 1;
                    }
                    waiting = false;
                    if (++step == kSteps) {
                        std::cout << "DONE channel=" << vehicle << " count=" << step << std::endl;
                    }
                } else if (result != HAKO_SERVICE_CLIENT_API_EVENT_NONE) {
                    return 1;
                }
            }
            if (!waiting && step < kSteps) {
                if (!send(handle, step, vehicle == "Car-2")) {
                    return 1;
                }
                waiting = true;
            }
            if (local_time <= hakoniwa_asset_get_worldtime()) {
                local_time += kDeltaUsec;
                hakoniwa_asset_notify_simtime(asset.c_str(), local_time);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
