#include "runner/hakoniwa/hakoniwa_asset_driver.hpp"

#include "hako_asset_service.h"
#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AckermannStepResponsePacket.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

namespace {

constexpr char kAsset[] = "AckermannStartupClient";
constexpr hako_time_t kDeltaUsec = 1000;
constexpr std::uint64_t kSteps = 3;
using Clock = std::chrono::steady_clock;

struct Vehicle {
    const char* service;
    float speed;
    HakoServiceHandleType handle {-1, -1};
    bool received {false};
};

bool send(Vehicle& vehicle, std::uint64_t step)
{
    char* packet = nullptr;
    std::size_t size = 0;
    HakoCpp_AckermannStepRequestPacket request {};
    auto converter = hako::pdu::msgs::hako_srv_msgs::AckermannStepRequestPacket {};
    if (hako_asset_service_client_get_request_buffer(
            &vehicle.handle, &packet, &size, HAKO_SERVICE_CLIENT_API_OPCODE_REQUEST, -1) != 0
        || !converter.pdu2cpp(packet, request)) {
        return false;
    }
    request.body.step = step;
    request.body.drive.speed = vehicle.speed;
    vehicle.received = false;
    return converter.cpp2pdu(request, packet, static_cast<int>(size)) >= 0
        && hako_asset_service_client_call_request(&vehicle.handle, packet, size, 30000) == 0;
}

bool poll(Vehicle& vehicle, std::uint64_t step)
{
    if (vehicle.received) {
        return true;
    }
    const auto event = hako_asset_service_client_poll(&vehicle.handle);
    if (event == HAKO_SERVICE_CLIENT_API_EVENT_NONE) {
        return true;
    }
    if (event != HAKO_SERVICE_CLIENT_API_RESPONSE_IN) {
        return false;
    }

    char* packet = nullptr;
    std::size_t size = 0;
    HakoCpp_AckermannStepResponsePacket response {};
    vehicle.received =
        hako_asset_service_client_get_response(&vehicle.handle, &packet, &size, 0) == 0
        && hako::pdu::msgs::hako_srv_msgs::AckermannStepResponsePacket {}.pdu2cpp(packet, response)
        && response.header.service_name == vehicle.service
        && response.header.result_code == HAKO_SERVICE_API_RESULT_CODE_OK
        && response.body.result_code == 0 && response.body.step == step
        && response.body.sim_time_usec == (step + 1) * kDeltaUsec;
    return vehicle.received;
}

class Callbacks final : public hakoniwa::robot_runtime::runner::detail::IHakoniwaAssetDriverCallbacks {
public:
    int on_initialize() noexcept override
    {
        for (auto& vehicle : vehicles_) {
            if (hako_asset_service_client_create(kAsset, vehicle.service, kAsset, &vehicle.handle) != 0) {
                std::cerr << "failed to create client for " << vehicle.service << std::endl;
                return 1;
            }
        }
        return 0;
    }

    bool ready_to_step() noexcept override { return step_ < kSteps; }

    int on_step() noexcept override
    {
        for (auto& vehicle : vehicles_) {
            if (!send(vehicle, step_)) {
                std::cerr << "failed to send " << vehicle.service << std::endl;
                return 1;
            }
        }
        std::cout << "STARTUP CLIENT sent batch k=" << step_ << std::endl;
        const auto deadline = Clock::now() + std::chrono::seconds(3);

        // The AssetDriver advances this callback asset only after on_step
        // returns. In particular, it makes no time notification at k=0
        // while this client waits for the replies, just like scenario-pro.
        do {
            for (auto& vehicle : vehicles_) {
                if (!poll(vehicle, step_)) {
                    std::cerr << "unexpected reply for " << vehicle.service << std::endl;
                    return 1;
                }
            }
            if (std::ranges::all_of(vehicles_, &Vehicle::received)) {
                std::cout << "STARTUP CLIENT received both replies k=" << step_
                          << " world_time_usec=" << hako_asset_simulation_time() << std::endl;
                if (++step_ == kSteps) {
                    std::cout << "STARTUP CLIENT complete batches=" << step_ << std::endl;
                }
                return 0;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } while (Clock::now() < deadline);
        std::cerr << "STARTUP CLIENT timeout at k=" << step_
                  << " world_time_usec=" << hako_asset_simulation_time() << std::endl;
        return 1;
    }

    int on_reset() noexcept override { return 0; }
    bool should_stop() const noexcept override { return false; }

private:
    std::array<Vehicle, 2> vehicles_ {{
        {"CarService/AckermannStep/Car-1", 1.0f},
        {"CarService/AckermannStep/Car-2", 2.0f},
    }};
    std::uint64_t step_ {0};
};

} // namespace

int main(int argc, char** argv)
{
    if (argc != 3) {
        return 2;
    }
    Callbacks callbacks;
    hakoniwa::robot_runtime::runner::detail::HakoniwaAssetDriver driver(
        {kAsset, argv[1], kDeltaUsec, HAKO_ASSET_MODEL_PLANT}, callbacks);
    if (driver.register_asset() != 0 || hako_asset_service_initialize(argv[2]) != 0) {
        return 1;
    }
    std::cout << "STARTUP CLIENT registered" << std::endl;
    return driver.start() == 0 ? 0 : 1;
}
