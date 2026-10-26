#include "hakoniwa/robot_runtime/adapters/endpoint/ackermann_drive_rpc_event_reader.hpp"
#include "hakoniwa/robot_runtime/factory/manifest_factory.hpp"
#include "hakoniwa/robot_runtime/runner/async_rpc_input.hpp"

#include <algorithm>
#include <cassert>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace hakoniwa::robot_runtime;

namespace {

class FakeChannel final : public runner::IAsyncRpcChannel {
public:
    explicit FakeChannel(std::string channel) : channel_(std::move(channel)) {}
    const std::string& name() const noexcept override { return channel_; }
    runner::AsyncRpcPollResult poll(
        runner::AsyncRpcRequest& request, std::string&) noexcept override
    {
        if (requests.empty()) return runner::AsyncRpcPollResult::none;
        request = std::move(requests.front());
        requests.pop_front();
        return runner::AsyncRpcPollResult::request;
    }
    bool reply(const runner::AsyncRpcResponse& response, std::string&) noexcept override
    {
        responses.push_back(response);
        return true;
    }
    void send(std::uint64_t step, const runtime::AckermannDriveCommand& command)
    {
        requests.push_back({channel_, step,
            adapters::hakoniwa::encode_ackermann_command(command)});
    }

    std::string channel_;
    std::deque<runner::AsyncRpcRequest> requests;
    std::vector<runner::AsyncRpcResponse> responses;
};

runtime::AckermannDriveCommand command(double speed, double steering)
{
    return {
        .steering_angle_rad = steering,
        .steering_angle_velocity_rad_s = 0.2,
        .speed_m_s = speed,
        .acceleration_m_s2 = 0.3,
        .jerk_m_s3 = 0.4,
    };
}

class FakePlant final : public runtime::IActuatorPlant {
public:
    FakePlant()
    {
        state_.sample_time_usec = 1000;
        state_.bodies = {
            {"A_body", {10.0, 1.0, 2.0}, {0.0, 0.0, 0.0, 1.0},
                {5.0, 0.0, 0.0}, {0.0, 0.0, 0.1}, 1000},
            {"B_body", {20.0, 3.0, 4.0}, {0.0, 0.0, 1.0, 0.0},
                {0.0, 1.0, 0.0}, {0.0, 0.0, 0.2}, 1000},
        };
    }
    std::uint64_t delta_time_usec() const noexcept override { return 1000; }
    runtime::RobotState read_state() const override { return state_; }
    runtime::RobotState step(
        const std::vector<runtime::ActuatorCommand>&) override { return state_; }
    runtime::RobotState reset(std::uint64_t = 0) override { return state_; }
private:
    runtime::RobotState state_;
};

} // namespace

int main()
{
    assert(adapters::hakoniwa::ackermann_step_service_name("Car-1")
        == "CarService/AckermannStep/Car-1");
    assert(adapters::hakoniwa::make_robot_rpc_service_name(
        "PersonService", "PersonStep", "Person-1")
        == "PersonService/PersonStep/Person-1");

    auto channel_a = std::make_shared<FakeChannel>("A");
    auto channel_b = std::make_shared<FakeChannel>("B");
    auto reader_a = std::make_shared<
        adapters::hakoniwa::AckermannDriveRpcEventReader>();
    auto reader_b = std::make_shared<
        adapters::hakoniwa::AckermannDriveRpcEventReader>();
    const std::map<std::string,
        std::shared_ptr<adapters::hakoniwa::AckermannDriveRpcEventReader>> readers {
        {"A", reader_a}, {"B", reader_b},
    };
    auto plant = std::make_shared<FakePlant>();
    runner::AsyncRpcInput input(
        {channel_a, channel_b}, {},
        [readers](std::span<const runner::AsyncRpcRequest> requests, std::string& error) {
            for (const auto& request : requests) {
                runtime::AckermannDriveCommand value;
                if (!adapters::hakoniwa::decode_ackermann_command(
                        request.payload, value)) {
                    error = "bad command";
                    return false;
                }
                readers.at(request.channel)->apply(value);
            }
            return true;
        },
        [plant](const runner::AsyncRpcRequest& request,
                 runner::AsyncRpcResponse& response, std::string&) {
            const auto state = plant->read_state();
            const auto body = std::find_if(
                state.bodies.begin(), state.bodies.end(),
                [&](const auto& candidate) {
                    return candidate.body_id == request.channel + "_body";
                });
            assert(body != state.bodies.end());
            response.step = request.step;
            response.payload = adapters::hakoniwa::encode_ackermann_state(
                {state.sample_time_usec, body->position, body->orientation,
                    body->linear_velocity, body->angular_velocity});
            return true;
        });

    channel_a->send(0, command(5.0, 0.1));
    channel_b->send(0, command(0.0, -0.1));
    assert(input.lockstep_ready());
    assert(input.apply_ready_requests());
    const auto a0 = reader_a->take_latest();
    const auto b0 = reader_b->take_latest();
    assert(a0 && a0->speed_m_s == 5.0 && a0->steering_angle_rad == 0.1);
    assert(b0 && b0->speed_m_s == 0.0 && b0->steering_angle_rad == -0.1);
    assert(input.complete_step());

    adapters::hakoniwa::AckermannStepState response_a;
    adapters::hakoniwa::AckermannStepState response_b;
    assert(adapters::hakoniwa::decode_ackermann_state(
        channel_a->responses.at(0).payload, response_a));
    assert(adapters::hakoniwa::decode_ackermann_state(
        channel_b->responses.at(0).payload, response_b));
    assert(channel_a->responses.at(0).step == 0);
    assert(response_a.simulation_time_usec == 1000);
    assert(response_a.position.x == 10.0);
    assert(response_a.linear_velocity.x == 5.0);
    assert(response_b.simulation_time_usec == 1000);
    assert(response_b.position.x == 20.0);
    assert(response_b.linear_velocity.x == 0.0);

    channel_a->send(1, command(7.0, 0.2));
    channel_b->send(1, command(2.0, 0.0));
    assert(input.lockstep_ready());
    assert(input.apply_ready_requests());
    const auto a1 = reader_a->take_latest();
    const auto b1 = reader_b->take_latest();
    assert(a1 && a1->speed_m_s == 7.0 && a1->steering_angle_rad == 0.2);
    assert(b1 && b1->speed_m_s == 2.0 && b1->steering_angle_rad == 0.0);
    assert(input.complete_step());

    // No opt-in selects the existing PDU reader path in ManifestFactory.
    const factory::ManifestFactoryConfig defaults;
    assert(defaults.ackermann_rpc_inputs.empty());
    assert(defaults.async_rpc_service_definition_path.empty());
    return 0;
}
