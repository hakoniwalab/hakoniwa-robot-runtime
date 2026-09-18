#include "runtime/actuator_runtime.hpp"
#include "runtime/arbiter/priority_command_arbiter.hpp"
#include "runtime/controller/mirror_body_controller.hpp"
#include "runtime/publisher/impulse_collision_publisher.hpp"
#include "runtime/source/mirror_body_state_source.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

using namespace hakoniwa::robot_runtime::runtime;

namespace {

class QueueMirrorReader final : public IMirrorBodyStateReader {
public:
    std::vector<MirrorBodySample> samples;
    std::size_t next {0};

    std::optional<MirrorBodySample> read_latest() override
    {
        if (next >= samples.size()) {
            return std::nullopt;
        }
        return samples[next++];
    }

    void reset() noexcept override
    {
        next = 0;
    }
};

class RecordingPlant final : public IActuatorPlant {
public:
    std::uint64_t time_usec {0};
    std::vector<ActuatorCommand> actuator_commands;
    std::vector<MirrorBodyCommand> mirror_commands;

    std::uint64_t delta_time_usec() const noexcept override
    {
        return 1'000;
    }

    RobotState read_state() const override
    {
        RobotState state;
        state.sample_time_usec = time_usec;
        return state;
    }

    RobotState step(const std::vector<ActuatorCommand>& commands) override
    {
        actuator_commands = commands;
        time_usec += delta_time_usec();
        return read_state();
    }

    RobotState step(
        const std::vector<ActuatorCommand>& commands,
        const std::vector<MirrorBodyCommand>& mirrors) override
    {
        actuator_commands = commands;
        mirror_commands = mirrors;
        time_usec += delta_time_usec();
        return read_state();
    }

    RobotState reset(const std::uint64_t requested_time_usec) override
    {
        time_usec = requested_time_usec;
        actuator_commands.clear();
        mirror_commands.clear();
        return read_state();
    }
};

class RecordingImpulseWriter final : public IImpulseCollisionWriter {
public:
    std::vector<ImpulseCollisionOutput> outputs;

    bool send(const ImpulseCollisionOutput& output) noexcept override
    {
        outputs.push_back(output);
        return true;
    }

    void reset() noexcept override
    {
        outputs.clear();
    }
};

MirrorContactState contact(
    const char* local_body,
    const double distance,
    const double speed)
{
    MirrorContactState value;
    value.mirror_id = "Drone-1";
    value.local_body_id = local_body;
    value.distance_m = distance;
    value.relative_normal_speed_mps = speed;
    value.target_mass = 10.0;
    return value;
}

ComponentStatus publish_at(
    ImpulseCollisionPublisher& publisher,
    const std::uint64_t time_usec,
    std::vector<MirrorContactState> contacts)
{
    RobotState state;
    state.sample_time_usec = time_usec;
    state.mirror_contacts = std::move(contacts);
    RuntimeStepContext context;
    context.simulation_time_usec = time_usec;
    context.delta_time_usec = 1'000;
    return publisher.publish(state, context);
}

void test_mirror_bypasses_arbiter_and_derives_velocity()
{
    auto reader = std::make_shared<QueueMirrorReader>();
    reader->samples = {
        MirrorBodySample {
            {1.0, 2.0, 3.0},
            {0.0, 0.0, 0.0},
            Vector3State {4.0, 5.0, 6.0},
            Vector3State {0.1, 0.2, 0.3},
        },
        MirrorBodySample {
            {1.001, 2.002, 3.003},
            {0.001, 0.002, 0.003},
            std::nullopt,
            std::nullopt,
        },
        MirrorBodySample {
            {1.001, 2.002, 3.003},
            {0.001, 0.002, 0.003},
            std::nullopt,
            std::nullopt,
        },
        MirrorBodySample {
            {1.002, 2.004, 3.006},
            {0.002, 0.004, 0.006},
            std::nullopt,
            std::nullopt,
        },
        MirrorBodySample {
            {1.002, 2.004, 3.006},
            {0.0, 0.0, 1.5707963267948966},
            Vector3State {1.0, 0.0, 0.0},
            Vector3State {0.0, 0.0, 1.0},
        },
    };
    auto source = std::make_shared<MirrorBodyStateSource>(
        "mirror-pdu:Drone-1", reader);
    auto controller = std::make_shared<MirrorBodyController>(
        "mirror:Drone-1", std::string(source->id()), "Drone-1",
        MirrorVelocityFrame::Body);
    auto plant = std::make_shared<RecordingPlant>();
    auto arbiter = std::make_shared<PriorityCommandArbiter>(
        std::vector<ControllerPriority> {});
    ActuatorRuntime runtime(
        {source}, {}, arbiter, plant, {}, {controller});

    const auto first = runtime.step();
    assert(first.arbitration.selected_commands.empty());
    assert(first.mirror_commands.size() == 1);
    assert(plant->actuator_commands.empty());
    assert(plant->mirror_commands.size() == 1);
    assert(plant->mirror_commands[0].linear_velocity.x == 4.0);

    const auto second = runtime.step();
    assert(second.arbitration.selected_commands.empty());
    assert(second.mirror_commands.size() == 1);
    assert(std::abs(second.mirror_commands[0].linear_velocity.x - 1.0) < 1.0e-9);
    assert(std::abs(second.mirror_commands[0].linear_velocity.y - 2.0) < 1.0e-9);
    assert(std::abs(second.mirror_commands[0].angular_velocity.z - 3.0) < 1.0e-9);

    const auto retained = runtime.step();
    assert(retained.mirror_commands[0].linear_velocity.x == 0.0);
    assert(retained.mirror_commands[0].angular_velocity.z == 0.0);

    const auto delayed = runtime.step();
    assert(std::abs(delayed.mirror_commands[0].linear_velocity.x - 0.5) < 1.0e-9);
    assert(std::abs(delayed.mirror_commands[0].angular_velocity.z - 1.5) < 1.0e-9);

    const auto body_frame = runtime.step();
    assert(std::abs(body_frame.mirror_commands[0].linear_velocity.x) < 1.0e-9);
    assert(std::abs(body_frame.mirror_commands[0].linear_velocity.y - 1.0) < 1.0e-9);
    assert(body_frame.mirror_commands[0].angular_velocity.z == 1.0);
}

void test_impulse_rising_edge_threshold_cooldown_and_deepest_contact()
{
    auto writer = std::make_shared<RecordingImpulseWriter>();
    ImpulseCollisionPublisher publisher(
        "drone-impulse",
        "Drone-1",
        {0.3, 0.2, 100'000},
        writer);

    assert(publish_at(publisher, 1'000, {
        contact("Car-1", -0.01, 1.0),
        contact("Car-2", -0.02, 1.0),
    }).state == ComponentState::Ready);
    assert(writer->outputs.size() == 1);
    assert(writer->outputs.back().local_body_id == "Car-2");

    publish_at(publisher, 2'000, {
        contact("Car-1", -0.03, 1.0),
        contact("Car-2", -0.04, 1.0),
    });
    assert(writer->outputs.size() == 1);

    publish_at(publisher, 3'000, {});
    publish_at(publisher, 50'000, {contact("Car-2", -0.02, 1.0)});
    assert(writer->outputs.size() == 1);

    publish_at(publisher, 51'000, {});
    publish_at(publisher, 102'000, {contact("Car-2", -0.02, 1.0)});
    assert(writer->outputs.size() == 2);

    publish_at(publisher, 103'000, {});
    publish_at(publisher, 203'000, {contact("Car-1", -0.01, 0.1)});
    assert(writer->outputs.size() == 2);
    publish_at(publisher, 204'000, {contact("Car-1", -0.01, 2.0)});
    assert(writer->outputs.size() == 2);
    publish_at(publisher, 205'000, {});
    publish_at(publisher, 206'000, {contact("Car-1", -0.01, 2.0)});
    assert(writer->outputs.size() == 3);

    publisher.reset();
    assert(writer->outputs.empty());
    publish_at(publisher, 207'000, {contact("Car-1", -0.01, 2.0)});
    assert(writer->outputs.size() == 1);
}

} // namespace

int main()
{
    test_mirror_bypasses_arbiter_and_derives_velocity();
    test_impulse_rising_edge_threshold_cooldown_and_deepest_contact();
    return 0;
}
