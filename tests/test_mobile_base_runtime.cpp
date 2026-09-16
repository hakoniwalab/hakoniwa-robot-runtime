#include "runtime/runtime_factory.hpp"
#include "runtime/controller/ackermann_controller.hpp"
#include "runtime/publisher/multi_dof_joint_state_publisher.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace hakoniwa::robot_runtime::runtime;

namespace {

constexpr double epsilon = 1.0e-9;

class EmptyScalarReader final : public IFloat64EventReader {
public:
    std::optional<double> take_latest() override { return std::nullopt; }
    void reset() noexcept override { }
};

class AckermannReader final : public IAckermannDriveEventReader {
public:
    explicit AckermannReader(AckermannDriveCommand command)
        : command_(command)
    {
    }

    std::optional<AckermannDriveCommand> take_latest() override
    {
        auto result = command_;
        command_.reset();
        return result;
    }

    void reset() noexcept override { }

private:
    std::optional<AckermannDriveCommand> command_;
};

class JointWriter final : public IJointStateWriter {
public:
    bool send(const JointStateOutput& output) noexcept override
    {
        outputs.push_back(output);
        return true;
    }
    void reset() noexcept override { outputs.clear(); }

    std::vector<JointStateOutput> outputs;
};

class MultiDofWriter final : public IMultiDofJointStateWriter {
public:
    bool send(const MultiDofJointStateOutput& output) noexcept override
    {
        outputs.push_back(output);
        return true;
    }
    void reset() noexcept override { outputs.clear(); }

    std::vector<MultiDofJointStateOutput> outputs;
};

class CountingPlant final : public IActuatorPlant {
public:
    explicit CountingPlant(std::vector<std::string> actuator_ids)
    {
        for (std::size_t index = 0; index < actuator_ids.size(); ++index) {
            state_.actuators.push_back({
                std::move(actuator_ids[index]),
                static_cast<double>(index),
                static_cast<double>(index) + 0.25,
                static_cast<double>(index) + 0.5,
                0,
            });
        }
        state_.bodies = {
            {"car_1_body", {1.0, 2.0, 3.0}, {0.0, 0.0, 0.0, 1.0},
                {4.0, 5.0, 6.0}, {0.1, 0.2, 0.3}, 0},
            {"car_2_body", {7.0, 8.0, 9.0}, {0.0, 0.0, 1.0, 0.0},
                {10.0, 11.0, 12.0}, {0.4, 0.5, 0.6}, 0},
        };
    }

    std::uint64_t delta_time_usec() const noexcept override { return 2'000; }
    RobotState read_state() const override { return state_; }

    RobotState step(const std::vector<ActuatorCommand>& commands) override
    {
        ++step_count;
        applied = commands;
        for (const auto& command : commands) {
            const auto state = std::find_if(
                state_.actuators.begin(), state_.actuators.end(),
                [&](const auto& candidate) {
                    return candidate.actuator_id == command.actuator_id;
                });
            assert(state != state_.actuators.end());
            if (command.type == ActuatorCommandType::Position) {
                state->position_rad = command.value;
            } else if (command.type == ActuatorCommandType::Velocity) {
                state->velocity_rad_per_sec = command.value;
            }
        }
        state_.sample_time_usec += delta_time_usec();
        for (auto& actuator : state_.actuators) {
            actuator.simulation_time_usec = state_.sample_time_usec;
        }
        for (auto& body : state_.bodies) {
            body.simulation_time_usec = state_.sample_time_usec;
        }
        return state_;
    }

    RobotState reset(const std::uint64_t time = 0) override
    {
        state_.sample_time_usec = time;
        return state_;
    }

    int step_count {0};
    std::vector<ActuatorCommand> applied;

private:
    RobotState state_;
};

RuntimeActuatorConfig actuator(
    std::string id,
    const ActuatorCommandType type)
{
    return {
        .component_id = std::move(id),
        .command_type = type,
        .limits = {-100.0, 100.0},
        .command_timeout_usec = 200'000,
    };
}

RuntimeAckermannControllerConfig controller(
    std::string id,
    std::string robot,
    const std::string& prefix)
{
    return {
        .component_id = std::move(id),
        .pdu_robot = std::move(robot),
        .pdu_name = "ackermann_cmd",
        .input_timeout_usec = 200'000,
        .update_rate_hz = 50.0,
        .geometry = {1.55, 1.04, 0.25, 0.70, 20.0},
        .actuators = {
            prefix + "steer_left",
            prefix + "steer_right",
            prefix + "drive_left",
            prefix + "drive_right",
        },
    };
}

const ActuatorCommand& command(
    const std::vector<ActuatorCommand>& commands,
    const std::string& id)
{
    const auto result = std::find_if(
        commands.begin(), commands.end(),
        [&](const auto& candidate) { return candidate.actuator_id == id; });
    assert(result != commands.end());
    return *result;
}

} // namespace

int main()
{
    RuntimeDefinition definition;
    for (const std::string prefix : {"car1_", "car2_"}) {
        definition.actuators.push_back(
            actuator(prefix + "steer_left", ActuatorCommandType::Position));
        definition.actuators.push_back(
            actuator(prefix + "steer_right", ActuatorCommandType::Position));
        definition.actuators.push_back(
            actuator(prefix + "drive_left", ActuatorCommandType::Velocity));
        definition.actuators.push_back(
            actuator(prefix + "drive_right", ActuatorCommandType::Velocity));
    }
    definition.ackermann_controllers = {
        controller("car1_ackermann", "Car-1", "car1_"),
        controller("car2_ackermann", "Car-2", "car2_"),
    };
    definition.state_outputs.push_back({
        .component_id = "joint_states",
        .pdu_robot = "UrbanFleet",
        .pdu_name = "joint_states",
        .update_rate_hz = 50.0,
        .joints = {
            {"car1_steer_left_joint", "car1_steer_left"},
            {"car1_steer_right_joint", "car1_steer_right"},
            {"car1_drive_left_joint", "car1_drive_left"},
            {"car1_drive_right_joint", "car1_drive_right"},
            {"car2_steer_left_joint", "car2_steer_left"},
            {"car2_steer_right_joint", "car2_steer_right"},
            {"car2_drive_left_joint", "car2_drive_left"},
            {"car2_drive_right_joint", "car2_drive_right"},
        },
    });
    definition.multi_dof_state_outputs.push_back({
        .component_id = "vehicle_states",
        .pdu_robot = "UrbanFleet",
        .pdu_name = "vehicle_states",
        .frame_id = "city_map",
        .update_rate_hz = 50.0,
        .bodies = {
            {"Car-1", "car_1_body", "car_1_freejoint"},
            {"Car-2", "car_2_body", "car_2_freejoint"},
        },
    });

    std::vector<std::string> actuator_ids;
    for (const auto& config : definition.actuators) {
        actuator_ids.push_back(config.component_id);
    }
    auto plant = std::make_shared<CountingPlant>(std::move(actuator_ids));
    auto joint_writer = std::make_shared<JointWriter>();
    auto multi_dof_writer = std::make_shared<MultiDofWriter>();
    std::vector<std::string> reader_namespaces;

    auto runtime = build_runtime(
        definition,
        plant,
        [](const RuntimeActuatorConfig&) {
            return std::make_shared<EmptyScalarReader>();
        },
        [&](const RuntimeStateOutputConfig& config) {
            assert(config.pdu_robot == "UrbanFleet");
            return joint_writer;
        },
        {},
        {},
        [&](const RuntimeAckermannControllerConfig& config) {
            reader_namespaces.push_back(config.pdu_robot);
            const double direction = config.pdu_robot == "Car-1" ? 1.0 : -1.0;
            return std::make_shared<AckermannReader>(
                AckermannDriveCommand {
                    .steering_angle_rad = direction * 0.35,
                    .speed_m_s = direction * 2.0,
                });
        },
        [&](const RuntimeMultiDofStateOutputConfig& config) {
            assert(config.pdu_robot == "UrbanFleet");
            return multi_dof_writer;
        });

    assert((reader_namespaces == std::vector<std::string> {"Car-1", "Car-2"}));
    const auto report = runtime->step();
    assert(plant->step_count == 1);
    assert(report.arbitration.selected_control_id == "ackermann-pdu-direct");
    assert(report.arbitration.conflicting_actuator_ids.empty());
    assert(report.arbitration.selected_commands.size() == 8);
    assert(plant->applied.size() == 8);

    const auto& car1_left = command(plant->applied, "car1_steer_left");
    const auto& car1_right = command(plant->applied, "car1_steer_right");
    const auto& car1_drive_left = command(plant->applied, "car1_drive_left");
    const auto& car1_drive_right = command(plant->applied, "car1_drive_right");
    assert(car1_left.value > car1_right.value);
    assert(car1_drive_left.value < car1_drive_right.value);
    assert(std::abs(car1_drive_left.value - 8.0) > epsilon);
    assert(std::abs(car1_drive_right.value - 8.0) > epsilon);

    const auto& car2_left = command(plant->applied, "car2_steer_left");
    const auto& car2_right = command(plant->applied, "car2_steer_right");
    assert(car2_left.value < 0.0 && car2_right.value < 0.0);
    assert(command(plant->applied, "car2_drive_left").value < 0.0);
    assert(command(plant->applied, "car2_drive_right").value < 0.0);

    assert(joint_writer->outputs.size() == 1);
    const auto& joints = joint_writer->outputs.front();
    assert(joints.names.size() == 8);
    assert(joints.names.size() == joints.position.size());
    assert(joints.names.size() == joints.velocity.size());
    assert(joints.names.size() == joints.effort.size());

    assert(multi_dof_writer->outputs.size() == 1);
    const auto& bodies = multi_dof_writer->outputs.front();
    assert(bodies.frame_id == "city_map");
    assert(bodies.bodies.size() == 2);
    assert(bodies.bodies[0].name == "Car-1");
    assert(bodies.bodies[0].position.x == 1.0);
    assert(bodies.bodies[0].linear_velocity.z == 6.0);
    assert(bodies.bodies[1].name == "Car-2");
    assert(bodies.bodies[1].position.z == 9.0);
    assert(bodies.bodies[1].angular_velocity.y == 0.5);

    bool rejected_invalid_geometry = false;
    try {
        auto invalid = controller("invalid", "Car-3", "car3_");
        invalid.geometry.wheel_radius_m = 0.0;
        [[maybe_unused]] AckermannController candidate(std::move(invalid));
    } catch (const std::invalid_argument&) {
        rejected_invalid_geometry = true;
    }
    assert(rejected_invalid_geometry);

    bool rejected_duplicate_body = false;
    try {
        [[maybe_unused]] MultiDofJointStatePublisher candidate(
            "invalid-bodies", "city_map",
            {{"Car-1", "same"}, {"Car-2", "same"}},
            50.0, multi_dof_writer);
    } catch (const std::invalid_argument&) {
        rejected_duplicate_body = true;
    }
    assert(rejected_duplicate_body);
    return 0;
}
