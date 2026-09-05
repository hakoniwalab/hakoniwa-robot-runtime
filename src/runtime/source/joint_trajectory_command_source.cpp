#include "runtime/source/joint_trajectory_command_source.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

JointTrajectoryInput::JointTrajectoryInput(
    ControllerInputMetadata metadata,
    JointTrajectory trajectory)
    : metadata_(std::move(metadata)), trajectory_(std::move(trajectory))
{
}

const ControllerInputMetadata& JointTrajectoryInput::metadata() const noexcept
{
    return metadata_;
}

std::string_view JointTrajectoryInput::type_name() const noexcept
{
    return "trajectory_msgs/JointTrajectory";
}

const JointTrajectory& JointTrajectoryInput::trajectory() const noexcept
{
    return trajectory_;
}

JointTrajectoryCommandSource::JointTrajectoryCommandSource(
    std::string source_id,
    std::shared_ptr<IJointTrajectoryEventReader> reader)
    : source_id_(std::move(source_id)), reader_(std::move(reader))
{
    if (source_id_.empty() || reader_ == nullptr) {
        throw std::invalid_argument("trajectory source requires an ID and reader");
    }
}

std::string_view JointTrajectoryCommandSource::id() const noexcept
{
    return source_id_;
}

CommandSourcePollResult JointTrajectoryCommandSource::poll(
    const RuntimeStepContext& context)
{
    auto trajectory = reader_->take_latest();
    if (!trajectory.has_value()) {
        return {nullptr, {source_id_, ComponentState::WaitingForInput, {}}};
    }
    if (sequence_ == std::numeric_limits<std::uint64_t>::max()) {
        return {nullptr, {source_id_, ComponentState::Error,
            "trajectory input sequence exhausted"}};
    }
    ++sequence_;
    return {
        std::make_shared<JointTrajectoryInput>(
            ControllerInputMetadata {
                source_id_, sequence_, context.simulation_time_usec, std::nullopt},
            std::move(*trajectory)),
        {source_id_, ComponentState::Ready, {}},
    };
}

void JointTrajectoryCommandSource::reset() noexcept
{
    sequence_ = 0;
    reader_->reset();
}

} // namespace hakoniwa::robot_runtime::runtime
