#include "runtime/source/ackermann_drive_command_source.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

AckermannDriveInput::AckermannDriveInput(
    ControllerInputMetadata metadata,
    AckermannDriveCommand command)
    : metadata_(std::move(metadata)), command_(command)
{
}

const ControllerInputMetadata& AckermannDriveInput::metadata() const noexcept
{
    return metadata_;
}

std::string_view AckermannDriveInput::type_name() const noexcept
{
    return "ackermann_msgs/AckermannDrive";
}

const AckermannDriveCommand& AckermannDriveInput::command() const noexcept
{
    return command_;
}

AckermannDriveCommandSource::AckermannDriveCommandSource(
    std::string source_id,
    const std::uint64_t command_timeout_usec,
    std::shared_ptr<IAckermannDriveEventReader> reader)
    : source_id_(std::move(source_id))
    , command_timeout_usec_(command_timeout_usec)
    , reader_(std::move(reader))
{
    if (source_id_.empty() || command_timeout_usec_ == 0 || reader_ == nullptr) {
        throw std::invalid_argument(
            "Ackermann source requires an ID, timeout, and reader");
    }
}

std::string_view AckermannDriveCommandSource::id() const noexcept
{
    return source_id_;
}

CommandSourcePollResult AckermannDriveCommandSource::poll(
    const RuntimeStepContext& context)
{
    auto command = reader_->take_latest();
    if (!command.has_value()) {
        return {nullptr, {source_id_, ComponentState::WaitingForInput, {}}};
    }
    if (context.simulation_time_usec
        > std::numeric_limits<std::uint64_t>::max() - command_timeout_usec_) {
        return {nullptr, {source_id_, ComponentState::Error,
            "Ackermann command expiry overflows simulation time"}};
    }
    if (sequence_ == std::numeric_limits<std::uint64_t>::max()) {
        return {nullptr, {source_id_, ComponentState::Error,
            "Ackermann command sequence exhausted"}};
    }
    ++sequence_;
    return {
        std::make_shared<AckermannDriveInput>(
            ControllerInputMetadata {
                source_id_,
                sequence_,
                context.simulation_time_usec,
                context.simulation_time_usec + command_timeout_usec_,
            },
            *command),
        {source_id_, ComponentState::Ready, {}},
    };
}

void AckermannDriveCommandSource::reset() noexcept
{
    sequence_ = 0;
    reader_->reset();
}

} // namespace hakoniwa::robot_runtime::runtime
