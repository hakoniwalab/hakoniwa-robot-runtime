#include "runtime/source/joy_command_source.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

JoyInput::JoyInput(ControllerInputMetadata metadata, JoyState state)
    : metadata_(std::move(metadata)), state_(std::move(state))
{
}

const ControllerInputMetadata& JoyInput::metadata() const noexcept
{
    return metadata_;
}

std::string_view JoyInput::type_name() const noexcept
{
    return "sensor_msgs/Joy";
}

const JoyState& JoyInput::state() const noexcept
{
    return state_;
}

JoyCommandSource::JoyCommandSource(
    std::string source_id,
    const std::uint64_t input_timeout_usec,
    std::shared_ptr<IJoyEventReader> reader)
    : source_id_(std::move(source_id))
    , input_timeout_usec_(input_timeout_usec)
    , reader_(std::move(reader))
{
    if (source_id_.empty() || input_timeout_usec_ == 0 || reader_ == nullptr) {
        throw std::invalid_argument("Joy source requires an ID, timeout, and reader");
    }
}

std::string_view JoyCommandSource::id() const noexcept
{
    return source_id_;
}

CommandSourcePollResult JoyCommandSource::poll(const RuntimeStepContext& context)
{
    auto state = reader_->take_latest();
    if (!state.has_value()) {
        return {nullptr, {source_id_, ComponentState::WaitingForInput, {}}};
    }
    if (sequence_ == std::numeric_limits<std::uint64_t>::max()
        || context.simulation_time_usec
            > std::numeric_limits<std::uint64_t>::max() - input_timeout_usec_) {
        return {nullptr, {source_id_, ComponentState::Error, "Joy input sequence/time exhausted"}};
    }
    ++sequence_;
    return {
        std::make_shared<JoyInput>(
            ControllerInputMetadata {
                source_id_, sequence_, context.simulation_time_usec,
                context.simulation_time_usec + input_timeout_usec_},
            std::move(*state)),
        {source_id_, ComponentState::Ready, {}},
    };
}

void JoyCommandSource::reset() noexcept
{
    sequence_ = 0;
    reader_->reset();
}

} // namespace hakoniwa::robot_runtime::runtime
