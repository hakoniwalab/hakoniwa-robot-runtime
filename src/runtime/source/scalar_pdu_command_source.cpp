#include "runtime/source/scalar_pdu_command_source.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

ScalarPduInput::ScalarPduInput(ControllerInputMetadata metadata, const double value)
    : metadata_(std::move(metadata)), value_(value)
{
}

const ControllerInputMetadata& ScalarPduInput::metadata() const noexcept
{
    return metadata_;
}

std::string_view ScalarPduInput::type_name() const noexcept
{
    return "std_msgs/Float64";
}

double ScalarPduInput::value() const noexcept
{
    return value_;
}

ScalarPduCommandSource::ScalarPduCommandSource(
    std::string source_id,
    const std::uint64_t command_timeout_usec,
    std::shared_ptr<IFloat64EventReader> reader)
    : source_id_(std::move(source_id))
    , command_timeout_usec_(command_timeout_usec)
    , reader_(std::move(reader))
{
    if (source_id_.empty()) {
        throw std::invalid_argument("scalar PDU source ID must not be empty");
    }
    if (command_timeout_usec_ == 0) {
        throw std::invalid_argument("scalar PDU timeout must be positive");
    }
    if (reader_ == nullptr) {
        throw std::invalid_argument("scalar PDU event reader must not be null");
    }
}

std::string_view ScalarPduCommandSource::id() const noexcept
{
    return source_id_;
}

CommandSourcePollResult ScalarPduCommandSource::poll(
    const RuntimeStepContext& context)
{
    auto value = reader_->take_latest();
    if (!value.has_value()) {
        return {nullptr, {source_id_, ComponentState::WaitingForInput, {}}};
    }
    if (context.simulation_time_usec
        > std::numeric_limits<std::uint64_t>::max() - command_timeout_usec_) {
        return {nullptr, {source_id_, ComponentState::Error,
            "scalar PDU command expiry overflows simulation time"}};
    }
    if (sequence_ == std::numeric_limits<std::uint64_t>::max()) {
        return {nullptr, {source_id_, ComponentState::Error,
            "scalar PDU command sequence exhausted"}};
    }
    ++sequence_;
    return {
        std::make_shared<ScalarPduInput>(
            ControllerInputMetadata {
                source_id_,
                sequence_,
                context.simulation_time_usec,
                context.simulation_time_usec + command_timeout_usec_,
            },
            *value),
        {source_id_, ComponentState::Ready, {}},
    };
}

void ScalarPduCommandSource::reset() noexcept
{
    sequence_ = 0;
    reader_->reset();
}

} // namespace hakoniwa::robot_runtime::runtime
