#include "runtime/source/mirror_body_state_source.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {

MirrorBodyStateInput::MirrorBodyStateInput(
    ControllerInputMetadata metadata,
    MirrorBodySample sample)
    : metadata_(std::move(metadata)), sample_(std::move(sample))
{
}

const ControllerInputMetadata& MirrorBodyStateInput::metadata() const noexcept
{
    return metadata_;
}

std::string_view MirrorBodyStateInput::type_name() const noexcept
{
    return "hakoniwa/MirrorBodyState";
}

const MirrorBodySample& MirrorBodyStateInput::sample() const noexcept
{
    return sample_;
}

MirrorBodyStateSource::MirrorBodyStateSource(
    std::string source_id,
    std::shared_ptr<IMirrorBodyStateReader> reader)
    : source_id_(std::move(source_id)), reader_(std::move(reader))
{
    if (source_id_.empty() || reader_ == nullptr) {
        throw std::invalid_argument("Mirror source requires an ID and reader");
    }
}

std::string_view MirrorBodyStateSource::id() const noexcept
{
    return source_id_;
}

CommandSourcePollResult MirrorBodyStateSource::poll(
    const RuntimeStepContext& context)
{
    auto sample = reader_->read_latest();
    if (!sample.has_value()) {
        return {nullptr, {source_id_, ComponentState::WaitingForInput, {}}};
    }
    if (sequence_ == std::numeric_limits<std::uint64_t>::max()) {
        return {nullptr, {source_id_, ComponentState::Error,
            "Mirror state sequence exhausted"}};
    }
    ++sequence_;
    return {
        std::make_shared<MirrorBodyStateInput>(
            ControllerInputMetadata {
                source_id_,
                sequence_,
                context.simulation_time_usec,
                context.simulation_time_usec >
                        std::numeric_limits<std::uint64_t>::max()
                            - context.delta_time_usec
                    ? std::optional<std::uint64_t> {
                        std::numeric_limits<std::uint64_t>::max()}
                    : std::optional<std::uint64_t> {
                        context.simulation_time_usec + context.delta_time_usec},
            },
            std::move(*sample)),
        {source_id_, ComponentState::Ready, {}},
    };
}

void MirrorBodyStateSource::reset() noexcept
{
    sequence_ = 0;
    reader_->reset();
}

} // namespace hakoniwa::robot_runtime::runtime
