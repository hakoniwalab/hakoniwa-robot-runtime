#pragma once

#include "runtime/source/command_source.hpp"

#include <memory>
#include <optional>
#include <string>

namespace hakoniwa::robot_runtime::runtime {

struct MirrorBodySample {
    Vector3State position;
    EulerState orientation;
    std::optional<Vector3State> linear_velocity;
    std::optional<Vector3State> angular_velocity;
};

class IMirrorBodyStateReader {
public:
    virtual ~IMirrorBodyStateReader() = default;
    [[nodiscard]] virtual std::optional<MirrorBodySample> read_latest() = 0;
    virtual void reset() noexcept = 0;
};

class MirrorBodyStateInput final : public IControllerInput {
public:
    MirrorBodyStateInput(
        ControllerInputMetadata metadata,
        MirrorBodySample sample);

    [[nodiscard]] const ControllerInputMetadata& metadata() const noexcept override;
    [[nodiscard]] std::string_view type_name() const noexcept override;
    [[nodiscard]] const MirrorBodySample& sample() const noexcept;

private:
    ControllerInputMetadata metadata_;
    MirrorBodySample sample_;
};

class MirrorBodyStateSource final : public ICommandSource {
public:
    MirrorBodyStateSource(
        std::string source_id,
        std::shared_ptr<IMirrorBodyStateReader> reader);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] CommandSourcePollResult poll(
        const RuntimeStepContext& context) override;
    void reset() noexcept override;

private:
    std::string source_id_;
    std::shared_ptr<IMirrorBodyStateReader> reader_;
    std::uint64_t sequence_ {0};
};

} // namespace hakoniwa::robot_runtime::runtime
