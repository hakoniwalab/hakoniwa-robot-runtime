#pragma once

#include "runtime/source/command_source.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace hakoniwa::robot_runtime::runtime {

/** Transport seam whose production implementation consumes Float64 events. */
class IFloat64EventReader {
public:
    virtual ~IFloat64EventReader() = default;
    [[nodiscard]] virtual std::optional<double> take_latest() = 0;
    virtual void reset() noexcept = 0;
};

class ScalarPduInput final : public IControllerInput {
public:
    ScalarPduInput(ControllerInputMetadata metadata, double value);
    [[nodiscard]] const ControllerInputMetadata& metadata() const noexcept override;
    [[nodiscard]] std::string_view type_name() const noexcept override;
    [[nodiscard]] double value() const noexcept;

private:
    ControllerInputMetadata metadata_;
    double value_ {0.0};
};

/** Stamps new Float64 receive events with simulation time and a local sequence. */
class ScalarPduCommandSource final : public ICommandSource {
public:
    ScalarPduCommandSource(
        std::string source_id,
        std::uint64_t command_timeout_usec,
        std::shared_ptr<IFloat64EventReader> reader);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] CommandSourcePollResult poll(
        const RuntimeStepContext& context) override;
    void reset() noexcept override;

private:
    std::string source_id_;
    std::uint64_t command_timeout_usec_ {0};
    std::shared_ptr<IFloat64EventReader> reader_;
    std::uint64_t sequence_ {0};
};

} // namespace hakoniwa::robot_runtime::runtime
