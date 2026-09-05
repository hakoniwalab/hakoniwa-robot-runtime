#pragma once

#include "runtime/source/command_source.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

struct JoyState {
    std::vector<float> axes;
    std::vector<std::int32_t> buttons;
};

class IJoyEventReader {
public:
    virtual ~IJoyEventReader() = default;
    [[nodiscard]] virtual std::optional<JoyState> take_latest() = 0;
    virtual void reset() noexcept = 0;
};

class JoyInput final : public IControllerInput {
public:
    JoyInput(ControllerInputMetadata metadata, JoyState state);
    [[nodiscard]] const ControllerInputMetadata& metadata() const noexcept override;
    [[nodiscard]] std::string_view type_name() const noexcept override;
    [[nodiscard]] const JoyState& state() const noexcept;

private:
    ControllerInputMetadata metadata_;
    JoyState state_;
};

class JoyCommandSource final : public ICommandSource {
public:
    JoyCommandSource(
        std::string source_id,
        std::uint64_t input_timeout_usec,
        std::shared_ptr<IJoyEventReader> reader);
    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] CommandSourcePollResult poll(
        const RuntimeStepContext& context) override;
    void reset() noexcept override;

private:
    std::string source_id_;
    std::uint64_t input_timeout_usec_ {0};
    std::shared_ptr<IJoyEventReader> reader_;
    std::uint64_t sequence_ {0};
};

} // namespace hakoniwa::robot_runtime::runtime
