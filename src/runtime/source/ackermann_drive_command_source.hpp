#pragma once

#include "runtime/source/command_source.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace hakoniwa::robot_runtime::runtime {

struct AckermannDriveCommand {
    double steering_angle_rad {0.0};
    double steering_angle_velocity_rad_s {0.0};
    double speed_m_s {0.0};
    double acceleration_m_s2 {0.0};
    double jerk_m_s3 {0.0};
};

class IAckermannDriveEventReader {
public:
    virtual ~IAckermannDriveEventReader() = default;
    [[nodiscard]] virtual std::optional<AckermannDriveCommand> take_latest() = 0;
    virtual void reset() noexcept = 0;
};

class AckermannDriveInput final : public IControllerInput {
public:
    AckermannDriveInput(
        ControllerInputMetadata metadata,
        AckermannDriveCommand command);
    [[nodiscard]] const ControllerInputMetadata& metadata() const noexcept override;
    [[nodiscard]] std::string_view type_name() const noexcept override;
    [[nodiscard]] const AckermannDriveCommand& command() const noexcept;

private:
    ControllerInputMetadata metadata_;
    AckermannDriveCommand command_;
};

class AckermannDriveCommandSource final : public ICommandSource {
public:
    AckermannDriveCommandSource(
        std::string source_id,
        std::uint64_t command_timeout_usec,
        std::shared_ptr<IAckermannDriveEventReader> reader);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] CommandSourcePollResult poll(
        const RuntimeStepContext& context) override;
    void reset() noexcept override;

private:
    std::string source_id_;
    std::uint64_t command_timeout_usec_ {0};
    std::shared_ptr<IAckermannDriveEventReader> reader_;
    std::uint64_t sequence_ {0};
};

} // namespace hakoniwa::robot_runtime::runtime
