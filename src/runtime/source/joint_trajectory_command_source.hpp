#pragma once

#include "runtime/source/command_source.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

struct JointTrajectoryPoint {
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> effort;
    std::uint64_t time_from_start_usec {0};
};

struct JointTrajectory {
    std::vector<std::string> joint_names;
    std::vector<JointTrajectoryPoint> points;
};

class IJointTrajectoryEventReader {
public:
    virtual ~IJointTrajectoryEventReader() = default;
    [[nodiscard]] virtual std::optional<JointTrajectory> take_latest() = 0;
    virtual void reset() noexcept = 0;
};

class JointTrajectoryInput final : public IControllerInput {
public:
    JointTrajectoryInput(
        ControllerInputMetadata metadata,
        JointTrajectory trajectory);
    [[nodiscard]] const ControllerInputMetadata& metadata() const noexcept override;
    [[nodiscard]] std::string_view type_name() const noexcept override;
    [[nodiscard]] const JointTrajectory& trajectory() const noexcept;

private:
    ControllerInputMetadata metadata_;
    JointTrajectory trajectory_;
};

class JointTrajectoryCommandSource final : public ICommandSource {
public:
    JointTrajectoryCommandSource(
        std::string source_id,
        std::shared_ptr<IJointTrajectoryEventReader> reader);
    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] CommandSourcePollResult poll(
        const RuntimeStepContext& context) override;
    void reset() noexcept override;

private:
    std::string source_id_;
    std::shared_ptr<IJointTrajectoryEventReader> reader_;
    std::uint64_t sequence_ {0};
};

} // namespace hakoniwa::robot_runtime::runtime
