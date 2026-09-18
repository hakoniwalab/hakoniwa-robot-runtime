#pragma once

#include "runtime/publisher/state_publisher.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

namespace hakoniwa::robot_runtime::runtime {

struct ImpulseCollisionPolicy {
    double restitution_coefficient {0.3};
    double relative_normal_speed_threshold_mps {0.2};
    std::uint64_t cooldown_usec {100'000};
};

struct ImpulseCollisionOutput {
    std::string mirror_id;
    std::string local_body_id;
    bool collision {true};
    bool is_target_static {false};
    double restitution_coefficient {0.3};
    Vector3State self_contact_vector;
    Vector3State normal;
    Vector3State target_contact_vector;
    Vector3State target_velocity;
    Vector3State target_angular_velocity;
    EulerState target_euler;
    Vector3State target_inertia;
    double target_mass {0.0};
};

class IImpulseCollisionWriter {
public:
    virtual ~IImpulseCollisionWriter() = default;
    [[nodiscard]] virtual bool send(
        const ImpulseCollisionOutput& output) noexcept = 0;
    virtual void reset() noexcept = 0;
};

class ImpulseCollisionPublisher final : public IStatePublisher {
public:
    ImpulseCollisionPublisher(
        std::string component_id,
        std::string mirror_id,
        ImpulseCollisionPolicy policy,
        std::shared_ptr<IImpulseCollisionWriter> writer);

    [[nodiscard]] std::string_view id() const noexcept override;
    [[nodiscard]] ComponentStatus publish(
        const RobotState& state,
        const RuntimeStepContext& context) noexcept override;
    void reset() noexcept override;

private:
    struct PairState {
        bool contact_active {false};
        std::optional<std::uint64_t> last_emit_time_usec;
    };

    std::string component_id_;
    std::string mirror_id_;
    ImpulseCollisionPolicy policy_;
    std::shared_ptr<IImpulseCollisionWriter> writer_;
    std::unordered_map<std::string, PairState> pair_states_;
};

} // namespace hakoniwa::robot_runtime::runtime
