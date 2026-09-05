#pragma once

#include "runtime/plant/actuator_plant.hpp"
#include "runtime/runtime_definition.hpp"

#include <memory>

struct mjData_;
struct mjModel_;
using mjData = mjData_;
using mjModel = mjModel_;

namespace hakoniwa::robot_runtime::adapters::mujoco {

/** MuJoCo-backed actuator Plant assembled from a resolved Runtime definition. */
class MujocoActuatorPlant final : public runtime::IActuatorPlant {
public:
    explicit MujocoActuatorPlant(const runtime::RuntimeDefinition& definition);
    ~MujocoActuatorPlant() override;

    MujocoActuatorPlant(const MujocoActuatorPlant&) = delete;
    MujocoActuatorPlant& operator=(const MujocoActuatorPlant&) = delete;
    MujocoActuatorPlant(MujocoActuatorPlant&&) noexcept;
    MujocoActuatorPlant& operator=(MujocoActuatorPlant&&) noexcept;

    [[nodiscard]] std::uint64_t delta_time_usec() const noexcept override;
    [[nodiscard]] runtime::RobotState read_state() const override;
    [[nodiscard]] runtime::RobotState step(
        const std::vector<runtime::ActuatorCommand>& commands) override;
    [[nodiscard]] runtime::RobotState reset(
        std::uint64_t simulation_time_usec = 0) override;

    /** Non-owning handles for presentation adapters such as the Viewer. */
    [[nodiscard]] mjModel* model() noexcept;
    [[nodiscard]] mjData* data() noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::mujoco
