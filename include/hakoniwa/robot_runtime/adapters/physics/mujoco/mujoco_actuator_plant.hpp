#pragma once

#include "runtime/plant/actuator_plant.hpp"
#include "runtime/runtime_definition.hpp"
#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
#include "hakoniwa/robot_runtime/adapters/endpoint/person_step_rpc_event_reader.hpp"
#endif

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
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE
    void apply_directives(
        const runtime::PlantDirectiveList& directives) override;
#endif
    [[nodiscard]] runtime::RobotState reset(
        std::uint64_t simulation_time_usec = 0) override;

    /** Non-owning handles for presentation adapters such as the Viewer. */
    [[nodiscard]] mjModel* model() noexcept;
    [[nodiscard]] mjData* data() noexcept;

#if defined(HAKO_ENABLE_ASYNC_RPC_INPUT) && HAKO_ENABLE_ASYNC_RPC_INPUT
    /** Binds one `<person>/...` hakoniwa_person model to PersonStep. */
    void bind_person(std::string person_name);
    /** Keeps the latest body-frame command active until the next request. */
    [[nodiscard]] std::uint32_t apply_person_command(
        const std::string& person_name,
        const adapters::hakoniwa::PersonStepCommand& command);
    [[nodiscard]] adapters::hakoniwa::PersonStepState person_state(
        const std::string& person_name,
        std::uint32_t result_code = 0) const;
    /** Drains contact edges produced since the previous call. */
    [[nodiscard]] std::vector<runtime::PersonContactEvent>
        take_person_contact_events();
#endif

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hakoniwa::robot_runtime::adapters::mujoco
