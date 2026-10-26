#pragma once

#include "hakoniwa/robot_runtime/runner/runner.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::factory {

struct ManifestFactoryConfig {
    /** Empty means use manifest.name. */
    std::string asset_name;
    std::string endpoint_name;
    std::uint64_t realtime_sync_cycle_msec {0};
    /** True for standalone assets; false when another asset owns Conductor. */
    bool owns_conductor {true};
    /**
     * The local Conductor's max_delay: how far world time may run ahead of the
     * slowest asset. Used only when owns_conductor.
     */
    std::uint64_t conductor_max_delay_usec {100000};
    /**
     * Lockstep inputs, "robot/pdu" (an Ackermann controller's command): each
     * step waits until a new value has arrived (runner::ILockstepInput). Needs
     * HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP.
     */
    std::vector<std::string> lockstep_inputs;
    /** Physics steps per Hakoniwa step (HakoniwaRunnerConfig::physics_substeps). */
    std::uint32_t physics_substeps {1};
    /** Vehicles supplied by `CarService/AckermannStep/<vehicle>`. */
    std::vector<std::string> ackermann_rpc_inputs;
    /** People supplied by `PersonService/PersonStep/<person>`. */
    std::vector<std::string> person_rpc_inputs;
    /** Robot carrying the viewer-compatible people state PDUs. */
    std::string person_state_robot {"UrbanPeople"};
    /** Core service definition JSON; required when any RPC input is non-empty. */
    std::string async_rpc_service_definition_path;
    /** Optional JSONL destination for person contact edges. */
    std::string person_contact_log_path;
};

/**
 * Top-level composition root from a declarative manifest file to an
 * application-facing Runner.
 *
 * ManifestFactory owns common document loading/path resolution and delegates
 * Runtime semantics to RuntimeFactory and execution construction to
 * RunnerFactory.
 */
class ManifestFactory final {
public:
    [[nodiscard]] static std::unique_ptr<runner::IRunner> create(
        const std::string& manifest_path,
        ManifestFactoryConfig config);
};

} // namespace hakoniwa::robot_runtime::factory
