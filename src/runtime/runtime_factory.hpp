#pragma once

#include "runtime/actuator_runtime.hpp"
#include "runtime/plant/actuator_plant.hpp"
#include "runtime/publisher/joint_state_publisher.hpp"
#include "runtime/publisher/multi_dof_joint_state_publisher.hpp"
#include "runtime/runtime_definition.hpp"
#include "runtime/source/joint_trajectory_command_source.hpp"
#include "runtime/source/joy_command_source.hpp"
#include "runtime/source/scalar_pdu_command_source.hpp"
#include "runtime/source/ackermann_drive_command_source.hpp"
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR
#include "runtime/publisher/impulse_collision_publisher.hpp"
#include "runtime/source/mirror_body_state_source.hpp"
#endif

#include <functional>
#include <memory>

namespace hakoniwa::robot_runtime::runtime {

using Float64EventReaderFactory = std::function<
    std::shared_ptr<IFloat64EventReader>(const RuntimeActuatorConfig&)>;
using JointStateWriterFactory = std::function<
    std::shared_ptr<IJointStateWriter>(const RuntimeStateOutputConfig&)>;
using MultiDofJointStateWriterFactory = std::function<
    std::shared_ptr<IMultiDofJointStateWriter>(
        const RuntimeMultiDofStateOutputConfig&)>;
using JointTrajectoryEventReaderFactory = std::function<
    std::shared_ptr<IJointTrajectoryEventReader>(
        const RuntimeTrajectoryControllerConfig&)>;
using JoyEventReaderFactory = std::function<
    std::shared_ptr<IJoyEventReader>(const RuntimeManualControllerConfig&)>;
using AckermannDriveEventReaderFactory = std::function<
    std::shared_ptr<IAckermannDriveEventReader>(
        const RuntimeAckermannControllerConfig&)>;
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR
using MirrorBodyStateReaderFactory = std::function<
    std::shared_ptr<IMirrorBodyStateReader>(const RuntimeMirrorBodyConfig&)>;
using ImpulseCollisionWriterFactory = std::function<
    std::shared_ptr<IImpulseCollisionWriter>(
        const RuntimeImpulseCollisionOutputConfig&)>;
#endif

/**
 * Builds one ActuatorRuntime from an already-resolved RuntimeDefinition and
 * injected boundary implementations.
 *
 * RuntimeFactory owns Runtime composition only. It does not choose the
 * transport, physics backend, or execution Runner. Those concrete choices are
 * made by the outer composition root and injected here through the Plant and
 * reader/writer factories.
 */
[[nodiscard]] std::unique_ptr<ActuatorRuntime> build_runtime(
    const RuntimeDefinition& definition,
    std::shared_ptr<IActuatorPlant> plant,
    const Float64EventReaderFactory& reader_factory,
    const JointStateWriterFactory& writer_factory = {},
    const JointTrajectoryEventReaderFactory& trajectory_reader_factory = {},
    const JoyEventReaderFactory& joy_reader_factory = {},
    const AckermannDriveEventReaderFactory& ackermann_reader_factory = {},
    const MultiDofJointStateWriterFactory& multi_dof_writer_factory = {}
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR) && HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR
    , const MirrorBodyStateReaderFactory& mirror_reader_factory = {}
    , const ImpulseCollisionWriterFactory& impulse_writer_factory = {}
#endif
    );

} // namespace hakoniwa::robot_runtime::runtime
