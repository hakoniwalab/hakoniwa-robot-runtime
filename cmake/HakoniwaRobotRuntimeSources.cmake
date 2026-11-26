# Source groups for consumers that own their CMake targets.
# Use the groups with add_library(), add_executable() or target_sources().
# Keep each src/ implementation in exactly one group. The standalone catalog
# test checks this list against src/. Consumers select groups and the matching
# feature definitions based on what their targets build.

get_filename_component(HAKO_RUNTIME_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(HAKO_RUNTIME_INCLUDE_DIR "${HAKO_RUNTIME_ROOT}/include")
set(HAKO_RUNTIME_SRC_DIR "${HAKO_RUNTIME_ROOT}/src")
set(HAKO_RUNTIME_THIRD_PARTY_INCLUDE_DIR "${HAKO_RUNTIME_ROOT}/third_party")

# These are separate so callers can keep their existing include order and
# visibility. Foundation-free consumers do not need to set a prefix.
if(HAKONIWA_FOUNDATION_PREFIX)
    set(HAKO_RUNTIME_FOUNDATION_INCLUDE_DIR "${HAKONIWA_FOUNDATION_PREFIX}/include")
    set(HAKO_RUNTIME_FOUNDATION_HAKONIWA_INCLUDE_DIR "${HAKO_RUNTIME_FOUNDATION_INCLUDE_DIR}/hakoniwa")
    set(HAKO_RUNTIME_FOUNDATION_PDU_INCLUDE_DIR "${HAKO_RUNTIME_FOUNDATION_HAKONIWA_INCLUDE_DIR}/pdu")
endif()

# Apply the same values to any targets that share the corresponding types.
set(HAKO_RUNTIME_MOBILE_BASE_DEFINE HAKONIWA_ROBOT_RUNTIME_ENABLE_MOBILE_BASE)
set(HAKO_RUNTIME_LOCKSTEP_DEFINE HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP)
set(HAKO_RUNTIME_PLANT_DIRECTIVE_DEFINE HAKONIWA_ROBOT_RUNTIME_ENABLE_PLANT_DIRECTIVE)
set(HAKO_RUNTIME_SENSOR_DEFINE HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR)
set(HAKO_RUNTIME_CONTROLLER_PLUGIN_DEFINE HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN)
set(HAKO_RUNTIME_SENSOR_CATALOG_DEFINITION
    HAKONIWA_ROBOT_RUNTIME_SENSOR_CATALOG_PATH="${HAKO_RUNTIME_ROOT}/catalog/sensor-implementations.json")

# Actuator loop, standard components, and manifest parsing.
set(HAKO_RUNTIME_ACTUATOR_LOOP_SOURCES
    "${HAKO_RUNTIME_SRC_DIR}/runtime/actuator_runtime.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/arbiter/priority_command_arbiter.cpp"
)

set(HAKO_RUNTIME_ACTUATOR_COMPONENTS_SOURCES
    "${HAKO_RUNTIME_SRC_DIR}/runtime/runtime_factory.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/source/joint_trajectory_command_source.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/source/joy_command_source.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/controller/scalar_pdu_command_controller.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/controller/joint_trajectory_controller.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/controller/manual_controller.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/publisher/joint_state_publisher.cpp"
)

set(HAKO_RUNTIME_SCALAR_PDU_COMMAND_SOURCES "${HAKO_RUNTIME_SRC_DIR}/runtime/source/scalar_pdu_command_source.cpp")

set(HAKO_RUNTIME_HOLD_CONTROLLER_SOURCES "${HAKO_RUNTIME_SRC_DIR}/runtime/controller/hold_controller.cpp")

set(HAKO_RUNTIME_ACTUATOR_MANIFEST_SOURCES
    "${HAKO_RUNTIME_SRC_DIR}/runtime/runtime_factory_manifest.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/runtime_factory_actuator.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/runtime_factory_trajectory.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/runtime_factory_manual.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/runtime_factory_state_output.cpp"
)

set(HAKO_RUNTIME_MANIFEST_FACTORY_SOURCES
    "${HAKO_RUNTIME_SRC_DIR}/factory/manifest_factory.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/factory/manifest_factory_manifest.cpp"
)

set(HAKO_RUNTIME_ACTUATOR_PDU_SOURCES
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/float64_pdu_event_reader.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/joint_state_pdu_writer.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/joint_trajectory_pdu_event_reader.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/joy_pdu_event_reader.cpp"
)

# Physics and runner entry points. Tests also compile individual pieces.
set(HAKO_RUNTIME_MUJOCO_PLANT_SOURCES "${HAKO_RUNTIME_SRC_DIR}/adapters/physics/mujoco/mujoco_actuator_plant.cpp")

set(HAKO_RUNTIME_RUNNER_FACTORY_SOURCES "${HAKO_RUNTIME_SRC_DIR}/runner/runner_factory.cpp")

set(HAKO_RUNTIME_LOCKSTEP_GATE_SOURCES "${HAKO_RUNTIME_SRC_DIR}/runner/lockstep_gate.cpp")

set(HAKO_RUNTIME_HAKONIWA_ASSET_DRIVER_SOURCES "${HAKO_RUNTIME_SRC_DIR}/runner/hakoniwa/hakoniwa_asset_driver.cpp")

set(HAKO_RUNTIME_HAKONIWA_RUNNER_SOURCES "${HAKO_RUNTIME_SRC_DIR}/runner/hakoniwa/hakoniwa_runner.cpp")

# Async RPC pipeline, Core transport, and typed step routes.
set(HAKO_RUNTIME_ASYNC_PIPELINE_SOURCES "${HAKO_RUNTIME_SRC_DIR}/runtime/service/async_service_pipeline.cpp")

set(HAKO_RUNTIME_CORE_ASYNC_SERVER_SOURCES "${HAKO_RUNTIME_SRC_DIR}/runner/hakoniwa/core_async_service_server.cpp")

set(HAKO_RUNTIME_ACKERMANN_STEP_READER_SOURCES "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/ackermann_drive_rpc_event_reader.cpp")

set(HAKO_RUNTIME_ACKERMANN_STEP_SERVICE_SOURCES "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/ackermann_step_service.cpp")

set(HAKO_RUNTIME_PERSON_STEP_READER_SOURCES "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/person_step_rpc_event_reader.cpp")

set(HAKO_RUNTIME_PERSON_STEP_SERVICE_SOURCES "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/person_step_service.cpp")

# Mobile Base sources (HAKO_RUNTIME_MOBILE_BASE_DEFINE).
set(HAKO_RUNTIME_MOBILE_BASE_COMMAND_SOURCES
    "${HAKO_RUNTIME_SRC_DIR}/runtime/source/ackermann_drive_command_source.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/controller/ackermann_controller.cpp"
)

set(HAKO_RUNTIME_MOBILE_BASE_STATE_SOURCES "${HAKO_RUNTIME_SRC_DIR}/runtime/publisher/multi_dof_joint_state_publisher.cpp")

set(HAKO_RUNTIME_MOBILE_BASE_MANIFEST_SOURCES "${HAKO_RUNTIME_SRC_DIR}/runtime/runtime_factory_ackermann.cpp")

set(HAKO_RUNTIME_MOBILE_BASE_PDU_SOURCES
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/ackermann_drive_pdu_event_reader.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/multi_dof_joint_state_pdu_writer.cpp"
)

# Plant Directive sources (HAKO_RUNTIME_PLANT_DIRECTIVE_DEFINE).
set(HAKO_RUNTIME_MIRROR_RUNTIME_SOURCES
    "${HAKO_RUNTIME_SRC_DIR}/runtime/source/mirror_body_state_source.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/controller/mirror_body_controller.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/publisher/impulse_collision_publisher.cpp"
)

set(HAKO_RUNTIME_GEOM_FRICTION_SOURCES "${HAKO_RUNTIME_SRC_DIR}/runtime/controller/geom_friction_controller.cpp")

set(HAKO_RUNTIME_PLANT_DIRECTIVE_MANIFEST_SOURCES
    "${HAKO_RUNTIME_SRC_DIR}/runtime/runtime_factory_mirror.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/runtime_factory_geom_friction.cpp"
)

set(HAKO_RUNTIME_PLANT_DIRECTIVE_PDU_SOURCES
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/mirror_body_twist_pdu_reader.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/impulse_collision_pdu_writer.cpp"
)

# Sensor sources (HAKO_RUNTIME_SENSOR_DEFINE).
set(HAKO_RUNTIME_SENSOR_REGISTRY_SOURCES "${HAKO_RUNTIME_SRC_DIR}/runtime/sensor/sensor_registry.cpp")

set(HAKO_RUNTIME_SENSOR_PUBLISHERS_SOURCES
    "${HAKO_RUNTIME_SRC_DIR}/runtime/publisher/sensor_joint_state_publisher.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/publisher/sensor_wrench_publisher.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/publisher/sensor_imu_publisher.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/publisher/sensor_laser_scan_publisher.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/publisher/sensor_point_cloud_publisher.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/publisher/sensor_range_publisher.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/publisher/sensor_depth_image_publisher.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/publisher/sensor_compressed_image_publisher.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/publisher/sensor_magnetic_field_publisher.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/publisher/sensor_fluid_pressure_publisher.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/publisher/sensor_nav_sat_fix_publisher.cpp"
)

set(HAKO_RUNTIME_SENSOR_MUJOCO_SOURCES "${HAKO_RUNTIME_SRC_DIR}/adapters/physics/mujoco/mujoco_sensor_stage.cpp")

set(HAKO_RUNTIME_SENSOR_PDU_SOURCES
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/wrench_stamped_pdu_writer.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/imu_pdu_writer.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/laser_scan_pdu_writer.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/point_cloud2_pdu_writer.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/range_pdu_writer.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/depth_image_pdu_writer.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/compressed_image_pdu_writer.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/magnetic_field_pdu_writer.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/fluid_pressure_pdu_writer.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/adapters/endpoint/nav_sat_fix_pdu_writer.cpp"
)

set(HAKO_RUNTIME_SENSOR_MANIFEST_SOURCES
    "${HAKO_RUNTIME_SRC_DIR}/runtime/runtime_factory_sensor.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/sensor/sensor_catalog.cpp"
)

# Dynamic controller and service plugin sources (HAKO_RUNTIME_CONTROLLER_PLUGIN_DEFINE).
set(HAKO_RUNTIME_CONTROLLER_PLUGIN_SOURCES
    "${HAKO_RUNTIME_SRC_DIR}/plugin/plugin_library.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/controller/plugin_controller.cpp"
    "${HAKO_RUNTIME_SRC_DIR}/runtime/service/service_plugin_adapter.cpp"
)

set(HAKO_RUNTIME_CONTROLLER_PLUGIN_MANIFEST_SOURCES "${HAKO_RUNTIME_SRC_DIR}/runtime/runtime_factory_plugin.cpp")
