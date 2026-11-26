# Plugin contract tests (PDU headers only, no Hakoniwa, MuJoCo or ROS libraries).
#
#   cmake -S tests/plugins -B /tmp/hako-plugin-test -DCMAKE_BUILD_TYPE=Debug \
#         -DNLOHMANN_JSON_INCLUDE_DIR=<dir that contains nlohmann/json.hpp>
#   cmake --build /tmp/hako-plugin-test
#   ctest --test-dir /tmp/hako-plugin-test --output-on-failure
#
# Build in Debug: the test checks with assert() and refuses to compile with NDEBUG.
cmake_minimum_required(VERSION 3.21)
project(hakoniwa-robot-runtime-plugin-test LANGUAGES C CXX)

get_filename_component(RUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(HAKONIWA_FOUNDATION_PREFIX
    "${RUNTIME_ROOT}/../hakoniwa-business-pack/work/foundation/install"
    CACHE PATH "Hakoniwa Foundation installation (PDU support headers)")
set(HAKONIWA_PDU_REGISTRY_ROOT
    "${RUNTIME_ROOT}/../hakoniwa-pdu-registry"
    CACHE PATH "Hakoniwa PDU registry checkout (generated packet headers)")
include("${RUNTIME_ROOT}/cmake/HakoniwaRobotRuntimeSources.cmake")

set(NLOHMANN_JSON_INCLUDE_DIR
    "${RUNTIME_ROOT}/../hakoniwa-mujoco-robots/thirdparty/nolman/single_include"
    CACHE PATH "Directory that contains nlohmann/json.hpp")
if(NOT EXISTS "${NLOHMANN_JSON_INCLUDE_DIR}/nlohmann/json.hpp")
    message(FATAL_ERROR "nlohmann/json.hpp not found under ${NLOHMANN_JSON_INCLUDE_DIR}")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

enable_testing()
add_test(NAME hako-runtime-source-catalog
    COMMAND "${CMAKE_COMMAND}" -P "${RUNTIME_ROOT}/tests/check_runtime_sources.cmake")

# The sample plugin and its broken variants for the loader checks.
function(add_hold_target_plugin name)
    add_library(${name} MODULE hold_target_plugin.c)
    target_include_directories(${name} PRIVATE ${HAKO_RUNTIME_INCLUDE_DIR})
    set_target_properties(${name} PROPERTIES C_VISIBILITY_PRESET hidden PREFIX "lib")
    if(ARGN)
        target_compile_definitions(${name} PRIVATE ${ARGN})
    endif()
endfunction()
add_hold_target_plugin(hold_target)
add_hold_target_plugin(hold_target_bad_kind HOLD_TARGET_BAD_KIND)
add_hold_target_plugin(hold_target_bad_abi HOLD_TARGET_BAD_ABI)
add_hold_target_plugin(hold_target_small_table HOLD_TARGET_SMALL_TABLE)
add_hold_target_plugin(hold_target_no_entry HOLD_TARGET_NO_ENTRY)

# The header must compile as plain C (and as C++ through the test).
add_library(controller_plugin_header_c OBJECT header_check.c)
target_include_directories(controller_plugin_header_c PRIVATE ${HAKO_RUNTIME_INCLUDE_DIR})
set_target_properties(controller_plugin_header_c PROPERTIES C_STANDARD 99 C_EXTENSIONS OFF)
target_compile_options(controller_plugin_header_c PRIVATE -Wall -Wextra -Wpedantic -Werror)

set(PLUGIN_TEST_SOURCES
    ${RUNTIME_ROOT}/tests/test_controller_plugin.cpp
    ${HAKO_RUNTIME_CONTROLLER_PLUGIN_SOURCES}
    ${HAKO_RUNTIME_ROBOT_PLUGIN_SOURCES}
    ${HAKO_RUNTIME_CONTROLLER_PLUGIN_MANIFEST_SOURCES}
    ${HAKO_RUNTIME_ACTUATOR_MANIFEST_SOURCES}
    ${HAKO_RUNTIME_ACTUATOR_COMPONENTS_SOURCES}
    ${HAKO_RUNTIME_ACTUATOR_LOOP_SOURCES}
    ${HAKO_RUNTIME_SCALAR_PDU_COMMAND_SOURCES}
    ${HAKO_RUNTIME_HOLD_CONTROLLER_SOURCES}
)

function(add_plugin_test name)
    add_executable(${name} ${PLUGIN_TEST_SOURCES} ${ARGN})
    target_compile_features(${name} PRIVATE cxx_std_20)
    target_compile_options(${name} PRIVATE -Wall -Wextra)
    target_compile_definitions(${name} PRIVATE
        ${HAKO_RUNTIME_CONTROLLER_PLUGIN_DEFINE}=1
        HOLD_TARGET_PLUGIN_PATH="$<TARGET_FILE:hold_target>"
        HOLD_TARGET_BAD_KIND_PATH="$<TARGET_FILE:hold_target_bad_kind>"
        HOLD_TARGET_BAD_ABI_PATH="$<TARGET_FILE:hold_target_bad_abi>"
        HOLD_TARGET_SMALL_TABLE_PATH="$<TARGET_FILE:hold_target_small_table>"
        HOLD_TARGET_NO_ENTRY_PATH="$<TARGET_FILE:hold_target_no_entry>")
    target_include_directories(${name} PRIVATE
        ${HAKO_RUNTIME_INCLUDE_DIR}
        ${HAKO_RUNTIME_SRC_DIR})
    target_include_directories(${name} SYSTEM PRIVATE
        ${NLOHMANN_JSON_INCLUDE_DIR}
        ${HAKONIWA_PDU_REGISTRY_ROOT}/pdu/types
        ${HAKO_RUNTIME_FOUNDATION_PDU_INCLUDE_DIR})
    target_link_libraries(${name} PRIVATE ${CMAKE_DL_LIBS})
    add_dependencies(${name}
        hold_target hold_target_bad_kind hold_target_bad_abi
        hold_target_small_table hold_target_no_entry)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

# Without the sensor stage: a plugin cannot list sensor PDUs.
add_plugin_test(hako-controller-plugin-test)

# With the sensor stage (needs hakoniwa-sensor-adapter for the sensor
# loader's headers): "pdus" are matched with the sensor outputs.
set(SENSOR_ADAPTER_INCLUDE_DIR
    "${RUNTIME_ROOT}/../hakoniwa-sensor-adapter/include"
    CACHE PATH "hakoniwa-sensor-adapter include directory")
if(EXISTS "${SENSOR_ADAPTER_INCLUDE_DIR}/hakoniwa/sensor_adapter/sensor_adapter_implementation.hpp")
    add_plugin_test(hako-controller-plugin-sensor-test
        ${HAKO_RUNTIME_SENSOR_MANIFEST_SOURCES}
        ${HAKO_RUNTIME_SENSOR_REGISTRY_SOURCES})
    target_compile_definitions(hako-controller-plugin-sensor-test PRIVATE
        ${HAKO_RUNTIME_SENSOR_DEFINE}=1
        ${HAKO_RUNTIME_SENSOR_CATALOG_DEFINITION})
    target_include_directories(hako-controller-plugin-sensor-test PRIVATE
        ${SENSOR_ADAPTER_INCLUDE_DIR})
else()
    message(WARNING "hakoniwa-sensor-adapter not found: the sensor variant of the test is skipped")
endif()

function(add_robot_plugin name)
    add_library(${name} MODULE robot_fixture_plugin.c)
    target_include_directories(${name} PRIVATE ${HAKO_RUNTIME_INCLUDE_DIR})
    set_target_properties(${name} PROPERTIES
        C_STANDARD 99 C_EXTENSIONS OFF C_VISIBILITY_PRESET hidden PREFIX "lib")
    target_compile_options(${name} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_definitions(${name} PRIVATE ${ARGN})
endfunction()
add_robot_plugin(robot_fixture)
add_robot_plugin(robot_bad_kind FIXTURE_BAD_KIND)
add_robot_plugin(robot_bad_abi FIXTURE_BAD_ABI)
add_robot_plugin(robot_small_table FIXTURE_SMALL_TABLE)
add_robot_plugin(robot_missing_step FIXTURE_MISSING_STEP)
add_robot_plugin(robot_no_cancel FIXTURE_NO_CANCEL)
add_robot_plugin(robot_failing_step FIXTURE_FAILING_STEP)

set(ROBOT_PLUGIN_TEST_SOURCES
    ${HAKO_RUNTIME_CONTROLLER_PLUGIN_SOURCES}
    ${HAKO_RUNTIME_ROBOT_PLUGIN_SOURCES}
    ${HAKO_RUNTIME_CONTROLLER_PLUGIN_MANIFEST_SOURCES}
    ${HAKO_RUNTIME_ACTUATOR_MANIFEST_SOURCES}
    ${HAKO_RUNTIME_ACTUATOR_COMPONENTS_SOURCES}
    ${HAKO_RUNTIME_ACTUATOR_LOOP_SOURCES}
    ${HAKO_RUNTIME_SCALAR_PDU_COMMAND_SOURCES}
    ${HAKO_RUNTIME_HOLD_CONTROLLER_SOURCES}
    ${HAKO_RUNTIME_ASYNC_PIPELINE_SOURCES})

function(add_robot_plugin_test name)
    add_executable(${name} ${ROBOT_PLUGIN_TEST_SOURCES} ${ARGN})
    target_compile_features(${name} PRIVATE cxx_std_20)
    target_compile_options(${name} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>)
    target_compile_definitions(${name} PRIVATE
        ${HAKO_RUNTIME_CONTROLLER_PLUGIN_DEFINE}=1 ${HAKO_RUNTIME_LOCKSTEP_DEFINE}=1)
    target_include_directories(${name} PRIVATE ${HAKO_RUNTIME_INCLUDE_DIR} ${HAKO_RUNTIME_SRC_DIR})
    target_include_directories(${name} SYSTEM PRIVATE ${NLOHMANN_JSON_INCLUDE_DIR}
        ${HAKONIWA_PDU_REGISTRY_ROOT}/pdu/types
        ${HAKO_RUNTIME_FOUNDATION_PDU_INCLUDE_DIR})
    target_link_libraries(${name} PRIVATE ${CMAKE_DL_LIBS})
endfunction()

# The fixture is also compiled into the test as a builtin (HAKO_ROBOT_PLUGIN_BUILTIN).
add_library(robot_fixture_builtin OBJECT robot_fixture_plugin.c)
target_include_directories(robot_fixture_builtin PRIVATE ${HAKO_RUNTIME_INCLUDE_DIR})
set_target_properties(robot_fixture_builtin PROPERTIES C_STANDARD 99 C_EXTENSIONS OFF)
target_compile_options(robot_fixture_builtin PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_compile_definitions(robot_fixture_builtin PRIVATE HAKO_ROBOT_PLUGIN_BUILTIN)

add_robot_plugin_test(hako-robot-plugin-test
    ${RUNTIME_ROOT}/tests/test_robot_plugin.cpp $<TARGET_OBJECTS:robot_fixture_builtin>)
add_test(NAME hako-robot-plugin-test COMMAND hako-robot-plugin-test
    "$<TARGET_FILE:robot_fixture>" "$<TARGET_FILE:robot_bad_kind>" "$<TARGET_FILE:robot_bad_abi>"
    "$<TARGET_FILE:robot_small_table>" "$<TARGET_FILE:robot_missing_step>"
    "$<TARGET_FILE:robot_no_cancel>" "$<TARGET_FILE:robot_failing_step>")

# The manifest side of plugin and builtin robots.
add_robot_plugin_test(hako-service-plugin-test ${RUNTIME_ROOT}/tests/test_service_plugin.cpp)
add_test(NAME hako-service-plugin-test COMMAND hako-service-plugin-test
    "$<TARGET_FILE:robot_fixture>" "$<TARGET_FILE:hold_target>")

add_executable(hako-manifest-root-test
    ${RUNTIME_ROOT}/tests/test_manifest_root.cpp
    ${RUNTIME_ROOT}/src/factory/manifest_factory_manifest.cpp)
target_compile_features(hako-manifest-root-test PRIVATE cxx_std_20)
target_include_directories(hako-manifest-root-test PRIVATE ${HAKO_RUNTIME_INCLUDE_DIR} ${HAKO_RUNTIME_SRC_DIR})
target_include_directories(hako-manifest-root-test SYSTEM PRIVATE ${NLOHMANN_JSON_INCLUDE_DIR})
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/old-services-manifest.json"
    CONTENT [=[{"robots":[],"services":[]}]=])
add_test(NAME hako-manifest-root-test COMMAND hako-manifest-root-test
    "${CMAKE_CURRENT_BINARY_DIR}/old-services-manifest.json")

# Stage 2c-3: the declaration JSON next to a plugin and the startup validator.
add_robot_plugin_test(hako-robot-plugin-declaration-test ${RUNTIME_ROOT}/tests/test_robot_plugin_declaration.cpp)
add_test(NAME hako-robot-plugin-declaration-test COMMAND hako-robot-plugin-declaration-test)

# Stage 2c-2: car and person as builtin robot plugins.
add_robot_plugin_test(hako-builtin-robot-plugins-test ${RUNTIME_ROOT}/tests/test_builtin_robot_plugins.cpp)
add_test(NAME hako-builtin-robot-plugins-test COMMAND hako-builtin-robot-plugins-test)

# Stage 2c-4: the C++ template layer (HakoCpp types through the generated converters).
set(HAKO_PDU_REGISTRY_TYPES "${RUNTIME_ROOT}/../hakoniwa-pdu-registry/pdu/types"
    CACHE PATH "hakoniwa-pdu-registry pdu/types (generated C and C++ PDU types)")
function(add_adder_plugin name)
    add_library(${name} MODULE adder_robot_plugin.cpp)
    target_compile_features(${name} PRIVATE cxx_std_20)
    target_include_directories(${name} PRIVATE ${HAKO_RUNTIME_INCLUDE_DIR} ${HAKO_PDU_REGISTRY_TYPES})
    set_target_properties(${name} PROPERTIES CXX_VISIBILITY_PRESET hidden PREFIX "lib")
    target_compile_options(${name} PRIVATE -Wall -Wextra)
    target_compile_definitions(${name} PRIVATE ${ARGN})
endfunction()
add_adder_plugin(adder_robot)
add_adder_plugin(adder_wrong_type ADDER_WRONG_TYPE)
add_adder_plugin(adder_throwing_step ADDER_THROWING_STEP)
add_robot_plugin_test(hako-robot-plugin-template-test ${RUNTIME_ROOT}/tests/test_robot_plugin_template.cpp)
target_include_directories(hako-robot-plugin-template-test PRIVATE ${HAKO_PDU_REGISTRY_TYPES})
add_test(NAME hako-robot-plugin-template-test COMMAND hako-robot-plugin-template-test
    "$<TARGET_FILE:adder_robot>" "$<TARGET_FILE:adder_wrong_type>" "$<TARGET_FILE:adder_throwing_step>")
