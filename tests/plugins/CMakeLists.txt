# Controller plugin contract test (standalone; no Hakoniwa, MuJoCo or ROS).
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
        ${NLOHMANN_JSON_INCLUDE_DIR})
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

function(add_service_plugin name)
    add_library(${name} MODULE service_fixture_plugin.c)
    target_include_directories(${name} PRIVATE ${HAKO_RUNTIME_INCLUDE_DIR})
    set_target_properties(${name} PROPERTIES
        C_STANDARD 99 C_EXTENSIONS OFF C_VISIBILITY_PRESET hidden PREFIX "lib")
    target_compile_options(${name} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_definitions(${name} PRIVATE ${ARGN})
endfunction()
add_service_plugin(service_fixture)
add_service_plugin(service_bad_kind SERVICE_BAD_KIND)
add_service_plugin(service_bad_abi SERVICE_BAD_ABI)
add_service_plugin(service_small_table SERVICE_SMALL_TABLE)
add_service_plugin(service_missing_step SERVICE_MISSING_STEP)
add_service_plugin(service_no_optional SERVICE_NO_OPTIONAL)
add_service_plugin(service_failing_update SERVICE_FAILING_UPDATE)

add_executable(hako-service-plugin-test
    ${RUNTIME_ROOT}/tests/test_service_plugin.cpp
    ${HAKO_RUNTIME_CONTROLLER_PLUGIN_SOURCES}
    ${HAKO_RUNTIME_CONTROLLER_PLUGIN_MANIFEST_SOURCES}
    ${HAKO_RUNTIME_ACTUATOR_MANIFEST_SOURCES}
    ${HAKO_RUNTIME_ACTUATOR_COMPONENTS_SOURCES}
    ${HAKO_RUNTIME_ACTUATOR_LOOP_SOURCES}
    ${HAKO_RUNTIME_SCALAR_PDU_COMMAND_SOURCES}
    ${HAKO_RUNTIME_HOLD_CONTROLLER_SOURCES}
    ${HAKO_RUNTIME_ASYNC_PIPELINE_SOURCES})
target_compile_features(hako-service-plugin-test PRIVATE cxx_std_20)
target_compile_options(hako-service-plugin-test PRIVATE -Wall -Wextra)
target_compile_definitions(hako-service-plugin-test PRIVATE
    ${HAKO_RUNTIME_CONTROLLER_PLUGIN_DEFINE}=1 ${HAKO_RUNTIME_LOCKSTEP_DEFINE}=1)
target_include_directories(hako-service-plugin-test PRIVATE
    ${HAKO_RUNTIME_INCLUDE_DIR} ${HAKO_RUNTIME_SRC_DIR})
target_include_directories(hako-service-plugin-test SYSTEM PRIVATE ${NLOHMANN_JSON_INCLUDE_DIR})
target_link_libraries(hako-service-plugin-test PRIVATE ${CMAKE_DL_LIBS})
add_test(NAME hako-service-plugin-test COMMAND hako-service-plugin-test
    "$<TARGET_FILE:service_fixture>" "$<TARGET_FILE:hold_target>"
    "$<TARGET_FILE:service_bad_kind>" "$<TARGET_FILE:service_bad_abi>"
    "$<TARGET_FILE:service_small_table>" "$<TARGET_FILE:service_missing_step>"
    "$<TARGET_FILE:service_no_optional>" "$<TARGET_FILE:service_failing_update>")
