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
set(RUNTIME_SRC "${RUNTIME_ROOT}/src")
set(RUNTIME_INCLUDE "${RUNTIME_ROOT}/include")

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

# The sample plugin and its broken variants for the loader checks.
function(add_hold_target_plugin name)
    add_library(${name} MODULE hold_target_plugin.c)
    target_include_directories(${name} PRIVATE ${RUNTIME_INCLUDE})
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
target_include_directories(controller_plugin_header_c PRIVATE ${RUNTIME_INCLUDE})
set_target_properties(controller_plugin_header_c PROPERTIES C_STANDARD 99 C_EXTENSIONS OFF)
target_compile_options(controller_plugin_header_c PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(hako-controller-plugin-test
    ${RUNTIME_ROOT}/tests/test_controller_plugin.cpp
    ${RUNTIME_SRC}/plugin/plugin_library.cpp
    ${RUNTIME_SRC}/runtime/controller/plugin_controller.cpp
    ${RUNTIME_SRC}/runtime/runtime_factory_plugin.cpp
    ${RUNTIME_SRC}/runtime/runtime_factory_manifest.cpp
    ${RUNTIME_SRC}/runtime/runtime_factory_actuator.cpp
    ${RUNTIME_SRC}/runtime/runtime_factory_trajectory.cpp
    ${RUNTIME_SRC}/runtime/runtime_factory_manual.cpp
    ${RUNTIME_SRC}/runtime/runtime_factory_state_output.cpp
    ${RUNTIME_SRC}/runtime/runtime_factory.cpp
    ${RUNTIME_SRC}/runtime/actuator_runtime.cpp
    ${RUNTIME_SRC}/runtime/source/scalar_pdu_command_source.cpp
    ${RUNTIME_SRC}/runtime/source/joint_trajectory_command_source.cpp
    ${RUNTIME_SRC}/runtime/source/joy_command_source.cpp
    ${RUNTIME_SRC}/runtime/controller/scalar_pdu_command_controller.cpp
    ${RUNTIME_SRC}/runtime/controller/joint_trajectory_controller.cpp
    ${RUNTIME_SRC}/runtime/controller/manual_controller.cpp
    ${RUNTIME_SRC}/runtime/controller/hold_controller.cpp
    ${RUNTIME_SRC}/runtime/arbiter/priority_command_arbiter.cpp
    ${RUNTIME_SRC}/runtime/publisher/joint_state_publisher.cpp
)
target_compile_features(hako-controller-plugin-test PRIVATE cxx_std_20)
target_compile_options(hako-controller-plugin-test PRIVATE -Wall -Wextra)
target_compile_definitions(hako-controller-plugin-test PRIVATE
    HAKONIWA_ROBOT_RUNTIME_ENABLE_CONTROLLER_PLUGIN=1
    HOLD_TARGET_PLUGIN_PATH="$<TARGET_FILE:hold_target>"
    HOLD_TARGET_BAD_KIND_PATH="$<TARGET_FILE:hold_target_bad_kind>"
    HOLD_TARGET_BAD_ABI_PATH="$<TARGET_FILE:hold_target_bad_abi>"
    HOLD_TARGET_SMALL_TABLE_PATH="$<TARGET_FILE:hold_target_small_table>"
    HOLD_TARGET_NO_ENTRY_PATH="$<TARGET_FILE:hold_target_no_entry>")
target_include_directories(hako-controller-plugin-test PRIVATE
    ${RUNTIME_INCLUDE}
    ${RUNTIME_SRC})
target_include_directories(hako-controller-plugin-test SYSTEM PRIVATE
    ${NLOHMANN_JSON_INCLUDE_DIR})
target_link_libraries(hako-controller-plugin-test PRIVATE ${CMAKE_DL_LIBS})
add_dependencies(hako-controller-plugin-test
    hold_target hold_target_bad_kind hold_target_bad_abi
    hold_target_small_table hold_target_no_entry)
add_test(NAME hako-controller-plugin-contract COMMAND hako-controller-plugin-test)
