cmake_minimum_required(VERSION 3.21)
project(servoj_like_plugin LANGUAGES CXX)

set(HAKO_RUNTIME_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../include"
    CACHE PATH "Robot Runtime public headers")
set(HAKO_SERVOJ_PDU_TYPES "" CACHE PATH "Generated ServoJ PDU types directory")
set(HAKO_PDU_REGISTRY_TYPES "" CACHE PATH "Standard PDU Registry types directory")

foreach(header IN ITEMS
    "${HAKO_RUNTIME_INCLUDE_DIR}/hakoniwa/robot_runtime/plugin/service_plugin.h"
    "${HAKO_SERVOJ_PDU_TYPES}/my_robot_srvs_msgs/pdu_ctype_ServoJRequestPacket.h"
    "${HAKO_PDU_REGISTRY_TYPES}/pdu_primitive_ctypes.h")
    if(NOT EXISTS "${header}")
        message(FATAL_ERROR "Required header not found: ${header}")
    endif()
endforeach()

add_library(my_servoj MODULE servoj_like.cpp)
target_compile_features(my_servoj PRIVATE cxx_std_20)
target_include_directories(my_servoj PRIVATE "${HAKO_RUNTIME_INCLUDE_DIR}")
target_include_directories(my_servoj SYSTEM PRIVATE
    "${HAKO_SERVOJ_PDU_TYPES}" "${HAKO_PDU_REGISTRY_TYPES}")
set_target_properties(my_servoj PROPERTIES CXX_VISIBILITY_PRESET hidden)
