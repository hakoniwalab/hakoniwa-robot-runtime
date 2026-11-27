cmake_minimum_required(VERSION 3.21)
project(servoj_like_plugin LANGUAGES CXX)

set(HAKO_RUNTIME_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../include"
    CACHE PATH "Robot Runtime public headers")
set(HAKO_SERVOJ_PDU_TYPES "" CACHE PATH "Generated SO-101 service PDU types directory")
set(HAKO_PDU_REGISTRY_TYPES "" CACHE PATH "Standard PDU Registry types directory")

set(required_headers
    "${HAKO_RUNTIME_INCLUDE_DIR}/hakoniwa/robot_runtime/plugin/robot_plugin.hpp"
    "${HAKO_PDU_REGISTRY_TYPES}/pdu_dynamic_memory.hpp")
foreach(service IN ITEMS ServoJ Stop GetAngle)
    list(APPEND required_headers
        "${HAKO_SERVOJ_PDU_TYPES}/my_robot_srvs_msgs/pdu_cpptype_conv_${service}RequestPacket.hpp"
        "${HAKO_SERVOJ_PDU_TYPES}/my_robot_srvs_msgs/pdu_cpptype_conv_${service}ResponsePacket.hpp")
endforeach()
foreach(header IN LISTS required_headers)
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

set(declaration "${CMAKE_CURRENT_SOURCE_DIR}/my_servoj.plugin.json")
set_property(TARGET my_servoj APPEND PROPERTY LINK_DEPENDS "${declaration}")
add_custom_command(TARGET my_servoj POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${declaration}"
        "$<TARGET_FILE_DIR:my_servoj>/$<TARGET_FILE_PREFIX:my_servoj>$<TARGET_FILE_BASE_NAME:my_servoj>.plugin.json"
    VERBATIM)
