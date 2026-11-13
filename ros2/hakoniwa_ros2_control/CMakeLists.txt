cmake_minimum_required(VERSION 3.16)
project(hakoniwa_ros2_control LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(hakoniwa_ros2_control SHARED src/hakoniwa_system.cpp)
target_include_directories(hakoniwa_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(hakoniwa_ros2_control PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS})

pluginlib_export_plugin_description_file(hardware_interface hakoniwa_ros2_control.xml)

install(TARGETS hakoniwa_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/move_to_joints.py scripts/hakoniwa_clock.py scripts/add_grasp_object.py scripts/pick_and_place.py scripts/detect_objects.py DESTINATION lib/${PROJECT_NAME})

# HakoniwaAssetSystem (docs/ros2-control.md, chapter 10): a separate plugin that
# is itself a Hakoniwa asset. Built only with -DHAKONIWA_ROS2_CONTROL_WITH_ASSET=ON
# and the Hakoniwa core and PDU Endpoint on CMAKE_PREFIX_PATH; without it this
# package is built as before.
option(HAKONIWA_ROS2_CONTROL_WITH_ASSET "Build the HakoniwaAssetSystem plugin" OFF)
if(HAKONIWA_ROS2_CONTROL_WITH_ASSET)
  add_subdirectory(asset_link)
  find_package(geometry_msgs REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(hakoniwa_pdu_endpoint REQUIRED)  # also nlohmann_json::nlohmann_json (bundled)
  add_library(hakoniwa_ros2_control_asset SHARED
    asset_link/src/hakoniwa_asset_system.cpp
    asset_link/src/hakoniwa_sensor_publisher.cpp)
  set_target_properties(hakoniwa_ros2_control_asset PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON)
  target_link_libraries(hakoniwa_ros2_control_asset PUBLIC
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${rosgraph_msgs_TARGETS}
    nlohmann_json::nlohmann_json
    hakoniwa_asset_link)
  pluginlib_export_plugin_description_file(hardware_interface hakoniwa_ros2_control_asset.xml)
  install(TARGETS hakoniwa_ros2_control_asset
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  install(TARGETS hakoniwa_asset_link_probe RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

# The controller plugin (docs/design-controller-plugin.md): ros2_control inside the
# Robot Runtime process, stepped in simulation time. Built only with
# -DHAKONIWA_ROS2_CONTROL_WITH_PLUGIN=ON; it needs the Hakoniwa PDU type headers
# (header-only converters) on CMAKE_PREFIX_PATH, and links no Hakoniwa library
# (the Robot Runtime that loads it has its own).
option(HAKONIWA_ROS2_CONTROL_WITH_PLUGIN "Build the ros2_control controller plugin" OFF)
if(HAKONIWA_ROS2_CONTROL_WITH_PLUGIN)
  find_package(controller_manager REQUIRED)
  find_package(controller_manager_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(yaml_cpp_vendor REQUIRED)
  find_path(HAKONIWA_PDU_TYPES_INCLUDE_DIR pdu_primitive_ctypes.h
    PATH_SUFFIXES include/hakoniwa/pdu hakoniwa/pdu
    DOC "Directory of the Hakoniwa PDU type headers (pdu_primitive_ctypes.h)")
  if(NOT HAKONIWA_PDU_TYPES_INCLUDE_DIR)
    message(FATAL_ERROR "Hakoniwa PDU type headers not found: set HAKONIWA_PDU_TYPES_INCLUDE_DIR")
  endif()
  set(HAKONIWA_ROBOT_RUNTIME_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../include"
    CACHE PATH "Robot Runtime public headers (the plugin C ABI)")
  add_library(hakoniwa_ros2_control_plugin SHARED
    controller_plugin/src/ros2_control_plugin.cpp
    controller_plugin/src/hakoniwa_plugin_system.cpp
    controller_plugin/src/plugin_bridge.cpp
    controller_plugin/src/sensor_relay.cpp)
  set_target_properties(hakoniwa_ros2_control_plugin PROPERTIES
    CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_VISIBILITY_PRESET hidden)
  target_include_directories(hakoniwa_ros2_control_plugin PRIVATE
    ${HAKONIWA_ROBOT_RUNTIME_INCLUDE_DIR}
    ${HAKONIWA_PDU_TYPES_INCLUDE_DIR})
  target_link_libraries(hakoniwa_ros2_control_plugin PRIVATE
    controller_manager::controller_manager
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${controller_manager_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${rosgraph_msgs_TARGETS}
    yaml-cpp)
  pluginlib_export_plugin_description_file(hardware_interface hakoniwa_ros2_control_plugin.xml)
  install(TARGETS hakoniwa_ros2_control_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
endif()

ament_package()
