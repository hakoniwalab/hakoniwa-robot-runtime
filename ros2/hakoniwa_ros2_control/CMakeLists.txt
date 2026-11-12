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
install(PROGRAMS scripts/move_to_joints.py DESTINATION lib/${PROJECT_NAME})

ament_package()
