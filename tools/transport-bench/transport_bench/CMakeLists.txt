# One test driver, several transports (../README.md): ROS 2 messages, Hakoniwa PDU in
# shared memory, and optionally Gazebo's gz-transport. Needs the Hakoniwa core and the PDU Endpoint (for the PDU
# conversion headers) on CMAKE_PREFIX_PATH.
cmake_minimum_required(VERSION 3.16)
project(hakoniwa_transport_bench LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hakoniwa_transport_bench_msgs REQUIRED)
find_package(hakoniwa-core REQUIRED)
find_package(hakoniwa_pdu_endpoint REQUIRED)

add_executable(transport_bench src/transport_bench.cpp)
target_link_libraries(transport_bench PRIVATE rclcpp::rclcpp ${sensor_msgs_TARGETS} ${hakoniwa_transport_bench_msgs_TARGETS}
  hakoniwa-core::assets hakoniwa_pdu_endpoint::hakoniwa_pdu_endpoint)
# Optional third transport: Gazebo's gz-transport (Gazebo Harmonic in ROS 2 Jazzy).
find_package(gz_transport_vendor QUIET)
find_package(gz_msgs_vendor QUIET)
find_package(gz-transport13 QUIET)
find_package(gz-msgs10 QUIET)
if(gz-transport13_FOUND AND gz-msgs10_FOUND)
  target_compile_definitions(transport_bench PRIVATE HAKO_BENCH_WITH_GZ)
  target_link_libraries(transport_bench PRIVATE gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
  message(STATUS "transport_bench: gz-transport enabled")
endif()
install(TARGETS transport_bench RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
