# One test driver, two transports (../README.md): ROS 2 messages or Hakoniwa PDU in
# shared memory. Needs the Hakoniwa core and the PDU Endpoint (for the PDU
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
install(TARGETS transport_bench RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
