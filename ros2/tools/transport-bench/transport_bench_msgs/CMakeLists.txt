# Fixed-size payloads (plain types), so that ROS 2 can lend them in shared memory.
cmake_minimum_required(VERSION 3.16)
project(hakoniwa_transport_bench_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Fixed1K.msg msg/Fixed64K.msg msg/Fixed320K.msg msg/Fixed1M.msg msg/Fixed4M.msg)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
