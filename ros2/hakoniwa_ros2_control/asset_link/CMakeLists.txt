# The Hakoniwa side of HakoniwaSystem, without ROS 2: the core and the PDU Endpoint only (docs/ros2-control.md, chapter 10).
# Builds alone against an installed Hakoniwa core:
#   cmake -S asset_link -B build -DCMAKE_PREFIX_PATH=<Hakoniwa install prefix>
cmake_minimum_required(VERSION 3.16)
project(hakoniwa_asset_link LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)  # the PDU Endpoint headers use std::span
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(hakoniwa-core REQUIRED)
find_package(hakoniwa_pdu_endpoint REQUIRED)
find_package(Threads REQUIRED)

add_library(hakoniwa_asset_link STATIC src/hakoniwa_asset_link.cpp)
set_target_properties(hakoniwa_asset_link PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(hakoniwa_asset_link PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(hakoniwa_asset_link PUBLIC hakoniwa-core::assets
  hakoniwa_pdu_endpoint::hakoniwa_pdu_endpoint Threads::Threads)

add_executable(hakoniwa_asset_link_probe tools/hakoniwa_asset_link_probe.cpp)
target_link_libraries(hakoniwa_asset_link_probe PRIVATE hakoniwa_asset_link)
