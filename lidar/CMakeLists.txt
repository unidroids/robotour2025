cmake_minimum_required(VERSION 3.16)
project(robot_lidar_tcp)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
include_directories(${CMAKE_SOURCE_DIR}/unitree_lidar_sdk/include)
link_directories(${CMAKE_SOURCE_DIR}/unitree_lidar_sdk/lib/${CMAKE_SYSTEM_PROCESSOR})
add_executable(robot_lidar_tcp robot_lidar_tcp.cpp)
target_link_libraries(robot_lidar_tcp PRIVATE unilidar_sdk2 pthread)

