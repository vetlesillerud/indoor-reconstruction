cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(vlp16_pcd_writer)

set (CMAKE_CXX_STANDARD 11)
find_package(PCL 1.9 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (vlp ../Tinkerforge_IMU2.0/brick_imu_v2.cpp
                    ../Tinkerforge_IMU2.0/brick_imu_v2.h
                    ../Tinkerforge_IMU2.0/ip_connection.cpp
                    ../Tinkerforge_IMU2.0/ip_connection.h
                    vlp.cpp)


target_link_libraries (vlp ${PCL_LIBRARIES})



