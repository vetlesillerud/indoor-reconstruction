cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(binary_to_ascii)

set (CMAKE_CXX_STANDARD 11)
find_package(PCL 1.9 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (converter binary_to_ascii.cpp)
target_link_libraries (converter ${PCL_LIBRARIES})



