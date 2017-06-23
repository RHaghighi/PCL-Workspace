cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(correspondence_grouping)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(-std=c++11)

add_executable (pcd2ply pcd2ply.cpp)
target_link_libraries (pcd2ply ${PCL_LIBRARIES})
