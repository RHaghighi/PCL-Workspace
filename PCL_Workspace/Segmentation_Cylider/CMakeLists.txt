cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(PCL_openni_viewer)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(Segment_Cylinder Segment_Cylinder.cpp)
 
target_link_libraries (Segment_Cylinder ${PCL_LIBRARIES})
