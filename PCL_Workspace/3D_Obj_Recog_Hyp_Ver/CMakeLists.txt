cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(global_hypothesis_verification)

#Pcl
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (3D_Obj_Recog_Hyp_Ver 3D_Obj_Recog_Hyp_Ver.cpp)
target_link_libraries (3D_Obj_Recog_Hyp_Ver ${PCL_LIBRARIES})
