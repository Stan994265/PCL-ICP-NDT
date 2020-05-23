cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_icp_ndt)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(PCL  REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(pcl_icp pcl_icp.cpp)
target_link_libraries (pcl_icp ${PCL_LIBRARIES})

add_executable(pcl_ndt pcl_ndt.cpp)
target_link_libraries (pcl_ndt ${PCL_LIBRARIES})

add_executable(pcl_icp_continue pcl_icp_continue.cpp)
target_link_libraries (pcl_icp_continue ${PCL_LIBRARIES})

add_executable(pcl_ndt_continue pcl_ndt_continue.cpp)
target_link_libraries (pcl_ndt_continue ${PCL_LIBRARIES})
