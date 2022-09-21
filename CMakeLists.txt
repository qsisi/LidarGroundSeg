cmake_minimum_required(VERSION 3.10)

project(lidar_ground_seg)
SET(CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)
include_directories(${YAMLCPP_INCLUDE_DIRS})

add_executable (main main.cpp)
target_link_libraries (main ${PCL_LIBRARIES} ${YAMLCPP_LIBRARIES})
