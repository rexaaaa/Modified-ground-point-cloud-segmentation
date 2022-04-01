cmake_minimum_required(VERSION 3.0.2)
project(ground_seg)
find_package(catkin_simple 0.1.0 REQUIRED )
catkin_simple(ALL_DEPS_REQUIRED)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

cs_add_library(${PROJECT_NAME} src/ground_segmentation.cc src/segment.cc src/bin.cc)

cs_install()
cs_export()

FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(rexcool SOURCES ${LibFiles})

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
