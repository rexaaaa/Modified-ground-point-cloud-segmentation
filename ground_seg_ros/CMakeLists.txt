cmake_minimum_required(VERSION 3.0.2)
project(ground_seg_ros)
find_package(catkin_simple 0.1.0 REQUIRED )

catkin_simple(ALL_DEPS_REQUIRED)
find_library(GLOG_PATH glog)

add_definitions(-std=c++11 -g)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_executable(ground_seg_node src/ground_segmentation_node.cc)
#add_executable(ground_segmentation_test_node src/ground_segmentation_test_node.cc)
target_link_libraries(ground_seg_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GLOG_PATH})
#target_link_libraries(ground_segmentation_test_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GLOG_PATH})

cs_install()
cs_export()

FILE(GLOB_RECURSE LibFiles "include/*")

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
