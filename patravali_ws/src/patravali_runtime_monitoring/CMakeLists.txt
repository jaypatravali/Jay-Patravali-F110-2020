cmake_minimum_required(VERSION 2.8.3)
project(patravali_runtime_monitoring)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs vesc_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(patravali_average src/patravali_average.cpp)
target_link_libraries(patravali_average ${catkin_LIBRARIES})


add_executable(patravali_average_vesc src/patravali_average_vesc.cpp)
target_link_libraries(patravali_average_vesc ${catkin_LIBRARIES})
