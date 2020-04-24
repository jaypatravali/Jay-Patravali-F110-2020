cmake_minimum_required(VERSION 2.8.3)
project(patravali_wall_following)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES error_analysis.msg)
 
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
 