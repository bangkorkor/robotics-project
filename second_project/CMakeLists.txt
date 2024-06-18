cmake_minimum_required(VERSION 3.0.2)
project(second_project)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gmapping
  nav_msgs
  pointcloud_to_laserscan
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)


## Declare a catkin package
catkin_package()

## Build
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_to_tf2 src/odom_to_tf2.cpp)
target_link_libraries(odom_to_tf2 ${catkin_LIBRARIES})