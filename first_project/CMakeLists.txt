cmake_minimum_required(VERSION 3.0.2)
project(first_project)



find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  dynamic_reconfigure
)


generate_dynamic_reconfigure_options(
  cfg/config.cfg
)


catkin_package(

)


include_directories(

  ${catkin_INCLUDE_DIRS}
)



add_executable(gps_to_odom src/gps_to_odom.cpp)
target_link_libraries(gps_to_odom ${catkin_LIBRARIES})

add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})

add_executable(lidar_remap src/lidar_remap.cpp)
add_dependencies(lidar_remap ${PROJECT_NAME}_gencfg)
target_link_libraries(lidar_remap ${catkin_LIBRARIES})


