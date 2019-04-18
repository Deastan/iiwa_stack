cmake_minimum_required(VERSION 2.8.3)
project(iiwa_control)

find_package(catkin REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  control_msgs
  tf
  tf2
  tf2_ros
  tf2_msgs
  iiwa_msgs
  message_generation
  #Eigen3
  #Eigen REQUIRED
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rrbot_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

catkin_package(
  #INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  #INCLUDE_DIRS
  #include
#  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    control_msgs
    iiwa_msgs
    visualization_msgs
    std_msgs
    std_srvs
    nav_msgs
    message_runtime
#  DEPENDS #Eigen
)
