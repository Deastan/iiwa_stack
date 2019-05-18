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
  #Eigen3 REQUIRED
  moveit_ros_planning_interface
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rrbot_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
# TRY
#find_package(Eigen3 REQUIRED) include_directories(${EIGEN3_INCLUDE_DIR})

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
    #DEPENDS Eigen3
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Declare a cpp library
#add_library(contact src/move.cpp)

## Declare a cpp executable
add_executable(move_cpp src/move.cpp)
target_link_libraries(move_cpp ${catkin_LIBRARIES})

## Specify libraries to link a library or executable target against
#target_link_libraries(contact ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


## Declare a cpp executable
#add_executable(hand_visualizer2 src/hand_visualizer.cpp)
#add_executable(reflex imu.py)

## Specify libraries to link a library or executable target against
#target_link_libraries(hand_visualizer2 ${catkin_LIBRARIES})
#add_dependencies(hand_visualizer2 reflex_msgs2_gencpp)
