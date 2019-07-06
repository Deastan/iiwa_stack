#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "ros/ros.h"
// #include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"

#include "geometry_msgs/Pose.h"
#include <string>
#include <boost/thread.hpp>
#include <tf2/LinearMath/Quaternion.h>
// #include <ros/callback_queue.h>

// #include <sstream>
// #include <Eigen3/Eigen>

// private:
// Parameters for group




 

// Listener for the two messages
ros::Subscriber sub_pose;
ros::Subscriber sub_joint;

// void chatterCallback_pose(const std_msgs::Float64MultiArray::ConstPtr& msg);
void chatterCallback_pose(const geometry_msgs::Pose::ConstPtr& msg);
void chatterCallback_joint(const std_msgs::Float64MultiArray::ConstPtr& msg);