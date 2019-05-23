#!/usr/bin/env python2

import sys
import copy
import time

# Math + plot
import numpy as np
import math as math
from math import pi
import matplotlib.pyplot as plt

# Moveit
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import MoveItErrorCodes
# from moveit_python import MoveGroupInterface, PlanningSceneInterface

# RL
# import gym
# from gym import wrappers
# from gym import envs
# from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment

# ROS packages 
import rospy
import rospkg

# Messages
from std_msgs.msg import String
from control_msgs.msg import JointTrajectoryControllerState


import geometry_msgs.msg
from std_msgs.msg import String

        # state = robot.get_current_state()
        # # print(state)
        # group.set_start_state(state)


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)





def listener():
    # rospy.init_node('test_node_state', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':


    listener()
    
    # group.clear_pose_targets()
    print("End: test_node_state.py")
    