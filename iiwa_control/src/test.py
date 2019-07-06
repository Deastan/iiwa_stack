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

import geometry_msgs.msg


from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState





if __name__ == '__main__':

    # rospy.init_node('My_test', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

    group.set_planner_id("RRTkConfigDefault")

    #********************CODE****************
    # # We can get the name of the reference frame for this robot:
    # planning_frame = group.get_planning_frame()
    # print "============ Reference frame: %s" % planning_frame

    # # We can also print the name of the end-effector link for this group:
    # eef_link = group.get_end_effector_link()
    # print "============ End effector: %s" % eef_link

    # # We can get a list of all the groups in the robot:
    # group_names = robot.get_group_names()
    # print "============ Robot Groups:", robot.get_group_names()

    # # Sometimes for debugging it is useful to print the entire state of the
    # # robot:
    # print "============ Printing robot state"
    # print robot.get_current_state()
    # print ""
    #********************OUTPUT****************
    # ============ Reference frame: /world
    # ============ End effector: iiwa_link_ee
    # ============ Robot Groups: ['manipulator']
    # ============ Printing robot state

    state_trajectory = 1

    if state_trajectory == 1:
        increment =  0.0
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = 0.5
        
        # state = robot.get_current_state()
        # print(state)
        # group.set_start_state(state)

        # group.get_current_pose()
        t = 0
        while t<15:
            # state = robot.get_current_state()
            # print("Position: ( ", state.joint_state.position[0], ", ", state.joint_state.position[1], ", ", state.joint_state.position[2], " )")
            # state.joint_state.position[0] = 0.5
            # state.joint_state.position[1] = 0.5
            # state.joint_state.position[2] = 0.5
            # state.joint_state.position = [0.0, 0.0, 0.0, -1.5770, 0.0, -1.9886443826777622e-05, 1.4264836100785772e-05, 0.0001574951690832549, -5.156308670706267e-06, 0.12109634420144744, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00015744801537298514, -4.287808235581281e-06, 0.12108910820644159, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -8.576287356731882e-06, 0.12174941365793668, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # print("Position: ( ", state.joint_state.position[0], ", ", state.joint_state.position[1], ", ", state.joint_state.position[2], " )")
            # joint_state = JointState()
            # joint_state.header = Header()
            # joint_state.header.stamp = rospy.Time.now()
            # joint_state.name = ['joint_a', 'joint_b']
            # joint_state.position = [0.17, 0.34]
            # moveit_robot_state = RobotState()
            # moveit_robot_state.joint_state = joint_state
            # group.set_start_state(moveit_robot_state)
            
            # group.set_start_state(state)

            # print(group.set_start_state_to_current_state())
            # print(group.get_current_pose())
            pose_goal.orientation.w = 1.0
            pose_goal.position.x = 0.7*math.cos(increment)
            pose_goal.position.y = 0.7*math.sin(increment)
            pose_goal.position.z = 0.7#+0.2*math.sin(increment)
            group.set_pose_target(pose_goal)


            # self.plan = self.group.plan()
            # result = self.group.go(wait=True)

            plan = group.go(wait=True)
            # print(plan)
            # rospy.sleep(0.0001)
            # print(plan)
            # Calling `stop()` ensures that there is no residual movement
            group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            group.clear_pose_targets()

            # state = robot.get_current_state()
            # print(state)
            # group.set_start_state(state)
            # rospy.sleep(3.)
            # group.set_start_state_to_current_state()
            increment += 0.078539816339
            t+=1

            # rospy.spin()

    elif state_trajectory == 2:
        scale = 1.0
        waypoints = []

        wpose = group.get_current_pose().pose
        wpose.orientation.w = 1.0
        wpose.position.x = 0.5#*math.cos(increment)
        wpose.position.y = 0.5#*math.sin(increment)
        wpose.position.z = 0.5#+0.2*math.sin(increment)

        wpose.position.z += scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.1,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        # return plan, fraction

        group.execute(plan, wait=True)

    else:   
        while True:

            # state = robot.get_current_state()
            # print(state)
            print(group.get_current_pose().pose)
        print("Noting to do!!!!")


    
    group.clear_pose_targets()
    print("End: test.py")
    