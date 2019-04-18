#! /usr/bin/env python

# ET1 controller with camera_is1500
# Jonathan Burkhard, Kyburz 2018
# Documentations : Not exist yet

# TODO
# - Add gain
# - Clean the code with function, private variables etc...

# Basic imports
import sys
import rospy
import time
import tf
import numpy as np
import math
import tf.transformations
from geometry_msgs.msg import Quaternion, Point, Pose, Twist, Vector3
from std_msgs.msg import Empty, Float64
from nav_msgs.msg import Odometry
from iiwa_msgs.msg import JointPosition
from control_msgs.msg import GripperCommand
from math import radians, pow

# from std_msgs.msg import Empty
from std_msgs.msg import String

#*******************************************************************************
#   Try
#*******************************************************************************

#Global publisher
joint_1_pub = rospy.Publisher('/iiwa/PositionJointInterface_J1_controller/command', Float64, queue_size = 10)
joint_2_pub = rospy.Publisher('/iiwa/PositionJointInterface_J2_controller/command', Float64, queue_size = 10)
joint_3_pub = rospy.Publisher('/iiwa/PositionJointInterface_J3_controller/command', Float64, queue_size = 10)
joint_4_pub = rospy.Publisher('/iiwa/PositionJointInterface_J4_controller/command', Float64, queue_size = 10)
joint_5_pub = rospy.Publisher('/iiwa/PositionJointInterface_J5_controller/command', Float64, queue_size = 10)
joint_6_pub = rospy.Publisher('/iiwa/PositionJointInterface_J6_controller/command', Float64, queue_size = 10)
joint_7_pub = rospy.Publisher('/iiwa/PositionJointInterface_J7_controller/command', Float64, queue_size = 10)

def callback(self, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7):
    print("I am called!")



def run():
    rospy.loginfo("Start Node")
    rospy.init_node('moveIiwa')
    print("Start Node")
    #global mot_pub
    # pos_pub = rospy.Publisher('/gazebo/set_link_state', GripperCommand, queue_size = 1) # 100 to 1
    # #global mot_msg # Contain the msg
    # pos_msg = GripperCommand()
    #
    #
    # pos_msg.position = 5.0
    # pos_msg.max_effort = 1.0



    rospy.Subscriber("/iiwa/command/JointPosition", JointPosition, callback)


    #
    # pos_pub.publish(pos_msg)


    # rate.sleep()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


#*******************************************************************************
#   Main
#*******************************************************************************
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass

print("End of the python script")
