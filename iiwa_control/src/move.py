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
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from control_msgs.msg import GripperCommand
from math import radians, pow

# from std_msgs.msg import Empty
from std_msgs.msg import String

#*******************************************************************************
#   Try
#*******************************************************************************


def run():
    rospy.loginfo("Start Node")
    rospy.init_node('moveIiwa')
    print("Start Node")
    #global mot_pub
    pos_pub = rospy.Publisher('/gazebo/set_link_state', GripperCommand, queue_size = 1) # 100 to 1
    #global mot_msg # Contain the msg
    pos_msg = GripperCommand()


    pos_msg.position = 5.0
    pos_msg.max_effort = 1.0







    pos_pub.publish(pos_msg)


    #rate.sleep()
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


#*******************************************************************************
#   Main
#*******************************************************************************
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass

print("End of the python script")
