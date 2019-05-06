#bim

#! /usr/bin/env python3
#! /home/roboticlab14/.local/lib/python3.4/site-packages python3

# IIWA module
# Jonathan Burkhard,
# Documentations : Not exist yet

# Basic imports
import sys
import rospy


import tensorflow as ab


#*******************************************************************************
#   Try
#*******************************************************************************





def run():
    # rospy.loginfo("Start Node")
    # rospy.init_node('moveIiwa')
    print("Start Node")
    #global mot_pub
    # pos_pub = rospy.Publisher('/gazebo/set_link_state', GripperCommand, queue_size = 1) # 100 to 1
    # #global mot_msg # Contain the msg
    # pos_msg = GripperCommand()
    #
    #
    # pos_msg.position = 5.0
    # pos_msg.max_effort = 1.0

    print(sys.version)

    

    #
    # pos_pub.publish(pos_msg)


    # rate.sleep()
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


#*******************************************************************************
#   Main
#*******************************************************************************
if __name__ == '__main__':
   
    run()

    print("End of the python script")
