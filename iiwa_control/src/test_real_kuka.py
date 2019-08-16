#!/usr/bin/env python
'''
Test to go from current cartesian pose to another one
'''
import rospy

from iiwa_msgs.msg import CartesianPose
from geometry_msgs.msg import PoseStamped
# from iiwa_msgs.msg import CartesianPoseLin
import tf


def talker():
    CartesianPose_bool = False
    if CartesianPose_bool:
        pub = rospy.Publisher('/iiwa/command/CartesianPose', CartesianPose, queue_size=10)
    else:
        # pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=1)
        pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=1)
    # pub = rospy.Publisher('/iiwa/command/CartesianPoseLin', CartesianPose, queue_size=10)
    rospy.init_node('test_pub', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if CartesianPose_bool:
            goal = CartesianPose()
            # goal.header.seq

            # goal = PoseStamped()
            goal.poseStamped
            goal.poseStamped.header.seq = 1
            goal.poseStamped.header.stamp = rospy.Time.now()
            goal.poseStamped.header.frame_id = "world"

            goal.poseStamped.pose.position.x = 0.46875
            goal.poseStamped.pose.position.y = 0.0
            goal.poseStamped.pose.position.z = 0.97862

            quaternion = tf.transformations.quaternion_from_euler(0.0, 1.099, 0.0)#1.5707963

            goal.poseStamped.pose.orientation.x = quaternion[0]
            goal.poseStamped.pose.orientation.y = quaternion[1]
            goal.poseStamped.pose.orientation.z = quaternion[2]
            goal.poseStamped.pose.orientation.w = quaternion[3]

            # goal.redundancy.e1 = 0
            goal.redundancy.status = -1
            goal.redundancy.turn = -1

            rospy.loginfo(goal)
            pub.publish(goal)
            rate.sleep()
        else:
            goal = PoseStamped()

            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "world"

            goal.pose.position.x = 0.4
            goal.pose.position.y = -0.4
            goal.pose.position.z = 0.3

            quaternion = tf.transformations.quaternion_from_euler(3.14, 0.0, -3.14)#1.5707963

            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.z = quaternion[2]
            goal.pose.orientation.w = quaternion[3]

            rospy.loginfo(goal)
            pub.publish(goal)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

        # goal = PoseStamped()

        # goal.header.seq = 1
        # goal.header.stamp = rospy.Time.now()
        # goal.header.frame_id = "world"

        # goal.pose.position.x = 0.52
        # goal.pose.position.y = 0.0
        # goal.pose.position.z = 0.74

        # quaternion = tf.transformations.quaternion_from_euler(0.0, 1.5707963, 0.0)

        # goal.pose.orientation.x = quaternion[0]
        # goal.pose.orientation.y = quaternion[1]
        # goal.pose.orientation.z = quaternion[2]
        # goal.pose.orientation.w = quaternion[3]

    #     rospy.loginfo(goal)
    #     pub.publish(goal)
    #     rate.sleep()