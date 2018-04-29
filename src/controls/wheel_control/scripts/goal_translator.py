#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose, Pose2D


def pose_callback(msg):
    """

    :type msg: PoseStamped
    """
    global pose_updated, pose
    pose_updated = True
    pose = msg.pose
    rospy.loginfo ("got  a message")

if __name__ == '__main__':
    global pose_updated, pose
    pose_updated = False
    pose = Pose

    rospy.init_node('goal_translator')
    pub = rospy.Publisher ('/position_controller/additional_waypoint', Pose2D, queue_size=10)
    rospy.Subscriber('move_base_simple/goal', PoseStamped, pose_callback)
    pose_2 = Pose2D()
    rate = rospy.Rate(10)  # Update at 10Hz
    while not rospy.is_shutdown():
        if pose_updated:
            orientation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            pose_2.x = pose.position.x
            pose_2.y = pose.position.y
            pose_2.theta = tf.transformations.euler_from_quaternion(orientation)[2] # roll pitch yaw
            pub.publish(pose_2)
            pose_updated=False
        rate.sleep()
