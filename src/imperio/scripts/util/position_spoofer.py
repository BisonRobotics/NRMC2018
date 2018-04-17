#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose


def pose_callback(msg, br):
    """

    :type msg: PoseWithCovarianceStamped
    """
    global pose_updated, pose
    pose_updated = True
    pose = msg.pose.pose


if __name__ == '__main__':
    global pose_updated, pose
    pose_updated = False
    pose = Pose

    rospy.init_node('position_spoofer')
    br = tf.TransformBroadcaster()
    rospy.Subscriber('initialpose', PoseWithCovarianceStamped, pose_callback, br)

    rate = rospy.Rate(10)  # Update at 10Hz
    while not rospy.is_shutdown():
        if pose_updated:
            position = (pose.position.x, pose.position.y, pose.position.z)
            orientation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            br.sendTransform(position, orientation, rospy.Time.now(), "base_link", "map")
        rate.sleep()
