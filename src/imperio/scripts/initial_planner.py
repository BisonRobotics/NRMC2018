#!/usr/bin/env python
"""
Initial Planner Class - Gets the robot out of the initial orientation next to the occupancy grid

Author: Nicole Maguire - James Madison University
Date: 4/2/2018
Version: 1
"""


import rospy
from imperio.msg import DriveStatus

from std_msgs.msg import Float64, Empty


class InitialPlanner(object):
    def __init__(self):
        self.south_check_publisher = rospy.Publisher('/position_controller/south_check', Empty, queue_size=1, latch=True)
        self.theta_publisher = rospy.Publisher('/position_controller/initialTheta', Float64, queue_size=1, latch=True)
        rospy.Subscriber('/position_controller/drive_controller_status', DriveStatus, self.drive_status_callback)
        self.has_turned = False
        self.planner_failed = False
        self.msg_published = False

    def drive_status_callback(self, status_message):
        if status_message.has_reached_goal.data:
            self.has_turned = True
        if status_message.is_stuck.data:
            self.planner_failed = True
        if status_message.cannot_plan_path.data:
            self.planner_failed = True

    def turn_to_start(self):
        if self.planner_failed:
            return None
        if self.has_turned:
            return True
        if self.msg_published:
            return False

        rospy.loginfo("[IMPERIO] : Turning the robot in place")
        self.publish_south_check()
        self.publish_turn_msg(0)
        return False

    def publish_turn_msg(self, theta):
        msg = Float64()
        msg.data = theta
        self.msg_published = True
        self.theta_publisher.publish(msg)

    def publish_south_check(self):
        msg = Empty()
        self.south_check_publisher.publish(msg)






         