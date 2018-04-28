#!/usr/bin/env python
""" Super class for a robot object and state machine, describes the methods that a Robot should have.

Author: Nicole Maguire
Date: 4/12/2018
Version: 3

"""
from enum import Enum
import tf
import rospy

class RobotState(Enum):
    """
    RobotState represents the states that the robot can be in
    """
    OUTBOUND = 0
    DIG = 1
    INBOUND = 2
    DEPOSIT = 3
    RECOVERY = 4
    HALT = 5
    INITIAL = 6

class robot(object):
    """
    The robot class represents the robot being used
    """

    def __init__(self, node):
        """
        Initializes the robot
        :param node: the ROS node being used
        """
        self.state = None
        self.change_state(RobotState.INITIAL)
        self.tf = tf.TransformListener(node)
        self.location = None
        self.pose = None

    def change_state(self, new_state):
        """
        Change state, this should be the only method that has access to self.state
        :param new_state: the new state being requested for the robot
        """
        # HALT is a final/accepting state, you shouldn't be able to change it
        if self.state == RobotState.HALT:
            return
        else:
            self.state = new_state
            self.print_state(self.state)

    def print_state(self, state):
        """
        Prints the state of the Robot
        :param state: the state to be printed
        """
        if state == RobotState.OUTBOUND:
            rospy.loginfo("[IMPERIO] : OUTBOUND")
        elif state == RobotState.DEPOSIT:
            rospy.loginfo("[IMPERIO] : DEPOSIT")
        elif state == RobotState.INBOUND:
            rospy.loginfo("[IMPERIO] : INBOUND")
        elif state == RobotState.DIG:
            rospy.loginfo("[IMPERIO] : DIG")
        elif state == RobotState.RECOVERY:
            rospy.loginfo("[IMPERIO] : RECOVERY BEHAVIOR")
        elif state == RobotState.HALT:
            rospy.loginfo("[IMPERIO] : HALT")
        elif state == RobotState.INITIAL:
            rospy.loginfo("[IMPERIO] : Initial Orientation")

    def next_state(self):
        """
        Ease Function to change the state to the next expected state
        """
        if self.state == RobotState.INITIAL:
            self.change_state(RobotState.OUTBOUND)
        elif self.state == RobotState.OUTBOUND:
            self.change_state(RobotState.DIG)
        elif self.state == RobotState.DIG:
            self.change_state(RobotState.INBOUND)
        elif self.state == RobotState.INBOUND:
            self.change_state(RobotState.DEPOSIT)
        elif self.state == RobotState.DEPOSIT:
            self.change_state(RobotState.OUTBOUND)

    def localize(self):
        """
        Localization for the robot
        :return: robot location (x,y) and pose (x,y,theta)
        """
        while not rospy.is_shutdown():
            try:
                (self.location, self.pose) = self.tf.lookupTransform('/map', '/base_link', rospy.Time(0))
                rospy.loginfo("[IMPERIO] : Robot localized to location : {} and pose : {}".format(self.location, self.pose))
                return self.location, self.pose
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("[IMPERIO] : Robot is not able to localize")
                # TODO : Add recovery behavior [Jira NRMC2018-329]
                rospy.sleep(1.0)
