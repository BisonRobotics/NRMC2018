#!/usr/bin/env python
""" Super class for a robot object, describes the methods that a Robot should have.

Author: James Madison University
Date: 9/26/2017
Version: 1

"""
from enum import Enum
import time
import tf
import rospy

class RobotState(Enum):
    OUTBOUND = 0
    DIG = 1
    INBOUND = 2
    DEPOSIT = 3
    RECOVERY = 4
    HALT = 5

class robot(object):

    def __init__(self, node):
        self.state = RobotState.OUTBOUND
        self.tf = tf.TransformListener(node)
        self.location = None
        self.pose = None

    # This should be the only method that changes the robot state
    # param: The new state the robot should be in
    def change_state(self, new_state):
        # HALT is a final/accepting state, you shouldn't be able to change it
        if self.state == RobotState.HALT:
            return
        else:
            self.state = new_state

    # How the Robot handles a low level movement command (ie Twist)
    #   param : The goal coordinates as (x,y)
    def move_to_goal(self, goal):
        time.sleep(2)

    # Localization for the robot
    # return : The current location and pose of the robot
    def localize(self):
        try:
            (self.location, self.pose) = self.tf.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return (None, None)
        return (self.location, self.pose)

    # Commands required for the robot to dig
    def dig(self):
        time.sleep(2)

    # Commands required for the robot to deposit the regolith
    def deposit(self):
        time.sleep(2)
