#!/usr/bin/env python
""" Robot commands that relate to the movement or manipulation of regolith.

Author: James Madison University
Date: 9/26/2017
Version: 1

"""

import rospy
import roslib
import actionlib

from dig_control.msg import DigAction, DigActionGoal

class RegolithManipulation(object):
    def __init__(self):
        self.dig_client = actionlib.SimpleActionClient('dig_server', DigAction)
        #self.dig_client.wait_for_server()

    # Tells the robot to dig the regolith
    # Returns boolean of success
    def dig_regolith(self, robot):
        goal = DigActionGoal()
        #TODO : fill in the goal here
        self.dig_client.send_goal(goal)
        #TODO : Wait for the result here?
        return True

    # Tells the robot to deposit the regolith
    # Returns boolean of success
    def deposit_regolith(self, robot):
        #deposit regolith
        return True

    # The commands to tell the robot to stop moving the hoe
    def halt_regolithm_commands(self, robot):
        pass
