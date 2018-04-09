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

    # Tells the robot to dig the regolith
    # Returns boolean of success
    def dig_regolith(self, robot):
        print("enter dig regolith")
        goal = self.dig_goal_message()
        print("dig message created")
        self.dig_client.wait_for_server()
        print("finished waiting for the serer")
        self.ac.send_goal(goal)
        print("goal sent")
        #TODO : Check how long we should do this
        result = self.dig_client.get_goal_status_text()
        print(result)
        #TODO : Wait for the result here?
        return True

    def dig_goal_message(self):
        goal = DigActionGoal
        #TODO : fill in the angle
        goal.angle = 0.0
        return goal

    # Tells the robot to deposit the regolith
    # Returns boolean of success
    def deposit_regolith(self, robot):
        #deposit regolith
        return True

    # The commands to tell the robot to stop moving the hoe
    def halt_regolithm_commands(self, robot):
        pass
