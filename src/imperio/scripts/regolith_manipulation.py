#!/usr/bin/env python
""" Robot commands that relate to the movement or manipulation of regolith.

Author: James Madison University
Date: 9/26/2017
Version: 1

"""

import rospy
import roslib
import actionlib

from dig_control.msg import DigAction, DigActionGoal, DigActionResult

class RegolithManipulation(object):
    def __init__(self):
        self.dig_client = actionlib.SimpleActionClient('dig_server', DigAction)
        self.regolith_in_bucket = 0
        self.waiting_on_action = False

    # Tells the robot to dig the regolith
    # Returns boolean of success
    def dig_regolith(self, robot):
        if self.waiting_on_action == True:
            result = self.dig_client.get_result()
            if result == None:
                return False
            else:
                if result.is_error:
                    return None
                self.waiting_on_action = False
                self.regolith_in_bucket = result.weight_harvested
                rospy.loginfo("[IMPERIO] : Harvested {} of regolith".format(self.regolith_in_bucket))

        #TODO : Define the needed threshold with the NDSU team [JIRA NRMC2018-358]
        threshold = 10
        if self.regolith_in_bucket < threshold:
            self.single_dig()
            self.waiting_on_action = True
            return False

        return True

    def single_dig(self):
        rospy.loginfo("[IMPERIO] : Performing a dig")
        goal = self.dig_goal_message()
        self.dig_client.wait_for_server()
        self.dig_client.send_goal(goal)

    def dig_goal_message(self):
        goal = DigActionGoal
        #TODO : fill in the angle [JIRA NRMC2018-358]
        goal.angle = 10.0
        return goal

    # Tells the robot to deposit the regolith
    # Returns boolean of success
    def deposit_regolith(self, robot):
        #deposit regolith
        return True

    # The commands to tell the robot to stop moving the hoe
    def halt_regolithm_commands(self, robot):
        pass
