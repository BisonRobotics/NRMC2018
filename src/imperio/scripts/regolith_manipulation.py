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
from dig_control.msg import DumpAction, DumpActionGoal, DumpActionResult


class RegolithManipulation(object):
    def __init__(self):
        self.dig_client = actionlib.SimpleActionClient('dig_server', DigAction)
        self.dump_client = actionlib.SimpleActionClient('dump_server', DumpAction)
        self.regolith_in_bucket = 0
        self.waiting_on_action = False
        self.halt = False

    # Tells the robot to dig the regolith
    # Returns boolean of success
    def dig_regolith(self):
        if self.halt:
            return None

        if self.waiting_on_action == True:
            result = self.dig_client.get_result()
            if result == None: #The action is not yet complete
                return False
            else:
                if result.is_error:
                    return None
                self.waiting_on_action = False
                self.regolith_in_bucket = result.weight_harvested
                print("Imperio : Harvested {} of regolith".format(self.regolith_in_bucket))

        #TODO : Define the needed threshold with the NDSU team [JIRA NRMC2018-358]
        threshold = 10
        if self.regolith_in_bucket < threshold:
            self.single_dig()
            self.waiting_on_action = True
            return False

        return True

    def single_dig(self):
        print("Imperio : Performing a dig")
        goal = self.dig_goal_message()
        self.dig_client.wait_for_server()
        self.dig_client.send_goal(goal)

    def dig_goal_message(self):
        goal = DigActionGoal()
        goal.goal = 42
        return goal

    # Tells the robot to deposit the regolith
    # Returns boolean of success
    def deposit_regolith(self):
        if self.halt:
            return None

        if self.waiting_on_action == True:
            result = self.dump_client.get_result()
            if result == None: #The action is not yet complete
                return False
            else:
                if result.is_error:
                    return None
                self.waiting_on_action = False
                self.regolith_in_bucket = result.weight_in_bucket
                print("Imperio : Bucket now {} full".format(self.regolith_in_bucket))


        accepted_empty_threshold = 0 #TODO: Confirm this threshold with them [JIRA NRMC2018-358]
        if self.regolith_in_bucket > accepted_empty_threshold:
            self.single_dig()
            self.waiting_on_action = True
            return False

        return True

    def single_dump(self):
        print("Imperio : Performing a dump")
        goal = self.dump_goal_message()
        self.dig_client.wait_for_server()
        self.dig_client.send_goal(goal)

    def dump_goal_message(self):
        goal = DumpActionGoal
        return goal

    # The commands to tell the robot to stop moving the hoe
    def halt_regolith_commands(self):
        self.halt = True
