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
from std_msgs.msg import Empty

class RegolithManipulation(object):
    def __init__(self):
        self.dig_client = actionlib.SimpleActionClient('dig_server', DigAction)
        self.dump_client = actionlib.SimpleActionClient('dump_server', DumpAction)

        self.regolith_in_bucket = 0

        self.waiting_on_digging_action = False
        self.waiting_on_deposit_action = False

        self.halt = False
        self.initialize_digging()


    def initialize_digging(self):
        publisher = rospy.Publisher('/init_digging', Empty, queue_size=1, latch=True)
        publisher.publish(Empty())

    # Tells the robot to dig the regolith
    # Returns boolean of success
    def dig_regolith(self):
        if self.halt:
            return None
        
        threshold = 6969
        if self.regolith_in_bucket < threshold:
            self.single_dig()
	    return False

        return True

    # This needs to be a complete action
    def single_dig(self):
        rospy.loginfo("[IMPERIO] : Performing a dig")
        goal = self.dig_goal_message()
        self.dig_client.wait_for_server()
        self.dig_client.send_goal(goal)
        self.dig_client.wait_for_result()
        result = self.dig_client.get_result()
        self.regolith_in_bucket = result.weight_harvested
        rospy.loginfo("[IMPERIO] : Harvested {} of regolith".format(self.regolith_in_bucket))

    def dig_goal_message(self):
        goal = DigActionGoal
        goal.angle = 42
        return goal

    # Tells the robot to deposit the regolith
    # Returns boolean of success
    def deposit_regolith(self):
        if self.halt:
            return None

        if self.waiting_on_deposit_action == True:
            result = self.dump_client.get_result()
            if result == None: #The action is not yet complete
                return False
            else:
                if result.is_error:
                    return None
                self.waiting_on_deposit_action = False
                self.regolith_in_bucket = result.weight_in_bucket
                rospy.loginfo("[IMPERIO] : Bucket now {} full".format(self.regolith_in_bucket))


        accepted_empty_threshold = 3 #TODO: Confirm this threshold with them [JIRA NRMC2018-358]
        if self.regolith_in_bucket > accepted_empty_threshold:
            self.single_dump()
            self.waiting_on_deposit_action = True
            return False

        return True

    def deposit_regolith_take_control(self):
        if self.halt:
            return None

        self.single_dump()
        self.dump_client.wait_for_server()
        result = self.dump_client.get_result()
        return not result.is_error

    def single_dump(self):
        rospy.loginfo("[IMPERIO] : Performing a dump")
        goal = self.dump_goal_message()
        self.dump_client.wait_for_server()
        self.dump_client.send_goal(goal)

    def dump_goal_message(self):
        goal = DumpActionGoal
        goal.angle = 42
        return goal

    # The commands to tell the robot to stop moving the hoe
    def halt_regolith_commands(self):
        rospy.loginfo("[IMPERIO] : Halting Regolith Movement")

        self.halt = True

        self.dump_client.cancel_goal()
        self.dig_client.cancel_goal()
