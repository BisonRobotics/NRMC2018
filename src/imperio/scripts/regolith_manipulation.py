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
from dig_control.msg import OutriggerAction, OutriggerActionGoal, OutriggerActionResult


class RegolithManipulation(object):
    def __init__(self):
        self.dig_client = actionlib.SimpleActionClient('dig_server', DigAction)
        self.dump_client = actionlib.SimpleActionClient('dump_server', DumpAction)
        self.deploy_client = actionlib.SimpleActionClient('/outrigger_master/deploy_riggers', OutriggerAction)
        self.retract_client = actionlib.SimpleActionClient('/outrigger_master/retract_riggers', OutriggerAction)

        self.outriggers_deployed = False
        self.regolith_in_bucket = 0

        self.waiting_on_action = False
        self.waiting_on_retract_action = False
        self.waiting_on_deploy_action = False

        self.halt = False

    # Tells the robot to dig the regolith
    # Returns boolean of success
    def dig_regolith(self):
        if self.halt:
            return None

        if self.waiting_on_action == True:
            result = self.dig_client.get_result()
            if result == None: #The action is not yet
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
        goal.angle = 42
        return goal

    def deploy_outriggers(self):
        if self.halt:
            return None

        if self.waiting_on_deploy_action == True:
            result = self.deploy_client.get_result()
            if result == None:
                return False
            else:
                self.waiting_on_deploy_action = False
                rospy.loginfo("[IMPERIO] : Outriggers are now deployed")
                self.outriggers_deployed = True
                return True

        goal = self.outrigger_goal_message()
        rospy.loginfo("[IMPERIO] : Waiting for the deploy_client")
        self.deploy_client.wait_for_server()
        rospy.loginfo("[Imperio] : Deploying the outriggers")
        self.deploy_client.send_goal(goal)
        self.waiting_on_deploy_action = True
        return False


    def retract_outriggers(self):
        if self.halt:
            return None

        if self.waiting_on_retract_action == True:
            result = self.retract_client.get_result()
            if result == None:
                return False
            else:
                rospy.loginfo("[IMPERIO] : Outriggers are now retracted")
                self.waiting_on_retract_action = False
                self.outriggers_deployed = False
                return True

        goal = self.outrigger_goal_message()
        rospy.loginfo("[IMPERIO] : Waiting for the retract_client")
        self.retract_client.wait_for_server()
        rospy.loginfo('[IMPERIO] : Retracting the outriggers')
        self.retract_client.send_goal(goal)
        self.waiting_on_retract_action = True
        return False

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


        accepted_empty_threshold = 3 #TODO: Confirm this threshold with them [JIRA NRMC2018-358]
        if self.regolith_in_bucket > accepted_empty_threshold:
            self.single_dump()
            self.waiting_on_action = True
            return False

        return True

    def outrigger_goal_message(self):
        goal = OutriggerActionGoal
        goal.deploy = 69
        return goal

    def single_dump(self):
        print("Imperio : Performing a dump")
        goal = self.dump_goal_message()
        self.dump_client.wait_for_server()
        self.dump_client.send_goal(goal)

    def dump_goal_message(self):
        goal = DumpActionGoal
        goal.angle = 42
        return goal

    # The commands to tell the robot to stop moving the hoe
    def halt_regolith_commands(self):
        self.halt = True

        self.retract_client.cancel_goal()
        self.deploy_client.cancel_goal()
        self.dump_client.cancel_goal()
        self.dig_client.cancel_goal()
