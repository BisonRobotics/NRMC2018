#!/usr/bin/env python
""" Move the robot to desired location.

Author: James Madison University
Date: 11/11/2017
Version: 2

"""

import rospy

from global_planner import *
from regolith_manipulation import *
from std_msgs.msg import Bool

class ImperioControl(object):

    # Initializer for the imperio node
    def __init__(self):
        self.node = rospy.init_node('imperio')
        rospy.Subscriber('/oh_shit', Bool, self.ohShitCallback)
        rospy.Subscriber('/times_up', Bool, self.timerCallback)

        self.robot = robot(self.node)
        self.planner = GlobalPlanner(self.robot)
        self.run()

    def ohShitCallback(self, bool_msg):
        self.robot.change_state(RobotState.HALT)

    def timerCallback(self, bool_msg):
        if bool_msg.data == True:
            self.robot.change_state(RobotState.HALT)

    # The operational loop for Imperio
    def run(self):

        while not self.robot.state == RobotState.HALT and not rospy.is_shutdown():
            if self.robot.state == RobotState.OUTBOUND:
                self.navigateOutbound()
            elif self.robot.state == RobotState.DIG:
                self.dig()
            elif self.robot.state == RobotState.INBOUND:
                self.navigateInbound()
            elif self.robot.state == RobotState.DEPOSIT:
                self.deposit()
            elif self.robot.state == RobotState.RECOVERY:
                self.recover()
            else:
                self.halt()

    # Navigates the robot to the area where it will dig
    def navigateOutbound(self):
        goal = (5, 8)
        result = self.planner.navigate_to_goal(goal)
        if result == None:
            self.robot.change_state(RobotState.HALT)
        if result:
            self.robot.next_state()

    # Navigates the robot back to the collection bin
    def navigateInbound(self):
        goal = (0, 0)
        result =  self.planner.navigate_to_goal(goal)
        if result == None:
            self.robot.change_state(RobotState.HALT)
        if result:
            self.robot.next_state()

    # Digs for regolith once the robot is in the desired location
    def dig(self):
        if dig_regolith(self.robot):
            self.robot.next_state()

    # Deposits the regolith once the robot is at the collection bin
    def deposit(self):
        if deposit_regolith(self.robot):
            self.robot.next_state()

    def halt(self):
        halt_movement(self.robot)
        halt_regolithm_commands(self.robot)

    def recover(self):
        self.robot.change_state(RobotState.HALT)


if __name__ == "__main__":
    nav = ImperioControl()
