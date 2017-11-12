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

class Imperio(object):

    # Initializer for the imperio node
    def __init__(self):
        self.node = rospy.init_node('imperio')
        rospy.Subscriber('/oh_shit', Bool, self.ohShitCallback)
        rospy.Subscriber('/times_up', Bool, self.timerCallback)

        self.robot = robot(self.node)
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
        print("Imperio : Outbound")
        goal = (5, 8)
        if navigate_to_goal(self.robot, goal):
            self.robot.change_state(RobotState.DIG)

    # Navigates the robot back to the collection bin
    def navigateInbound(self):
        print("Imperio : Inbound")
        goal = (0, 0)
        if navigate_to_goal(self.robot, goal):
            self.robot.change_state(RobotState.DEPOSIT)

    # Digs for regolith once the robot is in the desired location
    def dig(self):
        print("Imperio : Digging")
        if dig_regolith(self.robot):
            self.robot.change_state(RobotState.INBOUND)

    # Deposits the regolith once the robot is at the collection bin
    def deposit(self):
        print("Imperio : Depositing")
        if deposit_regolith(self.robot):
            self.robot.change_state(RobotState.OUTBOUND)

    def halt(self):
        print("Imperio : Halting")
        halt_movement(self.robot)
        halt_regolithm_commands(self.robot)

    def recover(self):
        print("Imperio : Entering Recovery Behavior")
        self.robot.change_state(RobotState.HALT)


if __name__ == "__main__":
    nav = Imperio()
