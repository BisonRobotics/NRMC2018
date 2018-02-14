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
    """
    Control for the entire Imperio Node
    Should always maintain the control of the robot
    """

    def __init__(self):
        """
        Initializes the imperio node
        """
        self.node = rospy.init_node('imperio')
        rospy.Subscriber('/halt_autonomy', Bool, self.haltAutonomyCallback)
        rospy.Subscriber('/times_up', Bool, self.timerCallback)

        self.robot = robot(self.node)
        self.planner = GlobalPlanner(self.robot)
        self.run()

    def haltAutonomyCallback(self, bool_msg):
        """
        Callback for the oh_shit topic, when the user wants to turn off Imperio
        No matter what the message is, if this topic is publised to, it will Halt the robot
        :param bool_msg: the passed message
        """
        self.robot.change_state(RobotState.HALT)

    def timerCallback(self, bool_msg):
        """
        Callback for the timer
        :param bool_msg: Published message
        """
        if bool_msg.data == True:
            self.robot.change_state(RobotState.HALT)

        if bool_msg.data == False:
            self.runningOutOftime()

    def run(self):
        """
        The operation loop for Imperio
        """
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

    def navigateOutbound(self):
        """
        Navigates the robot to the area where it will dig
        """
        # Goal is currently just dummy data
        goal = (3, 6)
        result = self.planner.navigate_to_goal(goal)
        if result == None:
            self.robot.change_state(RobotState.HALT)
        if result:
            self.robot.next_state()

    def navigateInbound(self):
        """
        Navigates the robot back to the collection big
        """
        goal = (0, 0)
        result =  self.planner.navigate_to_goal(goal)
        if result == None:
            self.robot.change_state(RobotState.HALT)
        if result:
            self.robot.next_state()

    def dig(self):
        """
        Digs for Regolith
        """
        if dig_regolith(self.robot):
            self.robot.next_state()

    def deposit(self):
        """
        Deposits the regolith
        """
        if deposit_regolith(self.robot):
            self.robot.next_state()

    def halt(self):
        """
        Halts the robot and Imperio
        """
        #halt_movement(self.robot)
        #halt_regolithm_commands(self.robot)
        pass

    def recover(self):
        """
        Recovery behavior for the robot
        """
        # Currently just halts the Robot
        print("Robot could not be recovered, please regain control.")
        self.robot.change_state(RobotState.HALT)

    def runningOutOfTime(self):
        """
        What the robot should do when it's running out of time
        """
        if self.robot.state == RobotState.DIG:
            self.robot.change_state(RobotState.INBOUND)

if __name__ == "__main__":
    nav = ImperioControl()