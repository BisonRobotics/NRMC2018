#!/usr/bin/env python
""" Control loop for imperio autonomy

Author: James Madison University
Date: 4/12/2018
Version: 2

"""

import rospy

from global_planner import *
from initial_planner import *
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
        rospy.loginfo("[IMPERIO] : Initializing Imperio")
        self.node = rospy.init_node('imperio')
        rospy.Subscriber('/halt_autonomy', Bool, self.haltAutonomyCallback)
        rospy.Subscriber('/times_up', Bool, self.timerCallback)

        self.robot = robot(self.node)
        self.initial_planner = InitialPlanner()
        self.planner = GlobalPlanner(self.robot)
        self.rm = RegolithManipulation()
        self.starting_region = None
        self.halt_complete = False
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
            rospy.loginfo("[IMPERIO] : Out Of Time")
            self.robot.change_state(RobotState.HALT)

        if bool_msg.data == False:
            self.turn_around()

    def run(self):
        """
        The operation loop for Imperio
        """
        rate = rospy.Rate(10)  # Refresh at 10Hz
        while not self.halt_complete and not rospy.is_shutdown():
            if self.robot.state == RobotState.INITIAL:
                self.navigateInitialPosition()
            elif self.robot.state == RobotState.OUTBOUND:
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
            rate.sleep()

    def navigateInitialPosition(self):
        result = self.initial_planner.turn_to_start()
        if result == None:
            rospy.logwarn("[IMPERIO] : ERROR! SOMETHING IS WRONG WITH INITIAL TURN!")
        if result:
            self.robot.next_state()

    def navigateOutbound(self):
        """
        Navigates the robot to the area where it will dig
        """
        # Goal is currently just dummy data
        #TODO : Find best digging goal [Jira NRMC2018-335]
        goal = (6, 0)
        result = self.planner.navigate_to_goal(goal)
        if result == None:
            rospy.logwarn("[IMPERIO] : ERROR! SOMETHING IS WRONG WITH PLANNER!")
        if result:
            self.robot.next_state()

    def navigateInbound(self):
        """
        Navigates the robot back to the collection big
        """
        goal = (.6, 0)
        result =  self.planner.navigate_to_goal(goal)
        if result == None:
            rospy.logwarn("[IMPERIO] : ERROR! SOMETHING IS WRONG WITH PLANNER!")
        if result:
            self.robot.next_state()


    def dig(self):
        """
        Digs for Regolith
        """
        result = self.rm.dig_regolith()
        if result == None:
            rospy.logwarn("[IMPERIO] : ERROR! SOMETHING IS WRONG WITH DIG!")
        if result:
            self.robot.next_state()
            self.rm.waiting_on_digging_action = False

    def deposit(self):
        """
        Deposits the regolith
        """
        result = self.rm.deposit_regolith()
        if result == None:
            rospy.logwarn("[IMPERIO] : Error with Deposit")
        if result:
            self.robot.next_state()
            self.rm.waiting_on_deposit_action = False

    def halt(self):
        """
        Halts the robot and Imperio
        """
        rospy.loginfo("[IMPERIO]: Halting all movement")
        self.planner.halt_movement()
        self.rm.halt_regolith_commands()
        self.halt_complete = True

    def recover(self):
        """
        Recovery behavior for the robot
        """
        rospy.logwarn("[IMPERIO] : CRITICAL ERROR! SOMETHING IS VERY WRONG WITH THE SYSTEM!")
        #self.robot.change_state(RobotState.HALT)

    def turn_around(self):
        """
        What the robot should do when it's running out of time
        """
        if self.robot.state == RobotState.DIG:
            rospy.loginfo("[IMPERIO] : Turn Around, changing to inbound mode from digging")
            self.robot.change_state(RobotState.INBOUND)

if __name__ == "__main__":
    nav = ImperioControl()
