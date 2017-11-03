#!/usr/bin/env python
""" Move the robot to desired location.

Author: Nicole Maguire
Date: 9/26/2017
Version: 1

"""

from base_movement import *
from regolith_manipulation import *

class Imperio(object):

    # Initializer for the imperio node
    def __init__(self):
        rospy.init_node('imperio')
        self.robot = None
        self.run()

    # The operational loop for Imperio
    def run(self):
        # TODO : should this be for time limit or kill? Ask team
        # Using a simple loop for now
        i = 0
        while i < 3:
            self.navigateOutbound()
            self.dig()
            self.navigateInbound()
            self.deposit()
            i = i + 1
        # TODO : need to set up some sort of monitor and to manual switch

    # Navigates the robot to the area where it will dig
    def navigateOutbound(self):
        print("Imperio : Navigate Outbound")
        goal = (1, 1)
        navigate_to_goal(self.robot, goal)

    # Navigates the robot back to the collection bin
    def navigateInbound(self):
        print("Imperio : Navigate Inbound")
        goal = (-1, -1)
        navigate_to_goal(self.robot, goal)

    # Digs for regolith once the robot is in the desired location
    def dig(self):
        print("Imperio : Dig")
        dig_regolith(self.robot)

    # Deposits the regolith once the robot is at the collection bin
    def deposit(self):
        print("Imperio : Deposit")
        deposit_regolith(self.robot)


if __name__ == "__main__":
    nav = Imperio()
