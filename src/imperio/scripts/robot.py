#!/usr/bin/env python
""" Super class for a robot object, describes the methods that a Robot should have.

Author: James Madison University
Date: 9/26/2017
Version: 1

"""

class robot(object):

    # How the Robot handles a low level movement command (ie Twist)
    #   param : The goal coordinates as (x,y)
    def move_to_goal(self, goal):
        pass

    # Localization for the robot
    # return : The current coordinates of the robot
    def localize(self):
        pass

    # Commands required for the robot to dig
    def dig(self):
        pass

    # Commands required for the robot to deposit the regolith
    def deposit(self):
        pass

    # The name of the robot
    # return : A string Value of the name
    def to_string(self):
        return "Default Robot Superclass"