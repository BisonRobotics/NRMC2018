#!/usr/bin/env python
""" Robot commands that relate to the movement or manipulation of regolith.

Author: James Madison University
Date: 9/26/2017
Version: 1

"""

from robot import *

# Tells the robot to dig the regolith
# Returns boolean of success
def dig_regolith(robot):
    robot.dig()
    return True

# Tells the robot to deposit the regolith
# Returns boolean of success
def deposit_regolith(robot):
    robot.deposit()
    return True

# The commands to tell the robot to stop moving the hoe
def halt_regolithm_commands(robot):
    pass
