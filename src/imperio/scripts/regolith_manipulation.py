#!/usr/bin/env python
""" Robot commands that relate to the movement or manipulation of regolith.

Author: James Madison University
Date: 9/26/2017
Version: 1

"""

from robot import *


def dig_regolith(robot):
    robot.dig()

def deposit_regolith(robot):
    robot.deposit()
