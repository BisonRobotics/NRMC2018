#!/usr/bin/env python
""" Move the robot to desired location.

Author: James Madison University
Date: 9/26/2017
Version: 1

"""

import math

from robot import *
from ndsu_robot import *

# navigates the robot around obstacles to the final goal
def navigate_to_goal(robot, goal):
    while not robot_within_threshold(robot, goal):
        current_location = robot.localize()
        actualGoal = transform_coordinates(current_location, goal)
        # TODO : avoid obstacles
        robot.move_to_goal(actualGoal)


# transforms the coordinates into the frame of the robot
# returns : coordinates as a tuple (x,y)
def transform_coordinates(current_location, goal):
    # TODO : Use tf
    return goal

# Determines whether or not the robot is within the threshold to the goal
# returns : boolean
def robot_within_threshold(robot, goal):
    # TODO : get this from the launch file
    errorThreshold = .1

    goal_x = goal[0]
    goal_y = goal[1]

    location = robot.localize()

    if location == None:
        #TODO recovery behavior for localization fail, probably a limited loop
        #currently returning true to get the super fake code to run
        return True

    loc_x = location[0]
    loc_y = location[1]

    abs_distance = math.sqrt((loc_x - goal_x) ** 2 + (loc_y - goal_y) ** 2)
    return  abs_distance < errorThreshold

