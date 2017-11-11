#!/usr/bin/env python
""" Move the robot to desired location.

Author: James Madison University
Date: 11/11/2017
Version: 1

"""

import math
import rospy

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
    errorThreshold = rospy.get_param('/location_accuracy')

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

