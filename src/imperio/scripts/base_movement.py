#!/usr/bin/env python
""" Move the robot to desired location.

Author: James Madison University
Date: 11/11/2017
Version: 1

"""

import math
import rospy

from robot import *

# attempts to navigate to the final goal
# param : the robot : the robot object
# param : the global goal
# returns : if robot is within goal threshold
def navigate_to_goal(robot, goal):
    # calculate A* on an occupancy grid
    # output those goals to the local planner
    # wait for the local planner to finish
    return robot_within_threshold(robot, goal)

# Determines whether or not the robot is within the threshold to the goal
# param : The robot
# param : The global goal
# returns : boolean
def robot_within_threshold(robot, goal):
    errorThreshold = rospy.get_param('/location_accuracy')

    goal_x = goal[0]
    goal_y = goal[1]

    (location, pose) = robot.localize()

    if location == None:
        print("Imperio: Unable to localize the robot")
        #TODO recovery behavior for localization fail, probably a limited loop
        #currently returning true to get the super fake code to run
        return True

    #TODO : check object type of localize return
    loc_x = 0
    loc_y = 0

    # TODO : Check the orientation of the robot
    abs_distance = math.sqrt((loc_x - goal_x) ** 2 + (loc_y - goal_y) ** 2)
    return  abs_distance < errorThreshold

# Halts the commands for the robot
def halt_movement(robot):
    #will need to empty the list of poses/goals to the local planner
    #will need to tell the planner to stop moving
    pass
