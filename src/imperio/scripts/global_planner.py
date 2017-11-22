#!/usr/bin/env python
""" Move the robot to desired location.

Author: James Madison University
Date: 11/11/2017
Version: 1

"""

import math
import rospy

from robot import *

# TODO : Currently an issue trying to import a custom message, no module named .msg
#from imperio.msg import GlobalWaypoints
#from imperio.msm import DriveStatus

class MovementStatus(Enum):
    MOVING = 0
    HAS_REACHED_GOAL = 1
    STUCK = 2
    CANNOT_PLAN_PATH = 3


class GlobalPlanner(object):

    def __init__(self, robot):
        #self.waypoints_publisher = rospy.Publisher('/global_planner_goal', GlobalWaypoints, queue_size=1)
        #rospy.Subscriber('/drive_controller_status', DriveStatus, drive_status_callback)
        self.robot = robot
        self.occupancy_grid = None
        self.movement_status = MovementStatus.HAS_REACHED_GOAL

    def drive_status_callback(self, status_message):
        if status_message.has_reached_goal:
            self.movement_status = MovementStatus.HAS_REACHED_GOAL
        if status_message.is_stuck:
            self.movement_status = MovementStatus.STUCK
        if status_message.cannot_plan_path:
            self.movement_status = CANNOT_PLAN_PATH


    # attempts to navigate to the final goal
    # param : the robot : the robot object
    # param : the global goal
    # returns : if robot is within goal threshold, returns none for fatal error
    def navigate_to_goal(self, goal):
        if self.movement_status == MovementStatus.CANNOT_PLAN_PATH:
            return None
        if self.movement_status == MovementStatus.MOVING:
            return False
        if self.movement_status == MovementStatus.HAS_REACHED_GOAL and self.robot_within_threshold(goal):
            return True

        waypoints = self.find_waypoints(goal)
        self.publish_waypoints(waypoints)
        return False

    # gets the waypoints for navigating around obstacles to goal
    # returns : an array of Waypoints
    def find_waypoints(self, goal):
        #TODO : use A* to find the waypoints on the occupancy grid
        pass

    # publishes the waypoints to the local planner through message passing
    def publish_waypoints(self, waypoints):
        # message = GlobalWaypoints()
        # message.poses = waypoints
        # message.occupancyGrid = self.occupancy_grid.to_message()
        # self.waypoints_publisher.publish(message)
        self.movement_status = MovementStatus.MOVING

    # Determines whether or not the robot is within the threshold to the goal
    # param : The robot
    # param : The global goal
    # returns : boolean
    def robot_within_threshold(self, goal):
        errorThreshold = rospy.get_param('/location_accuracy')

        goal_x = goal[0]
        goal_y = goal[1]

        (location, pose) = self.robot.localize()

        if location == None:
            print("Imperio: Unable to localize the robot")
            #TODO recovery behavior for localization fail
            return False

        loc_x = location[0]
        loc_y = location[1]

        # TODO : Check the orientation of the robot
        abs_distance = math.sqrt((loc_x - goal_x) ** 2 + (loc_y - goal_y) ** 2)
        return  abs_distance < errorThreshold

    # Halts the commands for the robot
    def halt_movement(self, robot):
        #will need to empty the list of poses/goals to the local planner
        #will need to tell the planner to stop moving
        pass
