#!/usr/bin/env python
"""
Move the robot to desired location.

Author: James Madison University
Date: 11/26/2017
Version: 2
"""

import math
import rospy
import astar as path_finder
import map_utils

from robot import *

# TODO : Currently an issue trying to import a custom message, no module named .msg
from imperio.msg import GlobalWaypoints
from imperio.msg import DriveStatus
from nav_msgs.msg import OccupancyGrid

class MovementStatus(Enum):
    """
    Movement Status is the enum of the different states that the robot can be in for movement.
    """
    MOVING = 0
    HAS_REACHED_GOAL = 1
    STUCK = 2
    CANNOT_PLAN_PATH = 3


class GlobalPlanner(object):
    """
    Global Planner takes a goal point in relation to the global map and outputs waypoints to the local planner.
    """

    def __init__(self, robot):
        """
        Initializes the global planner
        :param robot: the robot object the planner will be moving
        """
        self.waypoints_publisher = rospy.Publisher('/global_planner_goal', GlobalWaypoints, queue_size=1)
        rospy.Subscriber('/drive_controller_status', DriveStatus, self.drive_status_callback)
        rospy.Subscriber('/vrep/map', OccupancyGrid, self.map_callback)
        self.robot = robot
        self.occupancy_grid = None
        self.movement_status = MovementStatus.HAS_REACHED_GOAL

    def map_callback(self, map_message):
        args = [map_message]
        #self.occupancy_grid = map_utils.Map(map_message, None)

    def drive_status_callback(self, status_message):
        """
        Callback for when the drive status is published
        :param status_message: the message that was published
        """
        print("Goly Gee Yeah")
        if status_message.has_reached_goal:
            self.movement_status = MovementStatus.HAS_REACHED_GOAL
        if status_message.is_stuck:
            self.movement_status = MovementStatus.STUCK
        if status_message.cannot_plan_path:
            self.movement_status = CANNOT_PLAN_PATH

    def navigate_to_goal(self, goal):
        """
        Attempts to navigate to the final goal
        :param goal: the global goal (based on the overall map) as (x,y)
        :return: a boolean of it the robot has reached the goal, None for a fatal error
        """
        if self.movement_status == MovementStatus.CANNOT_PLAN_PATH:
            return None
        if self.movement_status == MovementStatus.MOVING:
            return False
        if self.movement_status == MovementStatus.HAS_REACHED_GOAL and self.robot_within_threshold(goal):
            return True

        #waypoints = self.find_waypoints(goal)
        # currently debugging the A* implementation.
        waypoints = [[0,1], [1,0]]
        oriented_waypoints = self.calculate_orientation(waypoints)

        self.publish_waypoints(oriented_waypoints)
        return False

    def find_waypoints(self, goal):
        """
        Finds the waypoints in the occupancy grid, navigation around obstacles
        :param goal: the final goal as (x,y)
        :return: an array of waypoints
        """
        #while self.occupancy_grid == None:
         #   print("Occupancy grid has not been initialized yet")
        (location, pose) = self.robot.localize()
        if location == None:
            location = [0,0]
        results = path_finder.aStar(location, goal, self.occupancy_grid)
        print("Path finder has returned")
        print(results)
        #path_finder.bullshit_it(location, goal, self.occupancy_grid)


    def publish_waypoints(self, waypoints):
        """
        Publishes the waypoints to the local planner
        :param waypoints: an array of warpoints
        """
        message = GlobalWaypoints()
        message.poses = waypoints
        # message.occupancyGrid = self.occupancy_grid.to_message()
        self.waypoints_publisher.publish(message)
        self.movement_status = MovementStatus.MOVING

    def robot_within_threshold(self, goal):
        """
        Determines if the robot is within a threshold specified in the imperio launch file
        :param goal: the final goal as (x,y)
        :return: a boolean of if the robot is within the threshold
        """
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

    def halt_movement(self):
        """
        Halts the command of the robot
        """
        #will need to empty the list of poses/goals to the local planner
        #will need to tell the planner to stop moving
        pass

    def calculate_orientation(self, waypoints):
        """
        Calcularets the orientation for each waypoint, will just point straight to the next waypoint
        :param waypoints: array of waypoints
        :return: array of waypoints with orientation
        """
        oriented_waypoints = []

        for i in range(1, len(waypoints)):
            waypoint1 = waypoints[i -1]
            waypoint2 = waypoints[i]
            x1 = waypoint1[0]
            y1 = waypoint1[1]
            x2 = waypoint2[0]
            y2 = waypoint2[1]

            orientation = 0
            #moving to the right
            if x2 > x1:
                orientation = 90

            #moving the the left
            if x1 > x2:
                orientation = 270

            #moving up the map
            if y2 > y1:
                orientation = 0

            #moving down the map
            if y1 > y2:
                orientation = 180

            single = [x1, y1, orientation]
            oriented_waypoints.append(single)

        #still need to add the last waypoint
        final_waypoint = waypoints[len(waypoints) - 1]
        final_orientation = 180
        single = [final_waypoint[0], final_waypoint[1], final_orientation]
        oriented_waypoints.append(single)

        return oriented_waypoints




