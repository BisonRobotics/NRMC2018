#!/usr/bin/env python
"""
Move the robot to desired location.

Author: Nicole Maguire - James Madison University
Date: 2/11/2018
Version: 3
"""

import math
import rospy
import map_utils
import time
import RRT
from robot import *

from imperio.msg import GlobalWaypoints
from imperio.msg import DriveStatus
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D

halt_for_visualization = False

class MovementStatus(Enum):
    """
    Movement Status is the enum of the different states that the robot can be in for movement.
    """
    MOVING = 0
    HAS_REACHED_GOAL = 1
    STUCK = 2
    CANNOT_PLAN_PATH = 3
    WAITING = 4


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
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.robot = robot
        self.occupancy_grid = None
        self.movement_status = MovementStatus.HAS_REACHED_GOAL

    def map_callback(self, map_message):
        self.occupancy_grid = map_utils.Map(map_message)

    def drive_status_callback(self, status_message):
        """
        Callback for when the drive status is published
        :param status_message: the message that was published
        """
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

        #We can't do anything until we have the occupancy grid
        if self.occupancy_grid == None:
            print("IMPERIO: Cannot find the occupancy grid")
            self.movement_status == MovementStatus.WAITING
            return False
        print("Imperio: Occupancy Grid Exists")

        waypoints = self.find_waypoints(goal)
        oriented_waypoints = self.calculate_orientation(waypoints)
        print("Imperio: Path found : {}".format(oriented_waypoints))

        self.publish_waypoints(oriented_waypoints)
        return False

    def find_waypoints(self, goal):
        """
        Finds the waypoints in the occupancy grid, navigation around obstacles
        :param goal: the final goal as (x,y)
        :return: an array of waypoints
        """
        (location, pose) = self.robot.localize()

        print("Starting the path planner")
        saved_time = time.time()
        results = RRT.find_best_rrt_path(location, goal, self.occupancy_grid, 20)
        print("Path Planning Complete. Total path planning time: {} seconds".format(time.time() - saved_time))
        print("Unoriented path found: {}".format(results))

        if len(results) == 0:
            #TODO : Enter recovery behavior
            print("IMPERIO : NO POSSIBLE PATH FOUND")

        return results

    def publish_waypoints(self, waypoints):
        """
        Publishes the waypoints to the local planner
        :param waypoints: an array of oriented waypoints
        """
        message = GlobalWaypoints()
        pose_array = []

        for point in waypoints:
            msg = Pose2D()
            msg.x, msg.y, msg.theta = point
            pose_array.append(msg)

        message.pose_array = pose_array
        self.waypoints_publisher.publish(message)
        self.movement_status = MovementStatus.MOVING
        print("Imperio: Waypoints published to local planner")


    def robot_within_threshold(self, goal):
        """
        Determines if the robot is within a threshold specified in the imperio launch file
        :param goal: the final goal as (x,y)
        :return: a boolean of if the robot is within the threshold
        """
        errorThreshold = rospy.get_param('/location_accuracy')

        goal_x, goal_y = goal
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
        # Check if none or one waypoint
        if (len(waypoints) == 0):
            return []

        if (len(waypoints) == 1):
            #TODO : final orientation will be passed as param from control (more logistics/strategy/testing needed)
            final_orientation = math.degrees(math.atan2(waypoints[0][1], waypoints[0][0]))
            return [[waypoints[0][0], waypoints[0][1], final_orientation]]

        # Use atan2 from math to calculate the orientation
        oriented_waypoints = []

        for i in range(1, len(waypoints)):
            x1, y1 = waypoints[i - 1]
            x2, y2 = waypoints[i]

            orientation = math.degrees(math.atan2((y2 - y1), (x2 - x1)))

            single = [x1, y1, orientation]
            oriented_waypoints.append(single)

        # still need to add the last waypoint
        if len(waypoints) > 1:
            final_waypoint = waypoints[len(waypoints) - 1]
            final_orientation = oriented_waypoints[len(oriented_waypoints) - 1][2]
            single = [final_waypoint[0], final_waypoint[1], final_orientation]
            oriented_waypoints.append(single)

        return oriented_waypoints






