#!/usr/bin/env python
"""
Abstract Planner Class
LEGACY CODE WARNING : Do not try to instantiate this class, it is an abstract class

Author: Nicole Maguire - James Madison University
Date: 4/12/2018
Version: 1
"""

import math
import rospy
import map_utils
from robot import *
from abc import ABCMeta, abstractmethod

from imperio.msg import GlobalWaypoints
from imperio.msg import DriveStatus
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Empty

class MovementStatus(Enum):
    """
    Movement Status is the enum of the different states that the robot can be in for movement.
    """
    MOVING = 0
    HAS_REACHED_GOAL = 1
    STUCK = 2
    CANNOT_PLAN_PATH = 3
    WAITING = 4


class Planner(object):
    """
    Abstract Planner Class
    """
    __metaclass__ = ABCMeta

    def __init__(self, robot):
        """
        Initializes the global planner
        :param robot: the robot object the planner will be moving
        """
        self.waypoints_publisher = rospy.Publisher('/position_controller/global_planner_goal', GlobalWaypoints, queue_size=100, latch=True)
        self.halt_publisher = rospy.Publisher('/position_controller/halt', Empty, queue_size=1, latch=True)

        rospy.Subscriber('/position_controller/drive_controller_status', DriveStatus, self.drive_status_callback)
        rospy.Subscriber('/costmap_2d_node/costmap/costmap', OccupancyGrid, self.map_callback)
        #rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.robot = robot
        self.occupancy_grid = None
        self.movement_status = MovementStatus.HAS_REACHED_GOAL
        self.goal_given = False
        self.halt = False

    def map_callback(self, map_message):
        self.occupancy_grid = map_utils.Map(map_message)

    def drive_status_callback(self, status_message):
        """
        Callback for when the drive status is published
        :param status_message: the message that was published
        """
        if status_message.in_motion.data:
            self.movement_status = MovementStatus.MOVING
        if status_message.has_reached_goal.data:
            self.movement_status = MovementStatus.HAS_REACHED_GOAL
            rospy.loginfo("[IMPERIO] : Movement Status HAS_REACHED_GOAL")
        if status_message.is_stuck.data:
            self.movement_status = MovementStatus.STUCK
            rospy.logwarn("[IMPERIO] : Movement Status STUCK")
        if status_message.cannot_plan_path.data:
            self.movement_status = MovementStatus.CANNOT_PLAN_PATH
            rospy.logwarn("[IMPERIO] : Movement Status CANNOT_PLAN_PATH")

    def navigate_to_goal(self, goal):
        """
        Attempts to navigate to the final goal
        :param goal: the global goal (based on the overall map) as (x,y)
        :return: a boolean of it the robot has reached the goal, None for a fatal error
        """
        if self.halt:
            return None
        if self.movement_status == MovementStatus.CANNOT_PLAN_PATH:
            return None
        if self.movement_status == MovementStatus.MOVING:
            return False
        if self.movement_status == MovementStatus.HAS_REACHED_GOAL and self.goal_given:
            self.goal_given = False
            return True

        rospy.loginfo("[IMPERIO] : PLANNING A PATH TO GOAL {}".format(goal))

        #We can't do anything until we have the occupancy grid
        if self.occupancy_grid == None:
            rospy.logwarn("[IMPERIO] : Cannot find the occupancy grid")
            self.movement_status = MovementStatus.WAITING
            return False
        rospy.loginfo("[IMPERIO] : Occupancy Grid Exists")

        waypoints = self.find_waypoints(goal)
        oriented_waypoints = self.calculate_orientation(waypoints)
        rospy.loginfo("[IMPERIO] : Path found : {}".format(oriented_waypoints))
        if oriented_waypoints == [] or len(oriented_waypoints) < 2:
            rospy.logwarn("[IMPERIO] : No possible path found")
            return None

        oriented_waypoints.pop(0)
        self.publish_waypoints(oriented_waypoints)
        self.movement_status = MovementStatus.MOVING
        self.goal_given = True
        rospy.loginfo("[IMPERIO] : For Goal {}".format(goal))
        return False

    @abstractmethod
    def find_waypoints(self, goal):
        """
        Finds the waypoints to the goal
        :param goal: the final goal as (x,y)
        :return: an array of waypoints
        """
        pass

    def publish_waypoints(self, waypoints):
        """
        Publishes the waypoints to the local planner
        :param waypoints: an array of oriented waypoint
        """

        message = GlobalWaypoints()
        pose_array = []

        for point in waypoints:
            msg = Pose2D()
            msg.x, msg.y, msg.theta = point
            pose_array.append(msg)

        message.pose_array = pose_array
        self.waypoints_publisher.publish(message)
        rospy.loginfo("[IMPERIO] : Waypoints published to local planner")

    def halt_movement(self):
        """
        Halts the command of the robot
        """
        rospy.loginfo("[IMPERIO] : Halting Navigation")
        self.halt_publisher.publish(Empty())
        self.halt = True

    def calculate_orientation(self, waypoints):
        """
        Calcularets the orientation for each waypoint, will just point straight to the next waypoint
        :param waypoints: array of waypoints
        :return: array of waypoints with orientation
        """

        final_orientation = 0

        # Check if none or one waypoint
        if (waypoints == None or len(waypoints) == 0):
            return []

        if (len(waypoints) == 1):
            return [[waypoints[0][0], waypoints[0][1], final_orientation]]

        # Use atan2 from math to calculate the orientation
        oriented_waypoints = []

        for i in range(1, len(waypoints)):
            point1 = waypoints[i - 1]
            point2 = waypoints[i]
            x1, y1 = point1[0], point1[1]
            x2, y2 = point2[0], point2[1]

            orientation = self.orient_forwards(math.atan2((y2 - y1), (x2 - x1)))

            single = [x1, y1, orientation]
            oriented_waypoints.append(single)

        # still need to add the last waypoint
        final_waypoint = waypoints[len(waypoints) - 1]
        single = [final_waypoint[0], final_waypoint[1], final_orientation]
        oriented_waypoints.append(single)

        return oriented_waypoints

    def get_robot_location(self):
        (location, pose) = self.robot.localize()
        return location

    def orient_forwards(self, orientation):
        if orientation > math.pi/2:
            return orientation - math.pi
        if orientation < -math.pi/2:
            return orientation + math.pi
        return orientation




