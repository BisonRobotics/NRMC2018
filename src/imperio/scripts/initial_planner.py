#!/usr/bin/env python
"""
Initial Planner Class - Gets the robot out of the initial orientation next to the occupancy grid

Author: Nicole Maguire - James Madison University
Date: 4/2/2018
Version: 0
"""

from planner import *

class InitialPlanner(Planner):
    def __init__(self,robot):
        super(InitialPlanner, self).__init__(robot)

    def find_waypoints(self, goal):
        current_location = self.get_robot_location()
        #Dummy Data
        waypoints = [current_location, goal]
        return waypoints

    def find_best_starting_goal(self):
        #Dummy Data
        return (1,1)
