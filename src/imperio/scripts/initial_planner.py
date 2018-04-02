#!/usr/bin/env python
"""
Initial Planner Class - Gets the robot out of the initial orientation next to the occupancy grid

Author: Nicole Maguire - James Madison University
Date: 4/2/2018
Version: 0
"""

import time
from planner import *

class InitialPlanner(Planner):
    def __init__(self,robot):
        super(InitialPlanner, self).__init__(robot)

    def find_waypoints(self, goal):
        current_location = self.get_robot_location()
        #Dummy Data
        waypoints = [current_location, (1,1), goal]
        return waypoints

    def find_best_starting_goal(self):
        saved_time = time.time()
        #TODO : need to find the exact y point of the start of the obstacle grid . . .using 2 for now
        y = 2

        print("Imperio : It took {} seconds to find the starting position".format(time.time() - saved_time))
        return (1,y)
