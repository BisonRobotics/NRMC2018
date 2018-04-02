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

        of_start_x = 1.5
        of_end_x = 4.44

        comp_start = []
        comp_start.append(self.grid_occupied_score(1))
        comp_start.append(self.grid_occupied_score(2))
        comp_start.append(self.grid_occupied_score(3))

        #find part of the obstacle grid with the least amount of occupied space in it


        print("Imperio : It took {} seconds to find the starting position".format(time.time() - saved_time))
        return (of_start_x,1)

    def grid_occupied_score(self, region):
        # -----|-----|-----
        # --1--|--2--|--3--
        # -----|-----|-----
        # -----|-0,0-|-----
        # ^Regions

         