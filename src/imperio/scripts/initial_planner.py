#!/usr/bin/env python
"""
Initial Planner Class - Gets the robot out of the initial orientation next to the occupancy grid

Author: Nicole Maguire - James Madison University
Date: 4/2/2018
Version: 0
"""

import time
import math
from planner import *
from harcoded_paths import hardcoded_paths

class InitialPlanner(Planner):
    def __init__(self,robot):
        super(InitialPlanner, self).__init__(robot)
        #Value checked against Robotics Mining Competition Rules and Rubrics - Part IV
        self.field_line = 1.89

    def find_waypoints(self, goal):
        x, y, orientation = self.get_robot_location()

        field = 1 if y <= self.field_line else 2
        position = self.get_starting_position(orientation)
        waypoints = hardcoded_paths[position]

        if field == 2:
            waypoints = self.flip_path(waypoints)
        return waypoints

    def flip_path(self, waypoints):
        flipped = []
        for point in waypoints:
            x,y = point[0], point[1]
            new_point = [x, -y]
            flipped.append(new_point)
        return flipped

    def get_starting_position(self, orientation):
        if orientation == 0:
            return 0
        return (orientation % math.pi) / (360 / math.pi)

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

         