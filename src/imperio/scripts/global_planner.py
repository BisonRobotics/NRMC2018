#!/usr/bin/env python
"""
Move the robot to desired location.

Author: Nicole Maguire - James Madison University
Date: 4/2/2018
Version: 4
"""

import time
import RRT

from planner import *

class GlobalPlanner(Planner):
    """
    Global Planner takes a goal point in relation to the global map and outputs waypoints to the local planner.
    """

    def __init__(self, robot):
        super(GlobalPlanner, self).__init__(robot)

    def find_waypoints(self, goal):
        """
        Finds the waypoints in the occupancy grid, navigation around obstacles
        :param goal: the final goal as (x,y)
        :return: an array of waypoints
        """
        location = self.get_robot_location()

        print("Imperio : Starting the path planner")
        saved_time = time.time()
        results = RRT.find_best_rrt_path(location, goal, self.occupancy_grid, 20)
        print("Imperio : Path Planning Complete. Total path planning time: {} seconds".format(time.time() - saved_time))

        if len(results) == 0:
            #TODO : Enter recovery behavior [Jira NRMC2018-336]
            print("IMPERIO : NO POSSIBLE PATH FOUND")

        return results