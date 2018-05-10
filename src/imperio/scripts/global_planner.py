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

        rospy.loginfo("[IMPERIO] : Starting the path planner")
        saved_time = time.time()

        #Hardcoded for competition
        location = (0.6, 0.0, 0.0)

        #TODO [JIRA : NRMC2018-577] might run just one non threaded path to check if there IS a possible path in the expanded map
        #TODO [JIRA : NRMC2018-578] possible run the two results against each other in case minimal path is better
        results = RRT.find_best_rrt_path(location, goal, self.expanded_map, 20)

        if len(results) == 0:
            rospy.logwarn("[IMPERIO] : NO POSSIBLE PATH FOUND FOR EXPANDED MAP")
            rospy.loginfo("[IMPERIO] : Switching to the minimal map")
            results = RRT.find_best_rrt_path(location, goal, self.minimal_map, 20)
            if len(results) == 0:
                rospy.logwarn("[IMPERIO] : CRITICAL FAILURE")
                rospy.logwarn("[IMPERIO] : NO POSSIBLE PATH FOUND FOR MINIMAL MAP")

        rospy.loginfo("[IMPERIO] : Path Planning Complete. Total path planning time: {} seconds".format(time.time() - saved_time))
        return results

