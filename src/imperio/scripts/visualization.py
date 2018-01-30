#!/usr/bin/env python
"""
Helper file to help visualize autonomy productions

Author: James Madison University
Date: 1/22/2018
Version: 1
"""

import rospy
import map_utils

from imperio.msg import GlobalWaypoints
from nav_msgs.msg import OccupancyGrid

import matplotlib.pyplot as plt

class Visualizaion(object):

    def __init__(self):
        """
        Initializes the global planner
        :param robot: the robot object the planner will be moving
        """
        rospy.init_node("imperio_viz")
        print("Imperio VIS: Visualization initiated")
        rospy.Subscriber('/draw_points', GlobalWaypoints, self.draw_points_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.map = None

    def map_callback(self, map_message):
        self.map = map_utils.Map(map_message)

    def draw_points_callback(self, message):

        print("Imperio VIS: Drawing Points")
        waypoints = []
        for pose in message.pose_array:
            point = (pose.x, pose.y)
            waypoints.append(point)

        for x in range(1, len(waypoints)):
            x1, y1 = waypoints[x - 1]
            x2, y2 = waypoints[x]
            plt.plot([x1, x2], [y1, y2])

        '''obstacle_grid = self.map
        for col in range(0, obstacle_grid.width - 1):
            for row in range(0, obstacle_grid.height - 1):
                if obstacle_grid.grid[row][col] < .7:
                    print("something should be here")
                    x,y = obstacle_grid.cell_position(row, col)
                    plt.plot(x,y)

        print("That didn't take toooo long")'''


        plt.xlim(0, 7)
        plt.ylim(0, 7)

        plt.show()

if __name__ == "__main__":
    nav = Visualizaion()
    #TODO : Check to make sure this is efficient
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()