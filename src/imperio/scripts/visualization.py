#!/usr/bin/env python
"""
Helper file to help visualize autonomy productions

Author: James Madison University
Date: 1/22/2018
Version: 1
"""

import rospy

from imperio.msg import GlobalWaypoints

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

    def draw_points_callback(self, message):
        print("Imperio VIS: Drawing Points")
        waypoints = []
        for pose in message.pose_array:
            point = (pose.x, pose.y)
            waypoints.append(point)
        self.draw_tree(waypoints)

    def draw_tree(self, waypoints):
        for x in range(1, len(waypoints)):
            x1, y1 = waypoints[x - 1]
            x2, y2 = waypoints[x]
            plt.plot([x1, x2], [y1, y2])

        # configure plot axises
        plt.xlim(0, 10)
        plt.ylim(-6, 6)

        plt.show()

if __name__ == "__main__":
    nav = Visualizaion()
    #TODO : Check to make sure this is efficient
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()