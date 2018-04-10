#!/usr/bin/env python
"""
Displays GlobalWaypoints and path to RVIZ

Author: James Madison University
Date: 2/20/2018
Version: 2
"""

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from imperio.msg import GlobalWaypoints


class MarkerNode(object):
    
    def __init__(self):
        """
        initializes the node
        """
        self.node = rospy.init_node('imperio_marker_utils')
        rospy.Subscriber('/position_controller/global_planner_goal', GlobalWaypoints, self.waypoint_callback)
        self.waypoint_pub = rospy.Publisher('/waypoint_marker', Marker, queue_size=1000)
        self.path_pub = rospy.Publisher('/path_marker', Marker, queue_size=1000)
        

    def waypoint_callback(self, message):
        waypoints = message.pose_array
        self.draw_path(waypoints)
        self.mark_waypoints(waypoints)


    def draw_path(self, waypoints):
        """
        publishes markers for the path defined by a set of waypoints
        assumes that the current location point will not be included in the path given
        :param waypoints: the set of waypoints as Pose2D[]
        """
        path = Marker()
        path.header.frame_id = '/map'
        path.header.stamp = rospy.get_rostime()
        path.type = path.LINE_STRIP
        path.action = 0
        path.scale.x = .15
        path.scale.y = .15
        path.scale.z = 1
        path.color.r = 254
        path.color.g = 1
        path.color.b = 0
        path.color.a = 1
        path.lifetime = rospy.Duration(0)
        path.frame_locked = True

        points = []

        #TODO : Make this the current  [NRMC2018-334]
        msg = Point()
        msg.x, msg.y = 0,0
        points.append(msg)

        for point in waypoints:
            msg = Point()
            msg.x = point.x
            msg.y = point.y
            points.append(msg)


        path.points = points
        self.path_pub.publish(path)

    def mark_waypoints(self, waypoints):
        """
        Publishes markers for a set of points
        :param waypoints: the set of waypoints
        """
        for i in range(0, len(waypoints)):
            marker = Marker()
            marker.header.frame_id = '/map'
            marker.header.stamp = rospy.get_rostime()
            marker.type = marker.CUBE
            marker.action = 0
            marker.pose.position.x = waypoints[i].x
            marker.pose.position.y = waypoints[i].y
            marker.scale.x = .1
            marker.scale.y = .1
            marker.scale.z = 3
            marker.color.r = 254
            marker.color.g = 1
            marker.color.b = 0
            marker.color.a = 1
            marker.lifetime = rospy.Duration(0)
            marker.frame_locked = True
            marker.id = i
            self.waypoint_pub.publish(marker)

if __name__ == "__main__":
    MarkerNode()
    while not rospy.is_shutdown():
	rospy.spin()
