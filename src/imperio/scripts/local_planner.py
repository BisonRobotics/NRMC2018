#!/usr/bin/env python
""" Move the robot to desired location.
Author: Nicole Maguire
Version: 10/1/2016
"""
import rospy
import math
import tf


from geometry_msgs.msg import Twist
from imperio.msg import GlobalWaypoints
from imperio.msg import DriveStatus

class TurtleNavNode(object):

    def __init__(self):
        """ Initialize the turtle_nav node. """
        rospy.init_node('turtle_nav')
        self.set = False
        self.P = .75
        self.K = 4
        self.goal_x = 1
        self.goal_y = 1
        self.tf = tf.TransformListener()
        self.drive_pub = rospy.Publisher('/vrep/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/global_planner_goal', GlobalWaypoints, self.handle_goal)
        rospy.spin()



    def handle_goal(self, req):
        """ callback method for the service, sets the goals"""
        self.goal_x = req.goal_x
        self.goal_y = req.goal_y
        self.run()
        return True

    def run(self):
        """ runs the turtlebot and publishes a twist method"""
        twist = Twist()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            (self.location, self.pose) = self.tf.lookupTransform('/map', '/base_link', rospy.Time(0))
            self.thetag = math.atan2(self.goal_y - self.pose.y, self.goal_x - self.pose.x)
            if(math.sqrt((self.location.x - self.goal_x)**2 + (self.location.y - self.goal_y)**2) <.1):
                return
            twist.linear.x = self.P * math.sqrt((self.location.x - self.goal_x)**2 + (self.location.y - self.goal_y)**2)
            twist.angular.z = self.K * (self.thetag - self.location.theta)
            self.drive_pub.publish(twist)
            rate.sleep()


if __name__ == "__main__":
    """initialized the node"""
    nav = TurtleNavNode()
    while not rospy.is_shutdown():
        rospy.spin()