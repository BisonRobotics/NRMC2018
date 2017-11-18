#!/usr/bin/env python
""" Robot class for the turtlebot.

Author: Nicole Maguire
Date: 9/26/2017
Version: 1

"""
import rospy

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Sound
from kobuki_msgs.msg import Led
from robot import *

class turtlebot_robot(robot):

    def __init__(self):
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
        self.sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)
        self.light_pub = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
        self.location = (0,0)

    # How the Robot handles a low level movement command (ie Twist)
    # param : The goal coordinates as : (x,y)
    def move_to_goal(self, goal):
        print("move_to_goal : " + repr(goal))

        twist = Twist()
        twist.linear.x = goal[0]
        twist.linear.y = goal[1]

        self.twist_pub.publish(twist)

        rospy.sleep(2)

        #Fake, just using to simulate the robot not making it all the way to the goal
        loc_x = goal[0] * 0.5 + self.location[0]
        loc_y = goal[1] * 0.5 + self.location[1]
        self.location = (loc_x, loc_y)

    # Localization for the robot
    # return : The current coordinates of the robot
    def localize(self):
        print("localize yields : " + repr(self.location))
        # TODO : Better way of mimicking localization
        return self.location

    # Commands required for the robot to dig
    # Unable to actually perform on turtlebot, simply a stand-in activity
    def dig(self):
        print("Dig")
        sound = Sound()
        led = Led()

        sound.value = 6
        led.value = 4

        self.sound_pub.publish(sound)
        self.light_pub.publish(led)

        rospy.sleep(5)

        sound.value = 2
        led.value = 0

        self.sound_pub.publish(sound)
        self.light_pub.publish(led)

        rospy.sleep(5)


    # Commands required for the robot to deposit the regolith
    # Unable to actually perform on turtlebot, simply a stand-in activity
    def deposit(self):
        print("Deposit")
        self.dig()

    # The name of the robot
    # return : A string Value of the name
    def to_string(self):
        return "Turtlebot"
