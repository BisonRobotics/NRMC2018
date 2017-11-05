#!/usr/bin/env python
""" Tests the robots base method implementation using integration tests
    i.e. observe the robot and make sure it behaves as you think it will

Author: James Madison University
Date: 10/1/2017
Version: 1

"""
import time
import rospy

#IMPORT TESTING ROBOT HERE
from turtlebot_robot import *

short_sleep = 1
med_sleep = 3
long_sleep = 5

spacing = "                     "

if __name__ == "__main__":
    rospy.init_node("robot_integration_testing")

    #ASSIGN TESTING ROBOT HERE
    robot = turtlebot_robot()
    print(spacing + "Testing Robot : " + robot.to_string())

    print
    print(spacing + "Testing Movement Methods")
    print
    time.sleep(short_sleep)

    # MARK : Movement methods
    print(spacing + "Testing Move To Goal (1,1)")
    goal = (1,1)
    robot.move_to_goal(goal)

    time.sleep(short_sleep)

    print(spacing + "Testing Localize")
    location = robot.localize()
    print(spacing + "Robot is at Location : " + repr(location))
    print

    time.sleep(short_sleep)

    print(spacing + "Testing Move To Goal (-1,-1)")
    goal = (-1,-1)
    robot.move_to_goal(goal)

    time.sleep(short_sleep)

    print(spacing + "Testing Localize")
    location = robot.localize()
    print(spacing + "Robot is at Location : " + repr(location))

    time.sleep(short_sleep)

    print
    print(spacing + "Testing Regolith Methods")
    print

    time.sleep(med_sleep)

    # MARK : Regolith methods
    print(spacing + "Testing Dig")
    robot.dig()

    time.sleep(short_sleep)

    print(spacing + "Testing Deposit")
    robot.deposit()
