#!/usr/bin/env python
""" Time the duration of the competition limit and publish to timesUp

Author: James Madison University
Date: 11/11/2017
Version: 1

"""

import rospy
from std_msgs.msg import Bool

rospy.init_node('competition_timer')
timerPublisher = rospy.Publisher('/times_up', Bool, queue_size=1)
time_limit = rospy.get_param('/time_limit')
turn_around = rospy.get_param('/turn_around')

rospy.sleep((time_limit - turn_around) * 60)
turn_around_time = Bool()
turn_around_time.data = False
timerPublisher.publish(turn_around_time)

rospy.sleep(turn_around * 60)
times_up = Bool()
times_up.data = True
timerPublisher.publish(times_up)

