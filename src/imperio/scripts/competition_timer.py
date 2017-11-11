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
timeLimit = rospy.get_param('/time_limit')

rospy.sleep(timeLimit * 60)

timesUp = Bool()
timesUp.data = True
timerPublisher.publish(timesUp)
