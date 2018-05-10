#!/usr/bin/env python
""" Time the duration of the competition limit and publish to timesUp

Author: James Madison University
Date: 2/11/2018
Version: 2

"""

import rospy
from std_msgs.msg import Bool


class CompetitionTimer(object):
    def __init__(self):
        self.node = rospy.init_node('competition_timer')
        #Gets the time in seconds
        self.start_time = rospy.get_time()
        self.timerPublisher = rospy.Publisher('/times_up', Bool, queue_size=1)

        self.time_limit = rospy.get_param('/time_limit') * 60
        if self.time_limit == None:
            self.time_limit = 600

        self.times_up_param = rospy.get_param('/turn_around')
        if self.times_up_param == None:
            self.times_up_param = 1

        self.opt_points_turn_around = rospy.get_param('/optimize_points_turn_around') * 60
        if self.opt_points_turn_around == None:
            self.opt_points_turn_around = 4

        self.turn_around = self.time_limit - self.times_up_param * 60
        
        self.run_timer()

    def publish_turn_around(self, time_passed):
        rospy.loginfo("[COMP TIMER] : Time to turn around. Time Passed: {}".format(time_passed))
        turn_around_time = Bool()
        turn_around_time.data = False
        self.timerPublisher.publish(turn_around_time)

    def publish_times_up(self):
        rospy.loginfo("[COMP TIMER] : Time is Up. Competition Time: {}".format(self.time_limit/60))
        times_up = Bool()
        times_up.data = True
        self.timerPublisher.publish(times_up)

    def run_timer(self):
        rospy.loginfo("[COMP TIMER] : Starting Competition Timer with {} minutes.".format(self.time_limit))
        rospy.loginfo("[COMP TIMER] : Turn around warning will occur {} minutes before the end of competition".format(self.times_up_param))
        opt_turn_around_not_published = True
        final_turn_around_not_published = True
        times_up_not_published = True
        while not rospy.is_shutdown() and times_up_not_published:
            time_passed = rospy.get_time() - self.start_time
            if time_passed >= self.opt_points_turn_around and opt_turn_around_not_published:
                self.publish_turn_around(time_passed)
                opt_turn_around_not_published = False
            if time_passed >= self.turn_around and final_turn_around_not_published:
                self.publish_turn_around(time_passed)
                final_turn_around_not_published = False
            if time_passed >= self.time_limit and times_up_not_published:
                self.publish_times_up()
                times_up_not_published = False

if __name__ == "__main__":
    timer = CompetitionTimer()