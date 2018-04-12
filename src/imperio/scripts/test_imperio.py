#!/usr/bin/env python
""" Super class for a robot object and state machine, describes the methods that a Robot should have.

Author: Nicole Maguire
Date: 4/12/2018
Test For Version : 3

"""
from robot import *

class TestRobot(object):

    def test_init(self):
        r = robot(None)
        assert r.state == RobotState.INITIAL
        assert not r.tf == None
        assert r.location == None
        assert r.pose == None

    def test_change_state(self):
        r = robot(None)
        r.change_state(RobotState.DEPOSIT)
        assert r.state == RobotState.DEPOSIT
        r.change_state(RobotState.INBOUND)
        assert r.state == RobotState.INBOUND
        r.change_state((RobotState.HALT))
        assert r.state == RobotState.HALT
        r.change_state(RobotState.INITIAL)
        assert r.state == RobotState.HALT

    def test_print_state(self):
        try:
            r = robot(None)
            r.print_state(RobotState.INITIAL)
            r.print_state(RobotState.OUTBOUND)
            r.print_state(RobotState.DIG)
            r.print_state(RobotState.INBOUND)
            r.print_state(RobotState.DEPOSIT)
            r.print_state(RobotState.HALT)
            r.print_state(RobotState.RECOVERY)
        except:
            assert False

    def test_next_state(self):
        r = robot(None)
        r.next_state()
        assert r.state == RobotState.OUTBOUND
        r.next_state()
        assert r.state == RobotState.DIG
        r.next_state()
        assert r.state == RobotState.INBOUND
        r.next_state()
        assert r.state == RobotState.DEPOSIT

        r.change_state(RobotState.RECOVERY)
        r.next_state()
        assert r.state == RobotState.RECOVERY

        r.change_state(RobotState.HALT)
        r.next_state()
        assert r.state == RobotState.HALT

    def test_localize(self):
        ref_tf = tf.TransformListener()
        r = robot(None)
        robot_loc, robot_pose = r.localize()
        try:
            ref_loc, ref_pose = ref_tf.lookupTransform('/map', '/base_link', 0)
            assert robot_loc == ref_loc
            assert robot_pose == ref_pose
        except:
            #TODO : Recovery behavior instead [Jira NRMC2018-329]
            assert robot_loc == (0,0,0)
            assert robot_pose == (0,0,0,0)
        pass