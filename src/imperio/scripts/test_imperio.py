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

""" Test for the abstract planner class using a spoof planner class.
Author: Nicole Maguire
Date: 4/12/2018
Test For Version : 1

"""
import planner

class SpoofPlanner(planner.Planner):
    def __init__(self, robot):
        super(SpoofPlanner, self).__init__(robot)

    def find_waypoints(self, goal):
        return []

class SpoofRobot():
    def localize(self):
        location = (0,0,0)
        pose = (0,0,0,0)
        return (location, pose)


class TestPlanner(object):

    def test_init(self):
        test_robot = robot(None)
        sp = SpoofPlanner(test_robot)
        assert not sp.waypoints_publisher == None
        assert sp.robot == test_robot
        assert not sp.movement_status == None

    def test_map_callback(self):
        sp = SpoofPlanner(None)
        map_message = planner.OccupancyGrid()
        sp.map_callback(map_message)
        assert not sp.occupancy_grid == None

    def test_drive_status_callback(self):
        sp = SpoofPlanner(None)

        message = planner.DriveStatus()
        message.in_motion.data = True
        sp.drive_status_callback(message)
        assert sp.movement_status == planner.MovementStatus.MOVING

        message = planner.DriveStatus()
        message.has_reached_goal.data = True
        sp.drive_status_callback(message)
        assert sp.movement_status == planner.MovementStatus.HAS_REACHED_GOAL

        message = planner.DriveStatus()
        message.is_stuck.data = True
        sp.drive_status_callback(message)
        assert sp.movement_status == planner.MovementStatus.STUCK

        message = planner.DriveStatus()
        message.cannot_plan_path.data = True
        sp.drive_status_callback(message)
        assert sp.movement_status == planner.MovementStatus.CANNOT_PLAN_PATH

    def test_navigate_to_goal(self):
        sp = SpoofPlanner(None)

        sp.movement_status = planner.MovementStatus.CANNOT_PLAN_PATH
        assert sp.navigate_to_goal(None) == None

        sp.movement_status = planner.MovementStatus.MOVING
        assert  sp.navigate_to_goal(None) == False

        sr = SpoofRobot()
        sp = SpoofPlanner(sr)
        sp.movement_status = planner.MovementStatus.HAS_REACHED_GOAL
        assert sp.navigate_to_goal((0,0)) == True

        sp.occupancy_grid = None
        result = sp.navigate_to_goal((1,1))
        assert result == False
        assert sp.movement_status == planner.MovementStatus.WAITING

        map = planner.map_utils.Map()
        sp.occupancy_grid = map
        assert not sp.occupancy_grid == None
        assert not sp.navigate_to_goal((1,1))

    def test_publish_waypoints(self):
        planner.rospy.init_node('Test_Imperio')

        sp = SpoofPlanner(None)
        waypoints = [(2, 4, 5), (34, 4, 5), (5, 2, 2)]
        sp.publish_waypoints(waypoints)
        assert sp.movement_status == planner.MovementStatus.MOVING

        sp = SpoofPlanner(None)
        waypoints = [(1,1,1),(2,4,5)]
        sp.publish_waypoints(waypoints)
        assert sp.movement_status == planner.MovementStatus.MOVING

        sp = SpoofPlanner(None)
        waypoints = [(1, 1, 1)]
        sp.publish_waypoints(waypoints)
        assert sp.movement_status == planner.MovementStatus.HAS_REACHED_GOAL

        sp = SpoofPlanner(None)
        waypoints = []
        sp.publish_waypoints(waypoints)
        assert sp.movement_status == planner.MovementStatus.HAS_REACHED_GOAL


    def test_robot_within_threshold(self):
        sr = SpoofRobot()
        sp = SpoofPlanner(sr)

        #default/launch distance threshold being .1
        assert not sp.robot_within_threshold((5,5)) #Distance : 7.071068
        assert sp.robot_within_threshold((0,0)) #Distance : 0.0
        assert sp.robot_within_threshold((0.05,0.05)) #Distance : 0.0707107
        assert not sp.robot_within_threshold((.08,.06)) #Distance : .1
        assert sp.robot_within_threshold((.08,-.05)) #Distance : .0943398


    def test_halt_movement(self):
        sp = SpoofPlanner(None)
        sp.halt_movement()

    def test_calculate_orientation(self):
        sp = SpoofPlanner(None)
        assert sp.calculate_orientation([]) == []
        assert sp.calculate_orientation(None) == []

        waypoints = [(1,1)]
        sp.calculate_orientation(waypoints)

        waypoints = [(-1,0)]
        sp.calculate_orientation(waypoints)

        waypoints = [(4,4),(4,4),(4,4),(5,5),(-1,-1),(4,2)]
        result = sp.calculate_orientation(waypoints)
        assert len(result) == len(waypoints)

        waypoints = [(1,-1),(2,-1),(3,0),(2,1)]
        expected_results = [(1,-1, 0), (2,-1,.785398), (3,0, 2.35619), (2,1,2.35619)]
        result = sp.calculate_orientation(waypoints)
        for i in range(0,len(waypoints)):
            assert abs(expected_results[i][2] - result[i][2]) < 0.001

    def test_get_robot_location(self):
        sr = SpoofRobot()
        sp = SpoofPlanner(sr)
        assert sp.get_robot_location() == (0,0,0)

import initial_planner
class TestInitialPlanner(object):
    def test_init(self):
        ip = initial_planner.InitialPlanner()
        assert not ip.theta_publisher == None
        assert ip.has_turned == False
        assert ip.planner_failed == False
        assert ip.msg_published == False

    def test_zero_pt_turn_callback(self):
        ip = initial_planner.InitialPlanner()
        msg = initial_planner.DriveStatus()
        msg.has_reached_goal.data = True
        ip.drive_status_callback(msg)
        assert ip.planner_failed == False
        assert ip.has_turned

        ip = initial_planner.InitialPlanner()
        msg = initial_planner.DriveStatus()
        msg.is_stuck.data = True
        ip.drive_status_callback(msg)
        assert ip.planner_failed
        assert ip.has_turned == False

        ip = initial_planner.InitialPlanner()
        msg = initial_planner.DriveStatus()
        msg.cannot_plan_path.data = True
        ip.drive_status_callback(msg)
        assert ip.planner_failed
        assert ip.has_turned == False

    def test_turn_to_start(self):
        ip = initial_planner.InitialPlanner()
        assert ip.turn_to_start() == False

        ip = initial_planner.InitialPlanner()
        ip.planner_failed = True
        assert ip.turn_to_start() == None

        ip = initial_planner.InitialPlanner()
        ip.has_turned = True
        assert ip.turn_to_start() == True


    def test_publish_turn_msg(self):
        ip = initial_planner.InitialPlanner()
        ip.publish_turn_msg(0)
        assert ip.msg_published == True



