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

""" Test for the Imperio Controll class
Author: Nicole Maguire
Date: 4/12/2018
Test For Version : 2

"""
import imperio_control as imperio
class TestImperioControl(object):
    def test_init(self):

        rospy.init_node("imperio_test")
        ic = imperio.ImperioControl()
        assert not ic.robot == None
        assert not ic.initial_planner == None
        assert not ic.planner == None
        assert not ic.rm == None
        assert ic.starting_region == None

    def test_halt_callback(self):
        pass

    def test_timer_callback(self):
        pass

    def test_run(self):
        pass

    def test_initital(self):
        pass

    def test_outbound(self):
        pass

    def test_inbound(self):
        pass

    def test_dig(self):
        pass

    def test_deposit(self):
        pass

    def test_halt(self):
        pass

    def test_recover(self):
        pass

    def test_runnint_out_of_time(self):
        pass

""" Test for the abstract planner class using a spoof planner class.
Author: Nicole Maguire
Date: 4/12/2018
Test For Version : 1

"""
import planner
import math

class SpoofPlanner(planner.Planner):
    def __init__(self, robot):
        super(SpoofPlanner, self).__init__(robot)

    def find_waypoints(self, goal):
        return []

class SpoofPlannerWaypoints(planner.Planner):
    def __init__(self, robot):
        super(SpoofPlannerWaypoints, self).__init__(robot)

    def find_waypoints(self, goal):
        return [(0,0), (1,1), (2,2)]

    def publish_waypoints(self, oriented_waypoints):
        pass

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
        assert sp.goal_given == False

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
        result = sp.navigate_to_goal((1, 1))
        assert result == False
        assert sp.movement_status == planner.MovementStatus.WAITING

        map = planner.map_utils.Map()
        sp.occupancy_grid = map
        assert not sp.occupancy_grid == None
        assert not sp.navigate_to_goal((1, 1))
        assert sp.goal_given == False

        spw = SpoofPlannerWaypoints(None)
        spw.occupancy_grid = map
        assert not spw.navigate_to_goal((1,1))
        assert spw.goal_given == True

        spw.movement_status = planner.MovementStatus.HAS_REACHED_GOAL
        assert spw.navigate_to_goal((0,0)) == True
        assert spw.goal_given == False


    def test_publish_waypoints(self):
        #planner.rospy.init_node('Test_Imperio')

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

    def test_halt_movement(self):
        sp = SpoofPlanner(None)
        sp.halt_movement()

    def test_orient_forward(self):
        sp = SpoofPlanner(None)
        assert sp.orient_forwards(0) == 0
        assert sp.orient_forwards(math.pi) == 0
        assert sp.orient_forwards(math.pi/2) == math.pi/2
        assert sp.orient_forwards(-math.pi/2) == -math.pi/2
        assert sp.orient_forwards(3.0/4.0*math.pi) == -math.pi/4
        assert sp.orient_forwards(-3.0/4.0*math.pi) == math.pi/4

    def test_calculate_orientation(self):
        sp = SpoofPlanner(None)
        assert sp.calculate_orientation([]) == []
        assert sp.calculate_orientation(None) == []

        waypoints = [(1,1)]
        sp.calculate_orientation(waypoints)

        waypoints = [(-1,0)]
        result = sp.calculate_orientation(waypoints)
        assert result[0][2] == 0

        waypoints = [(4,4),(4,4),(4,4),(5,5),(-1,-1),(4,2)]
        result = sp.calculate_orientation(waypoints)
        assert len(result) == len(waypoints)

        waypoints = [(1,-1),(2,-1),(3,0),(2,1)]
        expected_results = [(1,-1, 0), (2,-1,.785398), (3,0, 2.35619), (2,1,0)]
        result = sp.calculate_orientation(waypoints)
        for i in range(0,len(waypoints)):
            assert abs(sp.orient_forwards(expected_results[i][2]) - result[i][2]) < 0.001

    def test_get_robot_location(self):
        sr = SpoofRobot()
        sp = SpoofPlanner(sr)
        assert sp.get_robot_location() == (0,0,0)

import regolith_manipulation as reg_man

class SpoofRegMan(reg_man.RegolithManipulation):
    def __init__(self):
        super(SpoofRegMan, self).__init__()

<<<<<<< Updated upstream
    def single_dig(self):
        #Removed call to dig client
        pass

    def single_dump(self):
        #Removed call to dump client
        pass

class TestRegolithManipulation(object):

    def test_init(self):
        rm = reg_man.RegolithManipulation()
        assert not rm.dig_client == None
        assert not rm.dump_client == None
        assert rm.regolith_in_bucket == 0
        assert rm.waiting_on_action == False
        assert rm.halt == False

    def test_halt(self):
        rm = reg_man.RegolithManipulation()
        rm.halt_regolith_commands()

        assert rm.halt == True
        assert rm.dig_regolith() == None
        assert rm.deposit_regolith() == None

    def test_dump_goal_message(self):
        rm = reg_man.RegolithManipulation()
        message = rm.dump_goal_message()
        assert not message == None

    def test_dig_goal_message(self):
        rm = reg_man.RegolithManipulation()
        message = rm.dig_goal_message()
        assert not message == None

    def test_dig_regolith(self):
        rm = SpoofRegMan()
        result = rm.dig_regolith()
        assert result == False
        assert rm.waiting_on_action == True

        rm = SpoofRegMan()
        rm.regolith_in_bucket = 10
        result = rm.dig_regolith()
        assert result == True
        assert rm.waiting_on_action == False

    def test_deposit_regolith(self):
        rm = SpoofRegMan()
        result = rm.deposit_regolith()
        assert result == True
        assert rm.waiting_on_action == False

        rm = SpoofRegMan()
        rm.regolith_in_bucket = 10
        result = rm.deposit_regolith()
        assert result == False
        assert rm.waiting_on_action == True
=======
>>>>>>> Stashed changes

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



