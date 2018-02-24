#ifndef WAYPOINT_CONTROLLER_HELPER_H
#define WAYPOINT_CONTROLLER_HELPER_H
#include <waypoint_controller/maneuver.h>
#include <waypoint_controller/pose.h>
#include <waypoint_controller/waypoint_with_maneuvers.h>
#include <vector>

namespace WaypointControllerHelper
{
std::vector<maneuver> waypoint2maneuvers(pose robotPose, pose waypoint);

pose findCPP(pose robotPose, maneuver curManeuver);

pose endOfManeuver(pose robotPose, maneuver myMan);

std::pair<double, double> speedAndRadius2WheelVels(double speed, double radius, double AxelLen, double maxSpeed);

std::vector<std::pair<double, double> > waypointWithManeuvers2points(waypointWithManeuvers myMan);

double anglediff(double x, double y);

int sign(double A);  // MUST BE INT

pose reflectWaypointAroundRobot(pose waypoint, pose robot);

std::vector<maneuver> oneTurnSolution(pose robotPose, pose waypoint);

std::vector<maneuver> inverseOneTurnSolution(pose robotPose, pose waypoint);

std::vector<maneuver> twoTurnSolution(pose robotPose, pose waypoint);

pose transformPoseToRobotCoord(pose robotPose, pose waypoint);

std::pair<pose, pose> inputCleaner(pose robotPose, pose waypoint);

maneuver transformManeuverToWorldCoord(pose robotPose, maneuver myMan);
}

#endif
