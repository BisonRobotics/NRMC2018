#ifndef WAYPOINT_CONTROLLER_HELPER_H
#define WAYPOINT_CONTROLLER_HELPER_H
#include <waypoint_controller/maneuver.h>
#include <waypoint_controller/pose.h>
#include <waypoint_controller/waypointWithManeuvers.h>
#include <vector>

std::vector<maneuver> waypoint2maneuvers(pose robotPose, pose waypoint);

pose findCPP(pose robotPose, maneuver curManeuver);

pose endOfManeuver(pose robotPose, maneuver myMan);

std::pair<float, float> speedAndRadius2WheelVels(float speed, float radius, float AxelLen, float maxSpeed);

std::vector<std::pair<float, float> > waypointWithManeuvers2points(waypointWithManeuvers myMan);

float anglediff(float x, float y);

pose reflectWaypointAroundRobot(pose waypoint, pose robot);

std::vector<maneuver> oneTurnSolution(pose robotPose, pose waypoint);

#endif