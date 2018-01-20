#ifndef WAYPOINT_WITH_MANEUVERS_H
#define WAYPOINT_WITH_MANEUVERS_H

#include <waypoint_controller/maneuver.h>
#include <waypoint_controller/pose.h>
#include <vector>

struct waypointWithManeuvers
{
  pose initialPose, terminalPose;
  std::vector<maneuver> mans;
};

#endif