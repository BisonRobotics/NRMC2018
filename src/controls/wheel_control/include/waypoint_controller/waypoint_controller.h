#ifndef WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROLLER_H
#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>
#include <vector>
#include <utility>
#include <waypoint_controller/pose.h>
#include <waypoint_controller/maneuver.h>
#include <waypoint_controller/waypointWithManeuvers.h>

// a manuever is one arc that the robot does
// it has a radius, a turn center (in world coord)
// and a distance. a negative distance means to travel
// the arc backwards
class WaypointController
{
public:
  enum Status
  {
    ALLGOOD,
    GOALREACHED,
    ALLBAD
  };
  WaypointController(float axelLength, float maxSafeSpeed, pose initialPose, iVescAccess *fl, iVescAccess *fr,
                     iVescAccess *br, iVescAccess *bl);
  std::vector<std::pair<float, float> > addWaypoint(pose waypoint, pose currRobotPose);
  Status update(pose robotPose, float dt);
  std::vector<waypointWithManeuvers> getNavigationQueue();  // DEBUG
  pose getCPP();  // DEBUG
  unsigned int getCurrManeuverIndex();  // DEBUG
  pose getManeuverEnd();  // DEBUG
  float getETpEstimate();  // DEBUG
  float getEPpEstimate();  // DEBUG
  std::pair<float, float> getSetSpeeds();  // DEBUG

  void haltAndAbort();

private:
  std::vector<waypointWithManeuvers> navigationQueue;
  iVescAccess *front_left_wheel, *front_right_wheel, *back_right_wheel, *back_left_wheel;
  float axelLen, maxSpeed;
  float EPpGain, EPdGain, ETpGain, ETdGain, EPpLowPassGain, ETpLowPassGain;  // path and theta error gains
  float EPpLowPass, EPpLowPassPrev, ETpLowPass, ETpLowPassPrev, EPpDerivFiltEst, ETpDerivFiltEst;
  float EPpEst, ETpEst;
  float WheelSpeedPGain;
  float LvelCmd, RvelCmd;
  float LeftWheelSetSpeed, RightWheelSetSpeed;
  unsigned int currManeuverIndex;
  bool doingManeuver;

  pose maneuverEnd;
  maneuver currMan;
  pose theCPP;
};

#endif
