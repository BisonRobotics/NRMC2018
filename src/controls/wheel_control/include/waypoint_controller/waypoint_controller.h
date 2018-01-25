#ifndef WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROLLER_H
#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>
#include <vector>
#include <utility>
#include <waypoint_controller/pose.h>
#include <waypoint_controller/maneuver.h>
#include <waypoint_controller/waypoint_with_maneuvers.h>

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
  WaypointController(double axelLength, double maxSafeSpeed, pose initialPose, iVescAccess *fl, iVescAccess *fr,
                     iVescAccess *br, iVescAccess *bl);
  std::vector<std::pair<double, double> > addWaypoint(pose waypoint, pose currRobotPose);
  Status update(pose robotPose, double dt);
  std::vector<waypointWithManeuvers> getNavigationQueue();  // DEBUG
  pose getCPP();                                            // DEBUG
  unsigned int getCurrManeuverIndex();                      // DEBUG
  pose getManeuverEnd();                                    // DEBUG
  double getETpEstimate();                                   // DEBUG
  double getEPpEstimate();                                   // DEBUG
  std::pair<double, double> getSetSpeeds();                   // DEBUG
  std::pair<double, double> getCmdSpeeds();                  //DEBUG

  void haltAndAbort();

private:
  std::vector<waypointWithManeuvers> navigationQueue;
  iVescAccess *front_left_wheel, *front_right_wheel, *back_right_wheel, *back_left_wheel;
  double axelLen, maxSpeed;
  double EPpGain, EPdGain, ETpGain, ETdGain, EPpLowPassGain, ETpLowPassGain;  // path and theta error gains
  double EPpLowPass, EPpLowPassPrev, ETpLowPass, ETpLowPassPrev, EPpDerivFiltEst, ETpDerivFiltEst;
  double EPpEst, ETpEst;
  double WheelSpeedPGain;
  double LvelCmd, RvelCmd;
  double LeftWheelSetSpeed, RightWheelSetSpeed;
  unsigned int currManeuverIndex;
  bool doingManeuver;

  pose maneuverEnd;
  maneuver currMan;
  pose theCPP;
};

#endif
