#ifndef WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROLLER_H
#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>
#include <vector>
#include <utility>
#include <waypoint_controller/pose.h>
#include <waypoint_controller/maneuver.h>
#include <waypoint_controller/waypoint_with_maneuvers.h>


namespace WaypointControllerNs {
    typedef struct waypointGains_s {
        double eplpgain;
        double eplpalpha;
        double eppgain;
        double epdgain;
        double etpgain;
        double etdgain;
        double epplpgain;
        double etplpgain;
        double wheelspeedgain;
        double wheelalpha;
    }waypointControllerGains;


}

 static constexpr WaypointControllerNs::waypointControllerGains waypoint_default_gains = {
        .eplpgain = 0.0,
        .eplpalpha = 0,
        .eppgain = 0,
        .epdgain=.75,
        .etpgain = 0,
        .etdgain = .2,
        .epplpgain = 0,
        .etplpgain = 0,
        .wheelspeedgain  = 0,
        .wheelalpha = .5
};


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
    OVERSHOT,
    OFFPATH,
    CANTPLAN,
    ISSTUCK
  };
  WaypointController(double axelLength, double maxSafeSpeed, pose initialPose, iVescAccess *fl, iVescAccess *fr,
                     iVescAccess *br, iVescAccess *bl, double gaurenteedDt, WaypointControllerNs::waypointControllerGains gains);
  std::vector<std::pair<double, double> > addWaypoint(pose waypoint, pose currRobotPose);
  Status update(pose robotPose, double dt);
  std::vector<waypointWithManeuvers> getNavigationQueue();  // DEBUG
  pose getCPP();                                            // DEBUG
  unsigned int getCurrManeuverIndex();                      // DEBUG
  pose getManeuverEnd();                                    // DEBUG
  double getETpEstimate();                                  // DEBUG
  double getEPpEstimate();                                  // DEBUG
  double getDist2endOnPath();                               // DEBUG
  std::pair<double, double> getSetSpeeds();                 // DEBUG
  std::pair<double, double> getCmdSpeeds();  // DEBUG

  void haltAndAbort();
  void clearControlStates();

private:
  void modifyNavQueue2RecoverFromPathError(pose RobotPose, pose theCPP);
  void modifyNavQueue2RecoverFromGoalOvershoot();
  void halt();
  std::vector<waypointWithManeuvers> navigationQueue;
  iVescAccess *front_left_wheel, *front_right_wheel, *back_right_wheel, *back_left_wheel;
  double axelLen, maxSpeed;
  double EPpGain, EPdGain, ETpGain, ETdGain, EPpLowPassGain, ETpLowPassGain, EPlpGain,
      EPlpAlpha;  // path and theta error gains
  double EPpLowPass, EPpLowPassPrev, ETpLowPass, ETpLowPassPrev, EPpDerivFiltEst, ETpDerivFiltEst, EPLowerPass,
      EPLowerPassPrev;
  double EPpEst, ETpEst;
  double WheelSpeedPGain;
  double LvelCmd, RvelCmd;
  double LvelCmdPrev, RvelCmdPrev;
  double WheelAlpha;
  double LeftWheelSetSpeed, RightWheelSetSpeed;
  unsigned int currManeuverIndex;
  bool doingManeuver;

  double dist2endOnPath, dist2endAbs, dist2Path;

  pose maneuverEnd;
  maneuver currMan;
  pose theCPP;
};


#endif
