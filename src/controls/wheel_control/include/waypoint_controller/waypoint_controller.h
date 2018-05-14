#ifndef WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROLLER_H
#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>
#include <vector>
#include <utility>
#include <waypoint_controller/pose.h>
#include <waypoint_controller/maneuver.h>
#include <waypoint_controller/waypoint_with_maneuvers.h>
#include <localizer/localizer_interface.h>

#define _USE_MATH_DEFINES
#include <cmath>
#define DT_THAT_SHALL_BE_USED .02

namespace WaypointControllerNs
{
typedef struct waypointGains_s
{
  double eppgain;
  double epdgain;
  double etpgain;
  double etdgain;
  double epplpgain;
  double etplpgain;
  double wheelalpha;
  double wheelerrorgain;
} waypointControllerGains;
}

static constexpr WaypointControllerNs::waypointControllerGains waypoint_default_gains = {
  /*DNFW*/
  .eppgain = 30,
  .epdgain = -10,  //.55,
  .etpgain = 20,
  .etdgain = -10,  //.8,
  .epplpgain = 2 * M_PI * DT_THAT_SHALL_BE_USED * .1608 / (2 * M_PI * DT_THAT_SHALL_BE_USED * .1608 + 1),
  .etplpgain = 2 * M_PI * DT_THAT_SHALL_BE_USED * .1608 / (2 * M_PI * DT_THAT_SHALL_BE_USED * .1608 + 1),
  .wheelalpha = .1,
  .wheelerrorgain = 10
  /*DNFW*/
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
                     iVescAccess *br, iVescAccess *bl, double gaurenteedDt,
                     WaypointControllerNs::waypointControllerGains gains);
  std::vector<std::pair<double, double> > addWaypoint(pose waypoint, pose currRobotPose);
  Status update(LocalizerInterface::stateVector stateVector, double dt);
  std::vector<waypointWithManeuvers> getNavigationQueue();  // DEBUG
  pose getCPP();                                            // DEBUG
  unsigned int getCurrManeuverIndex();                      // DEBUG
  pose getManeuverEnd();                                    // DEBUG
  double getETpEstimate();                                  // DEBUG
  double getETdEstimate();                                  // DEBUG
  double getEPpEstimate();                                  // DEBUG
  double getEPdEstimate();                                  // DEBUG
  double getDist2endOnPath();                               // DEBUG
  double getDist2endAbs();                                  // DEBUG
  double getStuckMetric();                                  // DEBUG
  std::pair<double, double> getSetSpeeds();                 // DEBUG
  std::pair<double, double> getCmdSpeeds();                 // DEBUG
  
  void scootBack();

  void haltAndAbort();
  void clearControlStates();

private:
  void modifyNavQueue2RecoverFromPathError(pose RobotPose, pose manEnd);
  void modifyNavQueue2RecoverFromGoalOvershoot();
  void halt();
  std::vector<waypointWithManeuvers> navigationQueue;
  iVescAccess *front_left_wheel, *front_right_wheel, *back_right_wheel, *back_left_wheel;
  double axelLen, maxSpeed;
  double EPpGain, EPdGain, ETpGain, ETdGain, EPpLowPassGain, ETpLowPassGain;  // path and theta error gains
  double EPpLowPass, EPpLowPassPrev, ETpLowPass, ETpLowPassPrev, EPpDerivFiltEst, ETpDerivFiltEst;
  double EPpEst, ETpEst;

  double LvelCmd, RvelCmd;
  double LvelCmdPrev, RvelCmdPrev;
  double WheelAlpha;
  double WheelErrorGain;
  double LWheelError, RWheelError;
  double LeftWheelSetSpeed, RightWheelSetSpeed;

  unsigned int currManeuverIndex;
  bool doingManeuver;

  double dist2endOnPath, dist2endAbs, dist2Path;
  
  double stuckMetric;
  double prevDistToEndAbs;
  bool unstucking;
  bool unstucking_cooldown;
  double time_unstucking;
  double time_unstucking_cooldown;
  
  bool backing_it_in;
  bool squaring_it_up;
  double time_backing;
  double time_squaring;
  
  double time_scooting;
  
  pose maneuverEnd;
  maneuver currMan;
  pose theCPP;
};

#endif
