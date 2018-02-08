#include <waypoint_controller/waypoint_controller.h>
#include <waypoint_controller/waypoint_controller_helper.h>
#include <cmath>

#define POSITIONTOL .50f
#define GOALREACHEDDIST .10f
#define ANGLETOL .2f
#define SPEED_CONST .2
bool approx(double A, double B, double T)
{
  return ((A > B - T && A < B + T) ? true : false);
}

double dist(double A, double B, double C, double D)
{
  return sqrt((A - C) * (A - C) + (B - D) * (B - D));
}

void modifyNavQueue2RecoverFromPathError(); //queue modifications if outside of POSITIONTOL
void modifyNavQueue2RecoverFromGoalOvershoot(); //queue modifcations if goal has been overshot

WaypointController::WaypointController(double axelLength, double maxSafeSpeed, pose initialPose, iVescAccess *fl,
                                       iVescAccess *fr, iVescAccess *br, iVescAccess *bl, double gaurenteedDt, WaypointControllerNs::waypointControllerGains gains)
{
  axelLen = axelLength;
  maxSpeed = maxSafeSpeed;
  front_left_wheel = fl;
  front_right_wheel = fr;
  back_right_wheel = br;
  back_left_wheel = bl;
/*
  EPlpGain = 0;  //.0012;
  EPlpAlpha = 2 * M_PI * gaurenteedDt * .00008 / (2 * M_PI * gaurenteedDt * .00008 + 1);

  EPpGain = 0;  //.0015;
  EPdGain = .75;
  ETpGain = 0;  //.0001;
  ETdGain = .2;
  EPpLowPassGain = 2 * M_PI * gaurenteedDt * .1608 / (2 * M_PI * gaurenteedDt * .1608 + 1);
  ETpLowPassGain = 2 * M_PI * gaurenteedDt * .1608 / (2 * M_PI * gaurenteedDt * .1608 + 1);
  WheelSpeedPGain = 0;  //.009;
*/
  EPlpGain = gains.eplpgain;
  EPlpAlpha = gains.eplpalpha;
  EPpGain = gains.eppgain;
  EPdGain = gains.epdgain;
  ETpGain = gains.etpgain;
  ETdGain = gains.etdgain;
  EPpLowPassGain = gains.epplpgain;
  ETpLowPassGain = gains.etplpgain;
  WheelSpeedPGain = gains.wheelspeedgain;
  WheelAlpha = gains.wheelalpha;

  navigationQueue.clear();

  // control states
  clearControlStates();

  currManeuverIndex = 0;
  doingManeuver = false;
}

std::vector<waypointWithManeuvers> WaypointController::getNavigationQueue()  // DEBUG
{
  return navigationQueue;
}

pose WaypointController::getCPP()  // DEBUG
{
  return theCPP;
}

unsigned int WaypointController::getCurrManeuverIndex()  // DEBUG
{
  return currManeuverIndex;
}

pose WaypointController::getManeuverEnd()  // DEBUG
{
  return maneuverEnd;
}

double WaypointController::getETpEstimate()  // DEBUG
{
  return ETpEst;
}

double WaypointController::getEPpEstimate()  // DEBUG
{
  return EPpEst;
}

std::pair<double, double> WaypointController::getSetSpeeds()
{
  return std::pair<double, double>(LeftWheelSetSpeed, RightWheelSetSpeed);
}

std::pair<double, double> WaypointController::getCmdSpeeds()
{
  return std::pair<double, double>(LvelCmd, RvelCmd);
}

void WaypointController::clearControlStates()
{
  EPpLowPass = 0;
  EPpLowPassPrev = 0;
  ETpLowPass = 0;
  ETpLowPassPrev = 0;
  EPpDerivFiltEst = 0;
  ETpDerivFiltEst = 0;

  EPLowerPass = 0;
  EPLowerPassPrev = 0;

  EPpEst = 0;
  ETpEst = 0;

  LvelCmd = 0;
  RvelCmd = 0;

  LvelCmdPrev = LvelCmd;
  RvelCmdPrev = RvelCmd;
}

void WaypointController::haltAndAbort()
{
  front_left_wheel->setLinearVelocity(0);
  back_left_wheel->setLinearVelocity(0);
  front_right_wheel->setLinearVelocity(0);
  back_right_wheel->setLinearVelocity(0);

  clearControlStates();

  currManeuverIndex = 0;
  doingManeuver = false;

  navigationQueue.clear();
}

// TODO
// Dump future maneuvers

// TODO
// get planned goodness method
// X distance travelled / distance taken for all future maneuvers

std::vector<std::pair<double, double> > WaypointController::addWaypoint(pose waypoint, pose currRobotPose)
{
  // add waypoint to queue, and calculate needed maneuvers to get there
  waypointWithManeuvers newWaypoint;
  if (navigationQueue.size() == 0)  // this is the only waypoint right now
  {
    newWaypoint.initialPose = currRobotPose;
    newWaypoint.terminalPose = waypoint;
    newWaypoint.mans = WaypointControllerHelper::waypoint2maneuvers(currRobotPose, waypoint);
    // so plan from where the robot is to there
  }
  else
  {
    newWaypoint.initialPose = navigationQueue.back().terminalPose;
    newWaypoint.terminalPose = waypoint;
    newWaypoint.mans = WaypointControllerHelper::waypoint2maneuvers(navigationQueue.back().terminalPose, waypoint);
    // plan from last point to this new one
  }
  navigationQueue.push_back(newWaypoint);  // append calculated maneuvers to queue

  // calculate points that will be covered by the new maneuver
  std::vector<std::pair<double, double> > myVector;

  myVector = WaypointControllerHelper::waypointWithManeuvers2points(newWaypoint);
  // if returned vector is empty, we were unable to plan a path

  return myVector;
}

WaypointController::Status WaypointController::update(pose robotPose, double dt)
{
  // navQueue has some elements in it
  // these elements consist of starting and terminal poses along with the maneuvers to get from A to B

  Status returnStatus = Status::ALLGOOD;
  if (navigationQueue.size() > 0)  // places to go, there are waypoints in the navigationQueue
  {
    if (!doingManeuver)  // maneuver is completed or one has not yet been started, need to calculate end point and base
                         // wheel speeds
    {
      if (currManeuverIndex > navigationQueue.at(0).mans.size() - 1)  // did all maneuvers for waypoint
      {
        currManeuverIndex = 0;
        navigationQueue.erase(navigationQueue.begin());
        clearControlStates();
        return Status::ALLGOOD;  // gotta get out of here, if the navigationQueue is empty, then the next time
        // update is called, this function will return GOALREACHED and stop the robot
      }
      currMan = navigationQueue.at(0).mans.at(currManeuverIndex);  // set current maneuver
      theCPP = WaypointControllerHelper::findCPP(robotPose, currMan);  // closest pose on path
      if (currManeuverIndex == 0)
      {  // if this is the first maneuver on the stack for this waypoint
        maneuverEnd =
            WaypointControllerHelper::endOfManeuver(navigationQueue.at(0).initialPose,
                                                    currMan);  // the end pose is the extension from the initial pose
      }
      else
      {
        maneuverEnd = WaypointControllerHelper::endOfManeuver(maneuverEnd, currMan);
        // else the end pose is from the last maneuverEnd through the current maneuver
      }
      std::pair<double, double> myPair =
          WaypointControllerHelper::speedAndRadius2WheelVels(SPEED_CONST * maxSpeed * WaypointControllerHelper::sign(currMan.distance),
                                                             currMan.radius, axelLen, maxSpeed);
      // LeftWheelSetSpeed = myPair.first;
      // RightWheelSetSpeed = myPair.second;
      // reset control states
      clearControlStates();
      // input/seed new calculated wheel velocities
      LvelCmd = myPair.first;
      RvelCmd = myPair.second;

      doingManeuver = true;
    }
    else  // doing a maneuver, need to see if robot has completed it
    {
      /*
      if (approx(dist(robotPose.x, robotPose.y, maneuverEnd.x, maneuverEnd.y), 0, POSITIONTOL))
      {  // reached the end of this maneuver
        doingManeuver = false;
        currManeuverIndex++;
        return Status::ALLGOOD;  // next time function is called, maneuver will update
      }*/
      theCPP = WaypointControllerHelper::findCPP(robotPose, currMan);  // closest pose on path
      dist2endOnPath = currMan.radius*WaypointControllerHelper::anglediff(theCPP.theta, maneuverEnd.theta);
      dist2endAbs =  dist(robotPose.x, robotPose.y, maneuverEnd.x, maneuverEnd.y);
      dist2Path = dist(robotPose.x, robotPose.y, maneuverEnd.x, maneuverEnd.y);

      if (approx(dist2endOnPath,0, GOALREACHEDDIST) && approx(dist2endAbs,0,GOALREACHEDDIST)) //reached waypoint in a good way (on the end point)
      {
        doingManeuver = false;
        currManeuverIndex++;
        return Status::ALLGOOD;  // next time function is called, maneuver will update and either start next maneuver or stop
                                 // if it was the last one
      }
      if ( !approx(dist2Path,0,POSITIONTOL) ) //fell off of path (to the side most likely)
      {
         modifyNavQueue2RecoverFromPathError();
         returnStatus = Status::ALLBAD;// for now, keep old implementation of just raising warning
      }
      if ( dist2endOnPath < -GOALREACHEDDIST) //overshot path and drove past goal (but still might be close to path)
      {
         modifyNavQueue2RecoverFromGoalOvershoot();
         returnStatus = Status::ALLBAD;// for now, keep old implementation of just raising warning
      }
    }

    // do control system calculations
    if (currMan.radius > 0)
    {  // WaypointControllerHelper::signed turn radius, positive means turn left
      EPpEst = currMan.radius - dist(currMan.xc, currMan.yc, robotPose.x, robotPose.y);
    }
    else
    {
      EPpEst = currMan.radius + dist(currMan.xc, currMan.yc, robotPose.x, robotPose.y);
    }
    ETpEst = WaypointControllerHelper::anglediff(robotPose.theta,
                                                 theCPP.theta);  // order?  // positive error means turn left

    EPpLowPassPrev = EPpLowPass;
    EPpLowPass = EPpLowPassGain * EPpEst + (1 - EPpLowPassGain) * EPpLowPassPrev;
    ETpLowPassPrev = ETpLowPass;
    ETpLowPass = ETpLowPassGain * ETpEst + (1 - ETpLowPassGain) * ETpLowPassPrev;

    EPLowerPass = (EPlpAlpha * EPpEst + (1 - EPlpAlpha) * EPLowerPassPrev);
    EPLowerPassPrev = EPLowerPass;

    EPpDerivFiltEst = (EPpLowPass - EPpLowPassPrev) / dt;
    ETpDerivFiltEst = WaypointControllerHelper::anglediff(ETpLowPass, ETpLowPassPrev) / dt;  // order?

    LvelCmdPrev = LvelCmd;
    RvelCmdPrev = RvelCmd;

    LvelCmd = LvelCmd + WaypointControllerHelper::sign(currMan.distance) *
              ((EPpGain * EPpEst + EPdGain * EPpDerivFiltEst + EPlpGain * EPLowerPass) + WaypointControllerHelper::sign(currMan.distance) *
               (ETpGain * ETpEst + ETdGain * ETpDerivFiltEst) - WheelSpeedPGain * (LvelCmd - LeftWheelSetSpeed)) * dt;
    RvelCmd = RvelCmd - WaypointControllerHelper::sign(currMan.distance) *
              ((EPpGain * EPpEst + EPdGain * EPpDerivFiltEst + EPlpGain * EPLowerPass) + WaypointControllerHelper::sign(currMan.distance) *
               (ETpGain * ETpEst + ETdGain * ETpDerivFiltEst) - WheelSpeedPGain * (RvelCmd - RightWheelSetSpeed)) * dt;

    LvelCmd = WheelAlpha * LvelCmd + (1.0-WheelAlpha) * LvelCmdPrev;
    RvelCmd = WheelAlpha * RvelCmd + (1.0-WheelAlpha) * RvelCmdPrev;

    front_left_wheel->setLinearVelocity(LvelCmd);
    back_left_wheel->setLinearVelocity(LvelCmd);
    front_right_wheel->setLinearVelocity(RvelCmd);
    back_right_wheel->setLinearVelocity(RvelCmd);
    return returnStatus;
  }
  else
  {
    // remember to stop
    this->haltAndAbort();
    return Status::GOALREACHED;  // gotta get out of here
  }
}

void modifyNavQueue2RecoverFromPathError()
{
  //should plan a maneuver to last known cpp 
  //and insert this before the current maneuver (that was blown)
  //and switch to this new maneuver
  //when the new maneuver completes, the robot will switch to the blown one
  //and continue where it left off
    waypointWithManeuvers newWaypoint;
    newWaypoint.initialPose = currRobotPose;
    newWaypoint.terminalPose = theCPP; //last known good CPP
    newWaypoint.mans = WaypointControllerHelper::waypoint2maneuvers(currRobotPose, waypoint);

    std::vector<waypointWithManeuvers>::iterator it;
    it = navigationQueue.begin();
    navigationQueue.insert(it, newWaypoint);
    currManeuverIndex =0;

}

void modifyNavQueue2RecoverFromGoalOvershoot()
{
  //should just consider the current maneuver achieved and begin to attempt the next manuever
  doingManeuver = false;
  currManeuverIndex++;
}
