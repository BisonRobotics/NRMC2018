#include <waypoint_controller/waypoint_controller.h>
#include <waypoint_controller/waypoint_controller_helper.h>
#include <cmath>

#define POSITIONTOL .30f
#define ANGLETOL .2f
#define BADLINE .3f

bool APPROX(double A, double B, double T)
{
  return ((A > B - T && A < B + T) ? true : false);
}

double DIST(double A, double B, double C, double D)
{
  return sqrt((A - C) * (A - C) + (B - D) * (B - D));
}

WaypointController::WaypointController(float axelLength, float maxSafeSpeed, pose initialPose, iVescAccess *fl,
                                       iVescAccess *fr, iVescAccess *br, iVescAccess *bl)
{
  axelLen = axelLength;
  maxSpeed = maxSafeSpeed;
  front_left_wheel = fl;
  front_right_wheel = fr;
  back_right_wheel = br;
  back_left_wheel = bl;

  EPpGain = .02;
  EPdGain = .16;
  ETpGain = .0005;
  ETdGain = .0040;
  EPpLowPassGain = .01;
  ETpLowPassGain = .01;
  WheelSpeedPGain = .018;

  // control states
  EPpLowPass = 0;
  EPpLowPassPrev = 0;
  ETpLowPass = 0;
  ETpLowPassPrev = 0;
  EPpDerivFiltEst = 0;
  ETpDerivFiltEst = 0;

  EPpEst = 0;
  ETpEst = 0;

  currManeuverIndex = 0;
  doingManeuver = false;

  LvelCmd = 0;
  RvelCmd = 0;
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

float WaypointController::getETpEstimate()  // DEBUG
{
  return ETpEst;
}

float WaypointController::getEPpEstimate()  // DEBUG
{
  return EPpEst;
}

std::pair<float, float> WaypointController::getSetSpeeds()
{
  return std::pair<float, float>(LeftWheelSetSpeed, RightWheelSetSpeed);
}

void WaypointController::haltAndAbort()
{
  front_left_wheel->setLinearVelocity(0);
  back_left_wheel->setLinearVelocity(0);
  front_right_wheel->setLinearVelocity(0);
  back_right_wheel->setLinearVelocity(0);
}

std::vector<std::pair<float, float> > WaypointController::addWaypoint(pose waypoint, pose currRobotPose)
{
  // add waypoint to queue, and calculate needed maneuvers to get there
  waypointWithManeuvers newWaypoint;
  if (navigationQueue.size() == 0)  // this is the only waypoint right now
  {
    newWaypoint.initialPose = currRobotPose;
    newWaypoint.terminalPose = waypoint;
    newWaypoint.mans = waypoint2maneuvers(currRobotPose, waypoint);
    // so plan from where the robot is to there
  }
  else
  {
    newWaypoint.initialPose = navigationQueue.back().terminalPose;
    newWaypoint.terminalPose = waypoint;
    newWaypoint.mans = waypoint2maneuvers(navigationQueue.back().terminalPose, waypoint);
    // plan from last point to this new one
  }
  navigationQueue.push_back(newWaypoint);  // append calculated maneuvers to queue

  // calculate points that will be covered by the new maneuver
  std::vector<std::pair<float, float> > myVector;

  myVector = waypointWithManeuvers2points(newWaypoint);
  // if returned vector is empty, we were unable to plan a path

  return myVector;
}

WaypointController::Status WaypointController::update(pose robotPose, float dt)
{
  // navQueue has some elements in it
  // these elements consist of starting and terminal poses along with the maneuvers to get from A to B

  if (navigationQueue.size() > 0)  // places to go, there are waypoints in the navigationQueue
  {
    if (!doingManeuver)  // maneuver is completed or one has not yet been started, need to calculate end point and base
                         // wheel speeds
    {
      if (currManeuverIndex > navigationQueue.at(0).mans.size() - 1)  // did all maneuvers for waypoint
      {
        currManeuverIndex = 0;
        navigationQueue.erase(navigationQueue.begin());
        /*//remember to stop
        front_left_wheel->setLinearVelocity(0);
        back_left_wheel->setLinearVelocity(0);
        front_right_wheel->setLinearVelocity(0);
        back_right_wheel->setLinearVelocity(0);*/
        return Status::ALLGOOD;  // gotta get out of here, if the navigationQueue is empty, then the next time
        // update is called, this function will return GOALREACHED and stop the robot
      }
      currMan = navigationQueue.at(0).mans.at(currManeuverIndex);  // set current maneuver

      if (currManeuverIndex == 0)  // if this is the first maneuver on the stack for this waypoint
        maneuverEnd = endOfManeuver(navigationQueue.at(0).initialPose,
                                    currMan);  // the end pose is the extension from the initial pose
      else
        maneuverEnd = endOfManeuver(maneuverEnd, currMan);
      // else the end pose is from the last maneuverEnd through the current maneuver

      std::pair<float, float> myPair = speedAndRadius2WheelVels(.6f * maxSpeed, currMan.radius, axelLen, maxSpeed);
      LeftWheelSetSpeed = myPair.first;
      RightWheelSetSpeed = myPair.second;
      doingManeuver = true;
    }
    else  // doing a maneuver, need to see if robot has completed it
    {
      if (APPROX(DIST(robotPose.x, robotPose.y, maneuverEnd.x, maneuverEnd.y), 0, POSITIONTOL))
      {  // reached the end of this maneuver
        doingManeuver = false;
        currManeuverIndex++;
        return Status::ALLGOOD;  // next time function is called, maneuver will update
      }
    }
    theCPP = findCPP(robotPose, currMan);  // closest pose on path

    // figure out if we are in tolerance or not
    Status returnStatus;
    if (APPROX(DIST(robotPose.x, robotPose.y, theCPP.x, theCPP.y), 0, BADLINE))
      returnStatus = Status::ALLGOOD;
    else
      returnStatus = Status::ALLBAD;

    // do control system calculations
    if (currMan.radius > 0)  // signed turn radius
      EPpEst = currMan.radius - DIST(currMan.xc, currMan.yc, robotPose.x, robotPose.y);
    else
      EPpEst = currMan.radius + DIST(currMan.xc, currMan.yc, robotPose.x, robotPose.y);

    ETpEst = anglediff(theCPP.theta, robotPose.theta);  // positive error means turn left

    EPpLowPassPrev = EPpLowPass;
    EPpLowPass = EPpLowPassGain * EPpEst + (1 - EPpLowPassGain) * EPpLowPassPrev;
    ETpLowPassPrev = ETpLowPass;
    ETpLowPass = ETpLowPassGain * ETpEst + (1 - ETpLowPassGain) * ETpLowPassPrev;

    EPpDerivFiltEst = (EPpLowPass - EPpLowPassPrev) / dt;
    ETpDerivFiltEst = (ETpLowPass - ETpLowPassPrev) / dt;

    LvelCmd = LvelCmd + (EPpGain * EPpEst + EPdGain * EPpDerivFiltEst) -
              (ETpGain * ETpEst + ETdGain * ETpDerivFiltEst) - WheelSpeedPGain * (LvelCmd - LeftWheelSetSpeed);
    RvelCmd = RvelCmd - (EPpGain * EPpEst + EPdGain * EPpDerivFiltEst) +
              (ETpGain * ETpEst + ETdGain * ETpDerivFiltEst) - WheelSpeedPGain * (RvelCmd - RightWheelSetSpeed);

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
