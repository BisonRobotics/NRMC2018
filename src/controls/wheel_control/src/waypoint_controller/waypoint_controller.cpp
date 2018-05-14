#include <waypoint_controller/waypoint_controller.h>
#include <waypoint_controller/waypoint_controller_helper.h>
#define _USE_MATH_DEFINES
#include <cmath>

#define POSITIONTOL .40f  // should be well above noise floor of localization and wide enough
// for robot to make a corrective maneuver/zero point turn in
#define GOALREACHEDDIST .100f  // should be about the size of the noise floor of localization
// this also determines how far you can overshoot a goal
#define ANGLETOL .2f
#define SPEED_CONST .4  // average speed for the wheels in linear m/s
#define MAX_ABS_WHEEL_SPEED .28
#define MIN_WHEEL_SPEED .1
#define ZERO_TOLERANCE_POLICY .001

#define STUCK_METRIC_ALPHA .7
#define STUCK_METRIC_THRESHOLD .03
#define STUCK_METRIC_RELEASE .07
#define STUCK_TIMEOUT .8
#define STUCK_COOLDOWN 2.0

#define BACKUP_TIME 3.0
#define SCOOT_BACK_TIME .15
#define SQUARING_UP_TIMEOUT 9.0

bool approx(double A, double B, double T)
{
  return ((A > B - T && A < B + T) ? true : false);
}

double dist(double A, double B, double C, double D)
{
  return sqrt((A - C) * (A - C) + (B - D) * (B - D));
}

double clamp(double A, double upper, double lower)
{
    return ((A > upper) ? upper : ((A < lower) ? lower : A));
}

double bump(double A, double minimum, double zero_tol)
{
    return (std::abs(A) < minimum && std::abs(A) > zero_tol) ? WaypointControllerHelper::sign(A) * std::abs(minimum) : 
           ((std::abs(A) > minimum) ? A : 0);
}

double interpolateYFromXAndTwoPoints(double x0, double y0, double x1, double y1, double x)
{
    return ((y1 - y0)/(x1 - x0)) * (x - x0) + y0;
}

WaypointController::WaypointController(double axelLength, double maxSafeSpeed, pose initialPose, iVescAccess *fl,
                                       iVescAccess *fr, iVescAccess *br, iVescAccess *bl, double gaurenteedDt,
                                       WaypointControllerNs::waypointControllerGains gains)
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

    EPpGain = 0;  //0;
    EPdGain = .55;//.55;
    ETpGain = 0;  //0;
    ETdGain = .8; //.8;
    EPpLowPassGain = 2 * M_PI * gaurenteedDt * .1608 / (2 * M_PI * gaurenteedDt * .1608 + 1);
    ETpLowPassGain = 2 * M_PI * gaurenteedDt * .1608 / (2 * M_PI * gaurenteedDt * .1608 + 1);
    WheelSpeedPGain = 0;  //.0;
  */
  EPpGain = gains.eppgain;
  EPdGain = gains.epdgain;
  ETpGain = gains.etpgain;
  ETdGain = gains.etdgain;
  EPpLowPassGain = gains.epplpgain;
  ETpLowPassGain = gains.etplpgain;
  WheelAlpha = gains.wheelalpha;
  WheelErrorGain = gains.wheelerrorgain;
  
  dist2endAbs =0;
  prevDistToEndAbs =0;
  stuckMetric = 0;
  unstucking = false;
  unstucking_cooldown = false;
  time_unstucking =0;
  time_unstucking_cooldown =0;

  backing_it_in = false;
  time_backing = 0;
  time_squaring = 0;
  
  time_scooting = 0;
  
  // control states
  clearControlStates();
  navigationQueue.clear();
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

double WaypointController::getETdEstimate()  // DEBUG
{
  return ETpDerivFiltEst;
}

double WaypointController::getEPdEstimate()  // DEBUG
{
  return EPpDerivFiltEst;
}


double WaypointController::getDist2endOnPath()
{
  return dist2endOnPath;
}

double WaypointController::getDist2endAbs()
{
  return dist2endAbs;
}

double WaypointController::getStuckMetric()
{
    return stuckMetric;
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

  EPpEst = 0;
  ETpEst = 0;

  LvelCmd = 0;
  RvelCmd = 0;

  LWheelError = 0;
  RWheelError = 0;
}

void WaypointController::scootBack()
{
    time_scooting = SCOOT_BACK_TIME;
}

void WaypointController::haltAndAbort()
{
  halt();

  currManeuverIndex = 0;
  doingManeuver = false;

  navigationQueue.clear();
}

void WaypointController::halt()
{
  front_left_wheel->setLinearVelocity(0);
  back_left_wheel->setLinearVelocity(0);
  front_right_wheel->setLinearVelocity(0);
  back_right_wheel->setLinearVelocity(0);
  LvelCmdPrev = 0;  // 0
  RvelCmdPrev = 0;  // 0

  clearControlStates();
}

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

  // Check the path we planned,
  // make sure that it is inbounds.
  // Fix it if its not.

  // if (!mansAreInBounds())
  // fixWaypoint(&newWaypoint);

  navigationQueue.push_back(newWaypoint);  // append calculated maneuvers to queue

  // calculate points that will be covered by the new maneuver
  std::vector<std::pair<double, double> > myVector;

  myVector = WaypointControllerHelper::waypointWithManeuvers2points(newWaypoint);
  // if returned vector is empty, we were unable to plan a path

  return myVector;
}

WaypointController::Status WaypointController::update(LocalizerInterface::stateVector stateVector, double dt)
{
  // navQueue has some elements in it
  // these elements consist of starting and terminal poses along with the maneuvers to get from A to B

  pose robotPose;
  robotPose.x = stateVector.x_pos;
  robotPose.y = stateVector.y_pos;
  robotPose.theta = stateVector.theta;

  bool aggressiveFix = false;

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
      currMan = navigationQueue.at(0).mans.at(currManeuverIndex);      // set current maneuver
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
      
      //if maneuver end is negative x, maneuver means send it for a few seconds backwards
      //after reaching previous waypoint
      if (navigationQueue.at(0).terminalPose.x < 0)
      {
        backing_it_in = true;
        squaring_it_up = true;
        doingManeuver = true;
        clearControlStates();
      }
      else
      {
        std::pair<double, double> myPair = WaypointControllerHelper::speedAndRadius2WheelVels(
            SPEED_CONST * maxSpeed * WaypointControllerHelper::sign(currMan.distance), currMan.radius, axelLen, maxSpeed);
        LeftWheelSetSpeed = myPair.first;
        RightWheelSetSpeed = myPair.second;
        // reset control states
        clearControlStates();
        // input/seed new calculated wheel velocities
        LvelCmd = myPair.first;
        RvelCmd = myPair.second;

        doingManeuver = true;
      }

    }
    else // doing a maneuver, need to see if robot has completed it
    {
        
        if (backing_it_in)
        {
            if (!squaring_it_up)
            {
                time_backing += dt;
                if (time_backing > BACKUP_TIME)
                {
                    backing_it_in = false;
                    time_backing = 0;
                    prevDistToEndAbs =0;
                    dist2endAbs =0;
                    //doingManeuver = false;
                    //currManeuverIndex++;
                    //return Status::ALLGOOD;
                    this->haltAndAbort();
                    return Status::GOALREACHED;
                }
            }
        }
        else
        {
            
      theCPP = WaypointControllerHelper::findCPP(robotPose, currMan);  // closest pose on path
      prevDistToEndAbs = dist2endAbs; 
      dist2endAbs = dist(robotPose.x, robotPose.y, maneuverEnd.x, maneuverEnd.y);
      
      stuckMetric = STUCK_METRIC_ALPHA * ((prevDistToEndAbs > dist2endAbs) ? (prevDistToEndAbs - dist2endAbs)/dt : stuckMetric) + (1 - STUCK_METRIC_ALPHA) * stuckMetric;
      
      if (std::abs(currMan.radius) > 8)  // if straight line path, use linear projection
      {
        
        dist2endOnPath = -WaypointControllerHelper::sign(currMan.distance) 
                         * WaypointControllerHelper::sign((robotPose.x - maneuverEnd.x)*(maneuverEnd.y + sin(maneuverEnd.theta + M_PI_2) - maneuverEnd.y)
                                                         - (robotPose.y - maneuverEnd.y)*(maneuverEnd.x + cos(maneuverEnd.theta + M_PI_2) - maneuverEnd.x))
                         * dist(robotPose.x, robotPose.y, maneuverEnd.x, maneuverEnd.y);
      }
      else
      {
          dist2endOnPath = WaypointControllerHelper::sign(currMan.distance) * currMan.radius *
                       (WaypointControllerHelper::anglediff(maneuverEnd.theta, theCPP.theta));
      }
      
      dist2Path = dist(robotPose.x, robotPose.y, theCPP.x, theCPP.y);

      if (approx(dist2endAbs, 0, GOALREACHEDDIST))  // reached waypoint in a good way (on the end point)
      {
        doingManeuver = false;
        currManeuverIndex++;
        return Status::ALLGOOD;  // next time function is called, maneuver will update and either start next maneuver or
                                 // stop
                                 // if it was the last one
      }
      if (std::abs(dist2Path) > POSITIONTOL)  // fell off of path (to the side most likely)
      {
        // stop wheels on inside side
        aggressiveFix = true;
        // continue execution
        returnStatus = Status::OFFPATH;
      }
      if (dist2endOnPath < 0)  // overshot path and drove past goal (but still might be close to path)
      {
        modifyNavQueue2RecoverFromGoalOvershoot();  // mark maneuver as complete
        return Status::OVERSHOT;  // next time function is called, maneuver will update and either start next maneuver
                                  // or stop
        // if it was the last one.
      }
      // todo: implement cantplan
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
    if (std::abs(currMan.radius) > 900)
    {
        EPpEst = WaypointControllerHelper::sign(std::atan2(robotPose.y - (theCPP.y - WaypointControllerHelper::sign(currMan.distance) * .5*sin(theCPP.theta)),
                                                           robotPose.x - (theCPP.x - WaypointControllerHelper::sign(currMan.distance) * .5*cos(theCPP.theta))))
                 * dist(robotPose.x, robotPose.y, theCPP.x, theCPP.y);
    }
    ETpEst = WaypointControllerHelper::anglediff(robotPose.theta, theCPP.theta);
    // positive error means turn left

    EPpLowPassPrev = EPpLowPass;
    EPpLowPass = EPpLowPassGain * EPpEst + (1 - EPpLowPassGain) * EPpLowPassPrev;
    ETpLowPassPrev = ETpLowPass;
    ETpLowPass = ETpLowPassGain * ETpEst + (1 - ETpLowPassGain) * ETpLowPassPrev;

    EPpDerivFiltEst = (EPpLowPass - EPpLowPassPrev) / dt;
    ETpDerivFiltEst = WaypointControllerHelper::anglediff(ETpLowPass, ETpLowPassPrev) / dt;  // order?

    double speedEst = sqrt(stateVector.x_vel * stateVector.x_vel + stateVector.y_vel * stateVector.y_vel);
    double radiusEst = (stateVector.omega != 0) ? speedEst / stateVector.omega : 1000;

    std::pair<double, double> myPair =
        WaypointControllerHelper::speedAndRadius2WheelVels(speedEst, radiusEst, axelLen, maxSpeed);
    double LeftWheelEffectiveSpeed = myPair.first;
    double RightWheelEffectiveSpeed = myPair.second;
    RWheelError = RvelCmd - RightWheelEffectiveSpeed;
    LWheelError = LvelCmd - LeftWheelEffectiveSpeed;

    LvelCmd = LeftWheelSetSpeed +
              WaypointControllerHelper::sign(currMan.distance) *
                  ((EPpGain * EPpEst + EPdGain * EPpDerivFiltEst) +
                   WaypointControllerHelper::sign(currMan.distance) * (ETpGain * ETpEst + ETdGain * ETpDerivFiltEst) +
                   WheelErrorGain * RWheelError) *
                  dt;

    RvelCmd = RightWheelSetSpeed -
              WaypointControllerHelper::sign(currMan.distance) *
                  ((EPpGain * EPpEst + EPdGain * EPpDerivFiltEst) +
                   WaypointControllerHelper::sign(currMan.distance) * (ETpGain * ETpEst + ETdGain * ETpDerivFiltEst) -
                   WheelErrorGain * LWheelError) *
                  dt;

    LvelCmd = WheelAlpha * LvelCmd + (1.0 - WheelAlpha) * LvelCmdPrev;
    RvelCmd = WheelAlpha * RvelCmd + (1.0 - WheelAlpha) * RvelCmdPrev;
    LvelCmdPrev = LvelCmd;
    RvelCmdPrev = RvelCmd;
    
    //to clamp transparently or to not clamp transparently
    double LvelCmdClamped = clamp(LvelCmd, MAX_ABS_WHEEL_SPEED, -MAX_ABS_WHEEL_SPEED);
    double RvelCmdClamped = clamp(RvelCmd, MAX_ABS_WHEEL_SPEED, -MAX_ABS_WHEEL_SPEED);
    
    LvelCmdClamped = bump(LvelCmdClamped, MIN_WHEEL_SPEED, ZERO_TOLERANCE_POLICY);
    RvelCmdClamped = bump(RvelCmdClamped, MIN_WHEEL_SPEED, ZERO_TOLERANCE_POLICY);
    
    if (stuckMetric < STUCK_METRIC_THRESHOLD && !unstucking && !unstucking_cooldown)
    {
        unstucking = true;
    }
    
    if ((true || !aggressiveFix) && !unstucking && !unstucking_cooldown && !backing_it_in) //stuckMetric is ~velocity
    {
      front_left_wheel->setLinearVelocity(LvelCmdClamped);
      back_left_wheel->setLinearVelocity(LvelCmdClamped);
      front_right_wheel->setLinearVelocity(RvelCmdClamped);
      back_right_wheel->setLinearVelocity(RvelCmdClamped);
    }
    else if (backing_it_in)
    {
      if (squaring_it_up)
      {
          time_squaring += dt;
          double angle_measurement = WaypointControllerHelper::anglediff(robotPose.theta, 0);
          if (std::abs(angle_measurement) > .05 && time_squaring < SQUARING_UP_TIMEOUT)
          {
            front_left_wheel->setLinearVelocity(.12 * WaypointControllerHelper::sign(angle_measurement));
            back_left_wheel->setLinearVelocity(.12 * WaypointControllerHelper::sign(angle_measurement));
            front_right_wheel->setLinearVelocity(-.12 * WaypointControllerHelper::sign(angle_measurement));
            back_right_wheel->setLinearVelocity(-.12 * WaypointControllerHelper::sign(angle_measurement));

          }
          else 
          {
              squaring_it_up = false;
              time_squaring = 0;
          }
      }
      else
      {
        front_left_wheel->setLinearVelocity(-.12);
        back_left_wheel->setLinearVelocity(-.12);
        front_right_wheel->setLinearVelocity(-.12);
        back_right_wheel->setLinearVelocity(-.12); 
      }
    }
    else if (unstucking && !unstucking_cooldown)
    {
        time_unstucking += dt;
        if (time_unstucking > STUCK_TIMEOUT)
        {
            unstucking = false;
            time_unstucking = 0.0;
            unstucking_cooldown = true;
        }
        
        if (stuckMetric < STUCK_METRIC_RELEASE)
        {
           front_left_wheel->setLinearVelocity(.15*WaypointControllerHelper::sign(currMan.distance));
           back_left_wheel->setLinearVelocity(.15*WaypointControllerHelper::sign(currMan.distance));
           front_right_wheel->setLinearVelocity(.15*WaypointControllerHelper::sign(currMan.distance));
           back_right_wheel->setLinearVelocity(.15*WaypointControllerHelper::sign(currMan.distance)); 
        }
        else
        {
            unstucking = false;
        }
    }
    else if (unstucking_cooldown)
    {
        time_unstucking_cooldown += dt;
        /*
        front_left_wheel->setLinearVelocity(0);
        back_left_wheel->setLinearVelocity(0);
        front_right_wheel->setLinearVelocity(0);
        back_right_wheel->setLinearVelocity(0); 
        */
        front_left_wheel->setLinearVelocity(LvelCmdClamped);
        back_left_wheel->setLinearVelocity(LvelCmdClamped);
        front_right_wheel->setLinearVelocity(RvelCmdClamped);
        back_right_wheel->setLinearVelocity(RvelCmdClamped);
        
        if (time_unstucking_cooldown > STUCK_COOLDOWN)
        {
            unstucking_cooldown = false;
            time_unstucking_cooldown =0;
        }
    }
    else if (aggressiveFix)
    {
      if (EPpEst < 0)
      {
        front_left_wheel->setLinearVelocity(-.1*WaypointControllerHelper::sign(currMan.distance));  // TODO experiment with 0 and setting to fraction of LvelCmd
        back_left_wheel->setLinearVelocity(-.1*WaypointControllerHelper::sign(currMan.distance));  // in the sim it can just spin in circles (maybe the sim is bad?)
      }
      else
      {
        front_right_wheel->setLinearVelocity(-.1*WaypointControllerHelper::sign(currMan.distance));
        back_right_wheel->setLinearVelocity(-.1*WaypointControllerHelper::sign(currMan.distance));
      }
    }
    return returnStatus;
  }
  else if (time_scooting > 0)
  {
      front_left_wheel->setLinearVelocity(-.1);
      front_right_wheel->setLinearVelocity(-.1);
      back_left_wheel->setLinearVelocity(-.1);
      back_right_wheel->setLinearVelocity(-.1);
      time_scooting -= dt;
      if (time_scooting <= 0)
      {
          time_scooting =0;
      }
  }
  else
  {
    // remember to stop
    this->haltAndAbort();
    return Status::GOALREACHED;  // gotta get out of here
  }
}

void WaypointController::modifyNavQueue2RecoverFromPathError(pose RobotPose, pose manEnd)
{
  // should plan a maneuver to end of blown maneuver
  // and dump the blown one
  waypointWithManeuvers newWaypoint;
  newWaypoint.initialPose = RobotPose;
  newWaypoint.terminalPose = manEnd;
  newWaypoint.mans = WaypointControllerHelper::waypoint2maneuvers(RobotPose, manEnd);

  std::vector<waypointWithManeuvers>::iterator it;
  it = navigationQueue.begin();
  navigationQueue.insert(it, newWaypoint);
  doingManeuver = false;
  currManeuverIndex = 0;
}

void WaypointController::modifyNavQueue2RecoverFromGoalOvershoot()
{
  // should just consider the current maneuver achieved and begin to attempt the next manuever
  doingManeuver = false;
  currManeuverIndex++;
}
