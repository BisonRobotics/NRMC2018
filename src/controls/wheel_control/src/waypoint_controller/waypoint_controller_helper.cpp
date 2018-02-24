#include <waypoint_controller/waypoint_controller_helper.h>
#define _USE_MATH_DEFINES
#include <cmath>
#define CMPERPOINT 5
#define METERPERCM .01f
#define STRAIGHTRADIUS 1000
#define CUTOFFDIST4DOUBLEARC .01
#define CPPEQUALTOL .05

int WaypointControllerHelper::sign(double A)  // MUST BE INT
{
  return (A >= 0 ? 1 : -1);
}

std::pair<pose, pose> WaypointControllerHelper::inputCleaner(pose robotPose, pose waypoint)
{
  std::pair<pose, pose> newPair;
  newPair.first = robotPose;
  newPair.second = waypoint;

  pose Twp = WaypointControllerHelper::transformPoseToRobotCoord(robotPose, waypoint);

  // if waypoint and robot are exactly in line with each other
  // bump the robot a bit
  if (std::abs(Twp.y) < .01)
  {
    newPair.first.x += .02 * cos(robotPose.theta + M_PI_2);
    newPair.first.y += .02 * sin(robotPose.theta + M_PI_2);

    // if they were in a line and pointing the same way
    if (std::abs(Twp.theta) < .01)
    {
      newPair.first.theta = WaypointControllerHelper::anglediff(newPair.first.theta, .01);
      newPair.second.theta = WaypointControllerHelper::anglediff(newPair.second.theta, -.01);
    }

    // if they were in a line and facing the opposite way
    if (std::abs(Twp.theta) > M_PI - .01)
    {
      newPair.first.theta = WaypointControllerHelper::anglediff(newPair.first.theta, .01);
      newPair.second.theta = WaypointControllerHelper::anglediff(newPair.second.theta, -.01);
    }
    return newPair;
  }

  // if the xintercept would be zero
  // make that not by sliding the robot fwd or sideways a bit
  if (tan(Twp.theta) != 0)
  {
    double xintercept = -Twp.y / tan(Twp.theta) + Twp.x;
    if (std::abs(xintercept) < .01)
    {
      newPair.first.x += .02 * cos(robotPose.theta + M_PI_2);
      newPair.first.y += .02 * sin(robotPose.theta + M_PI_2);
      // check xintercept again
      Twp = WaypointControllerHelper::transformPoseToRobotCoord(robotPose, waypoint);
      if (tan(Twp.theta) != 0)
      {
        newPair.first.x += .02 * cos(robotPose.theta) - .02 * cos(robotPose.theta + M_PI_2);
        newPair.first.y += .02 * sin(robotPose.theta) - .02 * sin(robotPose.theta + M_PI_2);
      }
      return newPair;  // return if a correction was made
    }
  }

  // if the waypoint is pointing exactly up or down
  // make sure its not doing that
  if ((std::abs(WaypointControllerHelper::anglediff(M_PI_2, Twp.theta)) < .01) ||
      (std::abs(WaypointControllerHelper::anglediff(-M_PI_2, Twp.theta)) < .01))
  {
    newPair.second.theta = WaypointControllerHelper::anglediff(waypoint.theta, -.01);

    return newPair;
  }

  // if the waypoint and robot are parallel but opposite facing exactly
  // make them not do that
  if (std::abs(Twp.theta) > (M_PI - .01))
  {
    newPair.second.theta = WaypointControllerHelper::anglediff(waypoint.theta, -.01);
    return newPair;
  }

  // if the waypoint and robot are parralel and facing the same way
  // fix that too
  if (std::abs(Twp.theta) < .01)
  {
    newPair.first.theta = WaypointControllerHelper::anglediff(robotPose.theta, -.01);
    return newPair;
  }

  // if there were no problems, return normally
  return newPair;
}

double WaypointControllerHelper::anglediff(double x, double y)
{
  return atan2(sin(x - y), cos(x - y));
}

pose WaypointControllerHelper::transformPoseToRobotCoord(pose robotPose, pose worldPose)
{
  // takes a pose which is in world coordinates and transforms it to robot coords
  // this is used primarily for waypoints
  pose returnPose;
  returnPose.x =
      cos(robotPose.theta) * (worldPose.x - robotPose.x) + sin(robotPose.theta) * (worldPose.y - robotPose.y);
  returnPose.y =
      -sin(robotPose.theta) * (worldPose.x - robotPose.x) + cos(robotPose.theta) * (worldPose.y - robotPose.y);
  returnPose.theta = WaypointControllerHelper::anglediff(worldPose.theta, robotPose.theta);
  return returnPose;
}

pose WaypointControllerHelper::reflectWaypointAroundRobot(pose waypoint, pose robot)
{
  pose returnPose;
  returnPose.theta = WaypointControllerHelper::anglediff(
      robot.theta, WaypointControllerHelper::anglediff(waypoint.theta, robot.theta));

  double m = tan(robot.theta);  // need corner case for if robot angle is +/- 90 deg

  double a = 1;
  double b = -m;
  double c = m * robot.x - robot.y;

  returnPose.x = (waypoint.x * (a * a - b * b) - 2 * b * (a * waypoint.y + c)) / (a * a + b * b);
  returnPose.y = (waypoint.y * (b * b - a * a) - 2 * a * (b * waypoint.x + c)) / (a * a + b * b);
  return returnPose;
}

maneuver WaypointControllerHelper::transformManeuverToWorldCoord(pose robotPose, maneuver myMan)
{
  // takes a pose which is in robot coordinates and transforms it to world coords
  maneuver returnManeuver;
  returnManeuver.xc = cos(robotPose.theta) * myMan.xc - sin(robotPose.theta) * myMan.yc + robotPose.x;
  returnManeuver.yc = sin(robotPose.theta) * myMan.xc + cos(robotPose.theta) * myMan.yc + robotPose.y;
  returnManeuver.radius = myMan.radius;
  returnManeuver.distance = myMan.distance;
  return returnManeuver;
}

std::vector<maneuver> WaypointControllerHelper::oneTurnSolution(pose robotPose, pose waypoint)
{
  maneuver maneuver1, m1UT;  // maneuver and untransformed buddy
  maneuver maneuver2, m2UT;
  std::vector<maneuver> returnVector;

  pose wp;  // waypoint in robot coord, by inverse transform
  wp = WaypointControllerHelper::transformPoseToRobotCoord(robotPose, waypoint);

  double xintercept = -wp.y / tan(wp.theta) + wp.x;
  // if the waypoint were a line extended back,
  // this is where it would intersect on the robot's x axis
  // find which is closer, the robots position to the xintercept, or the final waypoint
  // to the xintercept

  if (xintercept < 0 || sign(wp.theta) != sign(wp.y))  // this solution won't work
  {
    return returnVector;  // return empty vector
  }

  double distancetoendsq = (wp.x - xintercept) * (wp.x - xintercept) + wp.y * wp.y;
  double distancetostartsq = xintercept * xintercept;
  // waypoint is assumed to have been transformed to robot coordinates
  // maneuvers will need transformed back to world coord

  // mostly Austin's work here, this is the turn first, straight later
  if (distancetostartsq < distancetoendsq)  // intersection is closer to start ,turn is first maneuver
  {
    maneuver1.xc = 0;
    maneuver1.yc = tan((M_PI - wp.theta) / 2.0f) * xintercept;
    maneuver1.radius = maneuver1.yc;
    maneuver1.distance = std::abs(wp.theta * maneuver1.radius);

    // now calculate second line as a turn
    // distance is straight line distance from common point to end
    // radius is LARGE
    // center is radius distance away, perpendicular to halfway point between common and end

    // find point common to arc and line
    double xtangent = maneuver1.radius * cos(maneuver1.distance / maneuver1.radius - M_PI_2);
    double ytangent = maneuver1.radius * sin(maneuver1.distance / maneuver1.radius - M_PI_2) + maneuver1.radius;

    maneuver2.radius = STRAIGHTRADIUS;
    maneuver2.distance = sqrt((xtangent - wp.x) * (xtangent - wp.x) + (ytangent - wp.y) * (ytangent - wp.y));

    double xhalf = (wp.x + xtangent) / 2.0f;
    double yhalf = (wp.y + ytangent) / 2.0f;
    maneuver2.xc = xhalf + maneuver2.radius * cos(wp.theta + M_PI_2);
    maneuver2.yc = yhalf + maneuver2.radius * sin(wp.theta + M_PI_2);
  }
  else  // turn is second
  {
    double A = 1;
    double B = (2 * wp.y / tan(wp.theta) - 2 * wp.x);
    double C = wp.x * wp.x - 2 * wp.y * wp.x / tan(wp.theta) - wp.y * wp.y;
    double potxcenter2a, potxcenter2b;
    if (B * B - 4 * A * C < 0)
    {
      // this solution should not have been called if we get to here
      maneuver1.distance = 0;
      maneuver1.radius = 0;
      maneuver1.xc = 0;
      maneuver1.yc = 0;
      // set everything to zero
      maneuver2.xc = 0;
      maneuver2.radius = 0;
      maneuver2.yc = 0;
      maneuver2.distance = 0;
      // this will be checked in waypoint master but should never happen
    }
    else
    {
      potxcenter2a = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
      potxcenter2b = (-B - sqrt(B * B - 4 * A * C)) / (2 * A);

      double xcenter2 = potxcenter2a < potxcenter2b ? potxcenter2a : potxcenter2b;  // pick smallest

      // first maneuver is going straight
      maneuver1.distance = xcenter2;
      maneuver1.radius = STRAIGHTRADIUS;
      maneuver1.xc = xcenter2 / 2.0f;
      maneuver1.yc = STRAIGHTRADIUS;
      // second is the turn
      maneuver2.xc = xcenter2;
      maneuver2.radius = -1.0f / tan(wp.theta) * (maneuver2.xc - wp.x) + wp.y;
      maneuver2.yc = maneuver2.radius;
      maneuver2.distance = std::abs(wp.theta * maneuver2.radius);
    }
  }
  // dont forget to transform back!!
  m1UT = transformManeuverToWorldCoord(robotPose, maneuver1);
  m2UT = transformManeuverToWorldCoord(robotPose, maneuver2);
  returnVector.push_back(m1UT);
  returnVector.push_back(m2UT);
  return returnVector;
}

std::vector<maneuver> WaypointControllerHelper::inverseOneTurnSolution(pose robotPose, pose waypoint)
{
  pose internalwaypoint = {.x = 2 * robotPose.x - waypoint.x,
                           .y = 2 * robotPose.y - waypoint.y,
                           .theta = waypoint.theta };
  std::vector<maneuver> intermediateSolution = oneTurnSolution(robotPose, internalwaypoint);

  intermediateSolution.at(0).distance = -intermediateSolution.at(0).distance;
  intermediateSolution.at(0).radius = -intermediateSolution.at(0).radius;
  intermediateSolution.at(0).xc = 2 * robotPose.x - intermediateSolution.at(0).xc;
  intermediateSolution.at(0).yc = 2 * robotPose.y - intermediateSolution.at(0).yc;

  intermediateSolution.at(1).distance = -intermediateSolution.at(1).distance;
  intermediateSolution.at(1).radius = -intermediateSolution.at(1).radius;
  intermediateSolution.at(1).xc = 2 * robotPose.x - intermediateSolution.at(1).xc;
  intermediateSolution.at(1).yc = 2 * robotPose.y - intermediateSolution.at(1).yc;

  return intermediateSolution;
}

std::vector<maneuver> WaypointControllerHelper::twoTurnSolution(pose robotPose, pose waypoint)
{
  bool beenFlipped = false;
  pose wp;
  // The two turn solver can give two solutions for a given waypoint, one where the robot changes
  // direction throughout the manuever or one where the robot stays going the same direction the
  // entire time. Under certain circumstances one solution is better than the other. So the
  // solving code is fed the problem such that it will yield the desired resultant maneuver.

  // there is either a bug in the flipping or in the distance calculation
  // this conditional is a little paranoid. Possible redundant cases in there.

  if (std::abs(robotPose.theta) <= M_PI_2)
  {
    if (std::tan(robotPose.theta) * (waypoint.x - robotPose.x) + robotPose.y > waypoint.y)
    {
      wp = reflectWaypointAroundRobot(waypoint, robotPose);
      beenFlipped = true;
    }
  }
  else
  {
    if (std::tan(robotPose.theta) * (waypoint.x - robotPose.x) + robotPose.y < waypoint.y)
    {
      wp = reflectWaypointAroundRobot(waypoint, robotPose);
      beenFlipped = true;
    }
  }

  // Now the solver begins, using wp which may or may not have been refelcted/flipped
  wp = WaypointControllerHelper::transformPoseToRobotCoord(robotPose, beenFlipped ? wp : waypoint);
  double cosanglearg = -wp.theta - M_PI_2;
  double sinanglearg = -wp.theta + M_PI_2;

  double A =
      std::cos(cosanglearg) * std::cos(cosanglearg) + (1 + std::sin(sinanglearg)) * (1 + std::sin(sinanglearg)) - 4;
  double B = -2 * wp.x * std::cos(cosanglearg) - 2 * wp.y * (1 + sin(sinanglearg));
  double C = wp.x * wp.x + wp.y * wp.y;

  double d;
  if (A < .0001 && A > -.0001)
    d = -C / B;
  else
  {
    double da = (-B + std::sqrt(B * B - 4.0f * A * C)) / (2.0f * A);
    double db = (-B - std::sqrt(B * B - 4.0f * A * C)) / (2.0f * A);
    d = std::max(da, db);  // max of da and db
  }

  maneuver man1, man2;
  man1.xc = 0;
  man1.yc = d;
  man1.radius = d;

  man2.xc = wp.x - d * std::cos(cosanglearg);
  man2.yc = wp.y - d * std::sin(sinanglearg);
  man2.radius = -d;

  double xintermediate = (man1.xc + man2.xc) / 2.0f;
  double yintermediate = (man1.yc + man2.yc) / 2.0f;

  double theta1 = std::atan2(xintermediate, d - yintermediate);

  man1.distance = d * theta1;
  man2.distance = d * (theta1 - wp.theta);  // should this be an WaypointControllerHelper::anglediff?

  // DONT FORGET TO UNTRANSFORM
  maneuver man1UT = transformManeuverToWorldCoord(robotPose, man1);
  maneuver man2UT = transformManeuverToWorldCoord(robotPose, man2);
  // After solving, if the waypoint was flipped to begin with, some care must be taken

  if (beenFlipped)
  {
    man1UT.radius = -man1UT.radius;
    man2UT.radius = -man2UT.radius;

    pose tempCenter = {.x = man1UT.xc, .y = man1UT.yc, .theta = 0 };
    tempCenter = reflectWaypointAroundRobot(tempCenter, robotPose);
    man1UT.xc = tempCenter.x;
    man1UT.yc = tempCenter.y;

    tempCenter.x = man2UT.xc;
    tempCenter.y = man2UT.yc;
    tempCenter = reflectWaypointAroundRobot(tempCenter, robotPose);
    man2UT.xc = tempCenter.x;
    man2UT.yc = tempCenter.y;
  }

  std::vector<maneuver> returnVector;
  returnVector.push_back(man1UT);
  returnVector.push_back(man2UT);
  return returnVector;
}

std::vector<maneuver> WaypointControllerHelper::waypoint2maneuvers(pose robotPose, pose waypoint)
{
  std::vector<maneuver> myMan;
  // couple different scenarios, credit to Sam Fehringer and Austin Oltmanns

  // clean the input
  std::pair<pose, pose> newPair;
  newPair = inputCleaner(robotPose, waypoint);
  robotPose = newPair.first;
  waypoint = newPair.second;

  // check xintercept to help determine which solution to use
  pose wp;  // waypoint in robot coordinates, by inverse transform
  wp = WaypointControllerHelper::transformPoseToRobotCoord(robotPose, waypoint);

  // if the waypoint were a line extended back,
  // this is where it would intersect on the robot's x axis

  double xintercept = -wp.y / tan(wp.theta) + wp.x;  // sign is set so sign(0) ==1

  if (sign(wp.y) == 1)
  {
    if (sign(xintercept) != sign(wp.theta))
      myMan = twoTurnSolution(robotPose, waypoint);
    else if (sign(xintercept) == 1 && sign(wp.theta) == 1)
      myMan = oneTurnSolution(robotPose, waypoint);
    else if (sign(xintercept) == -1 && sign(wp.theta) == -1)
      myMan = inverseOneTurnSolution(robotPose, waypoint);
  }
  else
  {
    if (sign(xintercept) == sign(wp.theta))
      myMan = twoTurnSolution(robotPose, waypoint);
    else if (sign(xintercept) == 1 && sign(wp.theta) == -1)
      myMan = oneTurnSolution(robotPose, waypoint);
    else if (sign(xintercept) == -1 && sign(wp.theta) == 1)
      myMan = inverseOneTurnSolution(robotPose, waypoint);
  }

  return myMan;
}

pose WaypointControllerHelper::findCPP(pose robotPose, maneuver curManeuver)
{
  // find nearest point on path (in world coords) of maneuver to robot as well as theta of path at that point
  double thetatangent;
  pose CPP;
  thetatangent = atan2(robotPose.y - curManeuver.yc, robotPose.x - curManeuver.xc);
  CPP.theta = WaypointControllerHelper::anglediff(thetatangent, -M_PI_2 * sign(curManeuver.radius));
  // CPP.theta = WaypointControllerHelper::anglediff(CPP.theta, 0);

  CPP.x = std::abs(curManeuver.radius) * cos(thetatangent) + curManeuver.xc;
  CPP.y = std::abs(curManeuver.radius) * sin(thetatangent) + curManeuver.yc;

  return CPP;
}

std::vector<std::pair<double, double> >
WaypointControllerHelper::waypointWithManeuvers2points(waypointWithManeuvers myMan)
{
  // get points from initial point to terminal point by
  // starting with initial point and solving through first maneuver
  // record intermediate point and solve from there through the next maneuver
  // repeat until out of maneuvers

  std::vector<std::pair<double, double> > allPoints;
  std::pair<double, double> aPoint;
  std::pair<double, double> transformPoint;

  pose currPose = myMan.initialPose;

  for (std::vector<maneuver>::iterator it = myMan.mans.begin(); it != myMan.mans.end(); ++it)
  {
    maneuver currMan = *it;  // I mean copy, and thats what this does.
    pose retPose;
    for (double travel = 0; std::abs(travel) <= std::abs(it->distance);
         travel += sign(it->distance) * METERPERCM * CMPERPOINT)
    {
      currMan.distance = travel;
      retPose = endOfManeuver(currPose, currMan);
      transformPoint.first = retPose.x;
      transformPoint.second = retPose.y;
      allPoints.push_back(transformPoint);
    }
    currPose = retPose;
  }

  return allPoints;
}

pose WaypointControllerHelper::endOfManeuver(pose robotPose, maneuver myMan)  // finds the conclusion of a maneuver
                                                                              // which starts at a pose
{
  pose endPoseUT, endPose;
  double NAD = myMan.distance / myMan.radius;

  // untransformed endpose
  endPoseUT.x = myMan.radius * cos(NAD - M_PI_2);
  endPoseUT.y = myMan.radius * sin(NAD - M_PI_2) + myMan.radius;

  endPose.x = cos(robotPose.theta) * endPoseUT.x - sin(robotPose.theta) * endPoseUT.y + robotPose.x;
  endPose.y = sin(robotPose.theta) * endPoseUT.x + cos(robotPose.theta) * endPoseUT.y + robotPose.y;
  endPose.theta = WaypointControllerHelper::anglediff(robotPose.theta, -NAD);

  return endPose;
}

std::pair<double, double> WaypointControllerHelper::speedAndRadius2WheelVels(double speed, double radius,
                                                                             double AxelLen, double maxSpeed)
{
  // positive radius is CCW, first vel is left wheel, second is right
  // speed is average of wheel velocities: TotalVel = .5*(LeftVel + RightVel)
  // Turn Radius = AxelLen/2 * (LeftVel+RightVel)/(RightVel - LeftVel)
  std::pair<double, double> Vels;
  if (radius != 0)  // todo: add min radius? or does max speed take care of any problems?
  {
    Vels.first = ((4 * radius * speed / AxelLen) - 2 * speed) * AxelLen / (radius * 4);
    Vels.second = (2 * speed) - Vels.first;

    // this scales the max wheel speed to the max speed if necessary
    double maxabs = std::max(std::abs(Vels.first), std::abs(Vels.second));
    if (maxabs > maxSpeed)
    {
      double newspeed = speed * maxSpeed / maxabs;  // this then calculates the new scaled speeds
      Vels.first = ((4 * radius * newspeed / AxelLen) - 2 * newspeed) * AxelLen / (radius * 4);
      Vels.second = (2 * newspeed) - Vels.first;
    }
  }
  else
  {
    Vels.first = -.5 * speed;
    Vels.second = .5 * speed;  // if doing min radius, multiply these by +/-sign(radius);
  }

  return Vels;
}
