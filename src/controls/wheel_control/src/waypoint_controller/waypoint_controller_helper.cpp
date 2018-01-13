#include <waypoint_controller/waypoint_controller_helper.h>
#define _USE_MATH_DEFINES
#include <cmath>
#define CMPERPOINT 5
#define METERPERCM .01f
#define SIGN(A) (A >= 0 ? 1 : -1)
#define STRAIGHTRADIUS 1000
#define CUTOFFDIST4DOUBLEARC .01
#define CPPEQUALTOL .05

float anglediff(float x, float y)
{
  return atan2(sin(x - y), cos(x - y));
}

pose transformPoseToRobotCoord(pose robotPose, pose worldPose)
{
  // takes a pose which is in world coordinates and transforms it to robot coords
  // this is used primarily for waypoints
  pose returnPose;
  returnPose.x =
      cos(robotPose.theta) * (worldPose.x - robotPose.x) + sin(robotPose.theta) * (worldPose.y - robotPose.y);
  returnPose.y =
      -sin(robotPose.theta) * (worldPose.x - robotPose.x) + cos(robotPose.theta) * (worldPose.y - robotPose.y);
  returnPose.theta = anglediff(worldPose.theta, robotPose.theta);
  return returnPose;
}

pose reflectWaypointAroundRobot(pose waypoint, pose robot)
{
  pose returnPose;
  returnPose.theta = anglediff(robot.theta, anglediff(waypoint.theta, robot.theta));

  float m = tan(robot.theta);  // need corner case for if robot angle is +/- 90 deg

  float a = 1;
  float b = -m;
  float c = m * robot.x - robot.y;

  returnPose.x = (waypoint.x * (a * a - b * b) - 2 * b * (a * waypoint.y + c)) / (a * a + b * b);
  returnPose.y = (waypoint.y * (b * b - a * a) - 2 * a * (b * waypoint.x + c)) / (a * a + b * b);
  return returnPose;
}

maneuver transformManeuverToWorldCoord(pose robotPose, maneuver myMan)
{
  // takes a pose which is in robot coordinates and transforms it to world coords
  maneuver returnManeuver;
  returnManeuver.xc = cos(robotPose.theta) * myMan.xc - sin(robotPose.theta) * myMan.yc + robotPose.x;
  returnManeuver.yc = sin(robotPose.theta) * myMan.xc + cos(robotPose.theta) * myMan.yc + robotPose.y;
  returnManeuver.radius = myMan.radius;
  returnManeuver.distance = myMan.distance;
  return returnManeuver;
}

std::vector<maneuver> oneTurnSolution(pose robotPose, pose waypoint)
{
  maneuver maneuver1, m1UT;  // maneuver and untransformed buddy
  maneuver maneuver2, m2UT;
  std::vector<maneuver> returnVector;

  pose wp;  // waypoint in robot coord, by inverse transform
  wp = transformPoseToRobotCoord(robotPose, waypoint);

  float xintercept = -wp.y / tan(wp.theta) + wp.x;
  // if the waypoint were a line extended back,
  // this is where it would intersect on the robot's x axis
  // find which is closer, the robots position to the xintercept, or the final waypoint
  // to the xintercept

  if (xintercept < 0 || SIGN(wp.theta) != SIGN(wp.y))  // this solution won't work
  {
    return returnVector;  // return empty vector
  }

  float distancetoendsq = (wp.x - xintercept) * (wp.x - xintercept) + wp.y * wp.y;
  float distancetostartsq = xintercept * xintercept;
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
    float xtangent = maneuver1.radius * cos(maneuver1.distance / maneuver1.radius - M_PI_2);
    float ytangent = maneuver1.radius * sin(maneuver1.distance / maneuver1.radius - M_PI_2) + maneuver1.radius;

    maneuver2.radius = STRAIGHTRADIUS;
    maneuver2.distance = sqrt((xtangent - wp.x) * (xtangent - wp.x) + (ytangent - wp.y) * (ytangent - wp.y));

    float xhalf = (wp.x + xtangent) / 2.0f;
    float yhalf = (wp.y + ytangent) / 2.0f;
    // this below could be correct
    float M = tan(wp.theta);
    maneuver2.xc = xhalf + sqrt(STRAIGHTRADIUS * STRAIGHTRADIUS / (M * M + 1));
    maneuver2.yc = (-1.0f/M) * (maneuver2.xc - xhalf) + yhalf;
  }
  else  // turn is second
  {
    float A = 1;
    float B = (2 * wp.y / tan(wp.theta) - 2 * wp.x);
    float C = wp.x * wp.x - 2 * wp.y * wp.x / tan(wp.theta) - wp.y * wp.y;
    float potxcenter2a, potxcenter2b;
    if (B * B - 4 * A * C < 0)
    {
      // gaahh //this solution should not have been called if we get to here
      maneuver1.distance = 0;
      maneuver1.radius = 0;
      maneuver1.xc = 0;
      maneuver1.yc = 0;
      // set everything to zero
      maneuver2.xc = 0;
      maneuver2.radius = 0;
      maneuver2.yc = 0;
      maneuver2.distance = 0;
    }
    else
    {
      potxcenter2a = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
      potxcenter2b = (-B - sqrt(B * B - 4 * A * C)) / (2 * A);
      
      float xcenter2 = potxcenter2a < potxcenter2b ? potxcenter2a : potxcenter2b;  // pick smallest

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

std::vector<maneuver> inverseOneTurnSolution(pose robotPose, pose waypoint)
{
   pose internalwaypoint = {.x = 2*robotPose.x - waypoint.x, .y = 2*robotPose.y - waypoint.y, .theta = waypoint.theta};
   std::vector<maneuver> intermediateSolution = oneTurnSolution(robotPose, internalwaypoint);

   intermediateSolution.at(0).distance = -intermediateSolution.at(0).distance;
   intermediateSolution.at(0).radius   = -intermediateSolution.at(0).radius;
   intermediateSolution.at(0).xc       = 2*robotPose.x - intermediateSolution.at(0).xc;
   intermediateSolution.at(0).yc       = 2*robotPose.y - intermediateSolution.at(0).yc;

   intermediateSolution.at(1).distance = -intermediateSolution.at(1).distance;
   intermediateSolution.at(1).radius   = -intermediateSolution.at(1).radius;
   intermediateSolution.at(1).xc       = 2*robotPose.x - intermediateSolution.at(1).xc;
   intermediateSolution.at(1).yc       = 2*robotPose.y - intermediateSolution.at(1).yc;

   return intermediateSolution;
}

std::vector<maneuver> twoTurnSolution(pose robotPose, pose waypoint)
{
  
  bool beenFlipped = false;
  pose wp;
 //The two turn solver can give two solutions for a given waypoint, one where the robot changes
 //direction throughout the manuever or one where the robot stays going the same direction the 
 //entire time. Under certain circumstances one solution is better than the other. So the 
 //solving code is fed the problem such that it will yield the desired resultant maneuver.

//there is either a bug in the flipping or in the distance calculation
//this conditional is a little paranoid. Possible redundant cases in there.


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

//Now the solver begins, using wp which may or may not have been refelcted/flipped
  wp = transformPoseToRobotCoord(robotPose, beenFlipped ? wp : waypoint);
  double cosanglearg = -wp.theta - M_PI_2;
  double sinanglearg = -wp.theta + M_PI_2;

  double A = std::cos(cosanglearg)*std::cos(cosanglearg) 
                   + (1+std::sin(sinanglearg)) * (1+std::sin(sinanglearg)) - 4;
  double B = -2*wp.x*std::cos(cosanglearg) - 2*wp.y*(1+sin(sinanglearg));
  double C = wp.x*wp.x + wp.y*wp.y;
 
  double d;
  if (A < .0001 && A > -.0001) d = -C/B;
  else
  {
    double da = (-B + std::sqrt(B*B - 4.0f*A*C))/(2.0f*A);
    double db = (-B - std::sqrt(B*B - 4.0f*A*C))/(2.0f*A);
    d  = da > db ? da : db; //max of da and db
  }

  maneuver man1, man2;
  man1.xc = 0;
  man1.yc = d;
  man1.radius = d;
  
  man2.xc = wp.x - d*std::cos(cosanglearg);
  man2.yc = wp.y - d*std::sin(sinanglearg);
  man2.radius = -d;

  double xintermediate = (man1.xc + man2.xc) / 2.0f;
  double yintermediate = (man1.yc + man2.yc) / 2.0f;

  float theta1 = std::atan2(xintermediate, d - yintermediate);
 
  man1.distance = d*theta1;
  man2.distance = d*(theta1 - wp.theta); //should this be an anglediff?

  //DONT FORGET TO UNTRANSFORM
  maneuver man1UT = transformManeuverToWorldCoord(robotPose, man1);
  maneuver man2UT = transformManeuverToWorldCoord(robotPose, man2);
//After solving, if the waypoint was flipped to begin with, some care must be taken

  if (beenFlipped)
  {
     man1UT.radius = -man1UT.radius;
     man2UT.radius = -man2UT.radius;

     pose tempCenter = {.x = man1UT.xc, .y = man1UT.yc, .theta =0};
     tempCenter = reflectWaypointAroundRobot(tempCenter, robotPose);
     man1UT.xc = tempCenter.x;
     man1UT.yc = tempCenter.y;

     tempCenter.x = man2UT.xc; tempCenter.y = man2UT.yc;
     tempCenter = reflectWaypointAroundRobot(tempCenter, robotPose);
     man2UT.xc = tempCenter.x;
     man2UT.yc = tempCenter.y;
  }

  std::vector<maneuver> returnVector;
  returnVector.push_back(man1UT);
  returnVector.push_back(man2UT);
  return returnVector;


}


std::vector<maneuver> waypoint2maneuvers(pose robotPose, pose waypoint)
{
  std::vector<maneuver> myMan;
  // couple different scenarios, credit to Sam Fehringer and Austin Oltmanns

  pose wp;  // waypoint in robot coordinates, by inverse transform
  wp = transformPoseToRobotCoord(robotPose, waypoint);

  if (std::abs(wp.theta) < .001) wp.theta = .0001; //avoid division by zero

  double xintercept = -wp.y / tan(wp.theta) + wp.x; //sign is set so SIGN(0) ==1

  // if the waypoint were a line extended back,
  // this is where it would intersect on the robot's x axis
/*
  if (std::abs(wp.theta) < .001) 
  {
     wp.theta = .0001; //avoid division by zero, only care about sign of this value
     waypoint.theta += .001; //avoid division by zero
  }
*/
  if (SIGN(wp.y) ==1)
  {
    if (SIGN(xintercept) != SIGN(wp.theta))
         myMan = twoTurnSolution(robotPose, waypoint);
    else if (SIGN(xintercept) ==1 && SIGN(wp.theta) ==1)
         myMan = oneTurnSolution(robotPose, waypoint);
    else if (SIGN(xintercept) == -1 && SIGN(wp.theta) ==-1)
         myMan = inverseOneTurnSolution(robotPose, waypoint);
  }
  else 
  { 
    if (SIGN(xintercept) == SIGN(wp.theta))
         myMan = twoTurnSolution(robotPose, waypoint);
    else if (SIGN(xintercept) == 1 && SIGN(wp.theta) == -1)
         myMan = oneTurnSolution(robotPose, waypoint);
    else if (SIGN(xintercept) == -1 && SIGN(wp.theta) ==1)
         myMan = inverseOneTurnSolution(robotPose, waypoint);
  }

  return myMan;
}

pose findCPP(pose robotPose, maneuver curManeuver)
{
  // find nearest point on path of maneuver to robot as well as theta of path at that point
  float Xp, Xpa, Xpb, Yp, Ypa, Ypb, M;
  pose CPP;
  if (std::abs(robotPose.x - curManeuver.xc) < CPPEQUALTOL)  // dont want to divide by zero
  {
    // Xp is the same, Yp is the closest y coord
    Xp = curManeuver.xc;
    Ypa = curManeuver.yc + curManeuver.radius;
    Ypb = curManeuver.yc - curManeuver.radius;
    if (std::abs(Ypa - robotPose.y) < std::abs(Ypb - robotPose.y))
      Yp = Ypa;
    else
      Yp = Ypb;
  }
  else
  {
    M = (curManeuver.yc - robotPose.y) / (curManeuver.xc - robotPose.x);
    // this is a shitfest
    Xpa =
        (-(-2 * curManeuver.xc - 2 * M * M * curManeuver.xc) +
         sqrt((-2 * curManeuver.xc - 2 * M * M * curManeuver.xc) * (-2 * curManeuver.xc - 2 * M * M * curManeuver.xc) -
              4 * (1 + M * M) * (curManeuver.xc * curManeuver.xc - curManeuver.radius * curManeuver.radius +
                                 M * M * curManeuver.xc * curManeuver.xc))) /
        (2 * (1 + M * M));

    Xpb =
        (-(-2 * curManeuver.xc - 2 * M * M * curManeuver.xc) -
         sqrt((-2 * curManeuver.xc - 2 * M * M * curManeuver.xc) * (-2 * curManeuver.xc - 2 * M * M * curManeuver.xc) -
              4 * (1 + M * M) * (curManeuver.xc * curManeuver.xc - curManeuver.radius * curManeuver.radius +
                                 M * M * curManeuver.xc * curManeuver.xc))) /
        (2 * (1 + M * M));

    if (std::abs(robotPose.x - Xpa) < std::abs(robotPose.x - Xpb))
      Xp = Xpa;
    else
      Xp = Xpb;
    Yp = M * (Xp - curManeuver.xc) + curManeuver.yc;
  }

  CPP.x = Xp;
  CPP.y = Yp;
  CPP.theta = anglediff(atan2(Yp - curManeuver.yc, Xp - curManeuver.xc), M_PI / 2);  // this needs fixed
  return CPP;
}

std::vector<std::pair<float, float> > waypointWithManeuvers2points(waypointWithManeuvers myMan)
{
  // get points from initial point to terminal point by
  // starting with initial point and solving through first maneuver
  // record intermediate point and solve from there through the next maneuver
  // repeat until out of maneuvers

  std::vector<std::pair<float, float> > allPoints;
  std::pair<float, float> aPoint;
  std::pair<float, float> transformPoint;

  pose currPoint = myMan.initialPose;

  for (std::vector<maneuver>::iterator it = myMan.mans.begin(); it != myMan.mans.end(); ++it)
  {
    // go from currPoint through manuever
    // get arc and transform to currPoint
    //
    // find the Net Arc Distance (NAD) in radians and each point should be
    // CMPERPOINT away along the arc
    // compute the untransformed point on the arc, then transform to robots position
    float NAD = it->distance / it->radius;
    float radPerMeter = NAD / it->distance;
    float radPerPoint = radPerMeter * METERPERCM * CMPERPOINT;
    for (float dTheta = 0; dTheta <= NAD; dTheta += radPerPoint)
    {
      // this could be optimized by only computing the arc which starts at the
      // proper angle, then translating it to the correct place
      aPoint.first = (it->radius * cos(dTheta - M_PI / 2));
      aPoint.second = (it->radius * sin(dTheta - M_PI / 2)) + it->radius;
      // transform point
      transformPoint.first = cos(currPoint.theta) * aPoint.first - sin(currPoint.theta) * aPoint.second + currPoint.x;
      transformPoint.second = sin(currPoint.theta) * aPoint.first + cos(currPoint.theta) * aPoint.second + currPoint.y;
      allPoints.push_back(transformPoint);
    }
    // went through maneuver, get ready for next maneuver
    currPoint.x = transformPoint.first;
    currPoint.y = transformPoint.second;
    currPoint.theta += NAD;
  }
  // if there are no points, there were no maneuvers
  return allPoints;
}

pose endOfManeuver(pose robotPose, maneuver myMan)  // finds the conclusion of a maneuver which starts at a pose
{
  pose endPoseUT, endPose;
  float NAD = myMan.distance / myMan.radius;

  // untransformed endpose
  endPoseUT.x = myMan.radius * cos(NAD - M_PI / 2);
  endPoseUT.y = myMan.radius * sin(NAD - M_PI / 2) + myMan.radius;

  endPose.x = cos(robotPose.theta) * endPoseUT.x - sin(robotPose.theta) * endPoseUT.y + robotPose.x;
  endPose.y = sin(robotPose.theta) * endPoseUT.x + cos(robotPose.theta) * endPoseUT.y + robotPose.y;
  endPose.theta = robotPose.theta + NAD;  // TODO:: should check to make sure nothing went out of [-pi,pi]

  return endPose;
}

std::pair<float, float> speedAndRadius2WheelVels(float speed, float radius, float AxelLen, float maxSpeed)
{
  // positive radius is CCW, first vel is left wheel, second is right
  // speed is average of wheel velocities: TotalVel = .5*(LeftVel + RightVel)
  // Turn Radius = AxelLen/2 * (LeftVel+RightVel)/(RightVel - LeftVel)
  std::pair<float, float> Vels;
  float Sfactor, S2factor;
  if (radius != 0)
  {
    // Vels.first = (1 + (AxelLen/2 + radius)/(radius - AxelLen/2))/(2*speed);
    Vels.first = (4 * radius * speed / (AxelLen)-2 * speed) * AxelLen / (radius * 4);
    Sfactor = 1;  // comment out these two lines and watch the magic!
    S2factor = 1;  // is it some crazy scope thing?
    if (std::abs(Vels.first) > maxSpeed)
    {
      Sfactor = Vels.first / maxSpeed;
      Vels.first = Vels.first / Sfactor;
    }

    Vels.second = 2 * (speed / Sfactor) - Vels.first;

    if (std::abs(Vels.second) > maxSpeed)
    {
      S2factor = Vels.second / maxSpeed;
      Vels.second = Vels.second / S2factor;
      Vels.first = Vels.first / S2factor;
    }
  }
  else
  {
    Vels.first = -.5 * speed;
    Vels.second = .5 * speed;
  }

  return Vels;
}
