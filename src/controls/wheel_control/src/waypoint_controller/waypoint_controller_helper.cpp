#include <waypoint_controller/waypoint_controller_helper.h>
#include <cmath>
#define CMPERPOINT 5
#define METERPERCM .01f
#define PI 3.1415926539f
#define SIGN(A) (A > 0 ? 1 : -1)
#define STRAIGHTRADIUS 1000
#define CUTOFFDIST4DOUBLEARC .01
#define CPPEQUALTOL .05

float anglediff(float x, float y) {return atan2(sin(x-y),cos(x-y));}

pose transformPoseToRobotCoord(pose robotPose, pose worldPose)
{
    //takes a pose which is in world coordinates and transforms it to robot coords
    //this is used primarily for waypoints
    pose returnPose;
    returnPose.x = cos(robotPose.theta) * (worldPose.x-robotPose.x) + sin(robotPose.theta) * (worldPose.y - robotPose.y);
    returnPose.y = -sin(robotPose.theta) * (worldPose.x-robotPose.x) + cos(robotPose.theta) * (worldPose.y - robotPose.y);
    returnPose.theta = anglediff(worldPose.theta,robotPose.theta);
    return returnPose;
}

maneuver transformManeuverToWorldCoord(pose robotPose, maneuver myMan)
{
    //takes a pose which is in robot coordinates and transforms it to world coords
    maneuver returnManeuver;
    returnManeuver.xc = cos(robotPose.theta) *myMan.xc - sin(robotPose.theta) * myMan.yc + robotPose.x;
    returnManeuver.yc = sin(robotPose.theta) *myMan.xc + cos(robotPose.theta) * myMan.yc + robotPose.y;
    returnManeuver.radius = myMan.radius;
    returnManeuver.distance = myMan.distance;
    return returnManeuver;
}

std::vector<maneuver> oneTurnSolution(pose robotPose, pose waypoint)
{
    maneuver maneuver1, m1UT; //maneuver and untransformed buddy
    maneuver maneuver2, m2UT;
    std::vector<maneuver> returnVector;

    pose wp;  // waypoint in robot coord, by inverse transform
    wp = transformPoseToRobotCoord(robotPose, waypoint);

    float xintercept = -wp.y / tan(wp.theta) + wp.x;
    // if the waypoint were a line extended back,
    // this is where it would intersect on the robot's x axis
    // find which is closer, the robots position to the xintercept, or the final waypoint
    // to the xintercept

    if (xintercept <0 || SIGN(wp.theta) != SIGN(wp.y) ) //this solution won't work
    {
        return returnVector; //return empty vector
    }

    float distancetoendsq = (wp.x - xintercept) * (wp.x - xintercept) + wp.y * wp.y;
    float distancetostartsq = xintercept * xintercept;
    //waypoint is assumed to have been transformed to robot coordinates
    //maneuvers will need transformed back to world coord

    // mostly Austin's work here, this is the turn first, straight later
    if (distancetostartsq < distancetoendsq)  // intersection is closer to start ,turn is first maneuver
    {
        maneuver1.xc = 0;
        maneuver1.yc = tan((PI - wp.theta) / 2) * xintercept;
        maneuver1.radius = maneuver1.yc;
        maneuver1.distance = std::abs(wp.theta * maneuver1.radius);

        // now calculate second line as a turn
        // distance is straight line distance from common point to end
        // radius is LARGE
        // center is radius distance away, perpendicular to halfway point between common and end

        //find point common to arc and line
        float xtangent = maneuver1.radius * cos(maneuver1.distance/maneuver1.radius - PI/2);
        float ytangent = maneuver1.radius * sin(maneuver1.distance/maneuver1.radius - PI/2) + maneuver1.radius;

        maneuver2.radius = STRAIGHTRADIUS;
        maneuver2.distance = sqrt((xtangent - wp.x) * (xtangent - wp.x) + (ytangent - wp.y) * (ytangent - wp.y));

        float xhalf = (wp.x + xtangent) / 2;
        float yhalf = (wp.y + ytangent) / 2;
        //this below could be correct
        float M = -1/tan(wp.theta);
        maneuver2.xc = xhalf + sqrt(STRAIGHTRADIUS * STRAIGHTRADIUS /(M*M+1) );
        maneuver2.yc =  M * (maneuver2.xc - xhalf) + yhalf;
    }
    else //turn is second
    {
        float A = 1;
        float B = (2 * wp.y / tan(wp.theta) - 2 * wp.x);
        float C = wp.x * wp.x - 2 * wp.y * wp.x/ tan(wp.theta) - wp.y * wp.y;
        float potxcenter2a, potxcenter2b;
        if (B * B - 4 * A * C < 0)
        {
            // gaahh
        }
        else
        {
            potxcenter2a = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
            potxcenter2b = (-B - sqrt(B * B - 4 * A * C)) / (2 * A);
        }
        // int a = 1,b=3;
        // float breakall = a/(b-3);
        float xcenter2 = potxcenter2a < potxcenter2b ? potxcenter2a : potxcenter2b;  // pick smallest

        // first maneuver is going straight
        maneuver1.distance = xcenter2;
        maneuver1.radius = STRAIGHTRADIUS;
        maneuver1.xc = xcenter2 / 2;
        maneuver1.yc = STRAIGHTRADIUS;
        // need to check this
        maneuver2.xc = xcenter2;
        maneuver2.radius = -1 / tan(wp.theta) * (maneuver2.xc - wp.x) + wp.y;
        maneuver2.yc = maneuver2.radius;
        maneuver2.distance = std::abs(wp.theta * maneuver2.radius);
    }
    //dont forget to transform back!!
    m1UT = transformManeuverToWorldCoord(robotPose, maneuver1);
    m2UT = transformManeuverToWorldCoord(robotPose, maneuver2);
    returnVector.push_back(m1UT);
    returnVector.push_back(m2UT);
    return returnVector;
}

std::vector<maneuver> doubleArcSolution(pose robotPose, pose waypoint)
{
// Double Arc, no linear displacement - credit Sam Fehringer
    // Some of the conditions may be redundant, but this is a catch all for when the other solutions don't work
    // only two maneuvers
    // solve based on a quadratic equation
    maneuver maneuver1, m1UT; //maneuver and untransformed buddy
    maneuver maneuver2, m2UT;
    std::vector<maneuver> returnVector;

    pose wp;  // waypoint in robot coord, by inverse transform
    wp = transformPoseToRobotCoord(robotPose, waypoint);

    float A = 2 * (1 - sin(PI / 2 - wp.theta));
    float B = 2 * (wp.x * cos(PI / 2 - wp.theta) + wp.y * (1 + sin(PI / 2 - wp.theta)));
    float C = -(wp.x * wp.x - wp.y * wp.y);

    // find roots of quadratic
    // first check for negative part under radical
    float potRa, potRb;
    if (B * B - 4 * A * C < 0)
    {
      // gaaahhh if this happens, shit is not real
    }
    else
    {
      potRa = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
      potRb = (-B - sqrt(B * B - 4 * A * C)) / (2 * A);
    }
    float r = potRa > potRb ? potRa : potRb;

    maneuver1.radius = r;
    maneuver1.xc = 0;
    maneuver1.yc = r;

    maneuver2.radius = r;
    maneuver2.xc = wp.x - r * cos((PI / 2) - wp.theta);
    maneuver2.yc = wp.y - r * sin((PI / 2) - wp.theta);

    maneuver1.distance = (PI / 2) + atan2(maneuver2.yc - r, maneuver2.xc);
    maneuver2.distance = (PI / 2) + atan2(maneuver2.yc - r, maneuver2.xc) + wp.theta;

    m1UT = transformManeuverToWorldCoord(robotPose, maneuver1);
    m2UT = transformManeuverToWorldCoord(robotPose, maneuver2);

    returnVector.push_back(m1UT);
    returnVector.push_back(m2UT);

    return returnVector;
}

std::vector<maneuver> doubleArcWithLinearSolution(pose robotPose, pose waypoint)
{
    // DoubleArc w/ linear Displacement - credit due to Sam Fehringer
    // if the intercept is behind the robot and the waypoint is in front of the robot
    // or if the intercept is ahead of the robot, but the waypoint is behind the robot
    // then we are performing a "DoubleArc - lineaer displacement"
    // 3 maneuvers, fisrt a straight segment, then two stacked circles, "like a snowman"
    // this is for waypoints which are extreme in the robot Y and facing in the same direction
    maneuver maneuver1, m1UT; //maneuver and untransformed buddy
    maneuver maneuver2, m2UT;
    maneuver maneuver3, m3UT;
    std::vector<maneuver> returnVector;

    pose wp;  // waypoint in robot coord, by inverse transform
    wp = transformPoseToRobotCoord(robotPose, waypoint);

    // calculate the radii and x-axis displacement
    float r = std::abs(wp.y) / (3 + cos(wp.theta));  // what is this 3?
    float firstlinear = wp.x + sin(wp.theta) * r;    // displacement from origin

    // data about the maneuvers can now be known
    maneuver1.xc = firstlinear / 2;
    maneuver1.yc = STRAIGHTRADIUS;
    maneuver1.radius = STRAIGHTRADIUS;
    maneuver1.distance = firstlinear;

    maneuver2.xc = firstlinear;
    maneuver2.yc = SIGN(wp.y) * r;
    maneuver2.radius = r;
    maneuver2.distance = PI * r;

    maneuver3.xc = firstlinear;
    maneuver3.yc = SIGN(wp.y) * 3 * r;  // the ycenter is 3 radii from the x axis
    maneuver3.radius = r;
    maneuver3.distance = (wp.theta + PI / 2) * r;

    //transform back to world coord
    m1UT = transformManeuverToWorldCoord(robotPose, maneuver1);
    m2UT = transformManeuverToWorldCoord(robotPose, maneuver2);
    m3UT = transformManeuverToWorldCoord(robotPose, maneuver3);

    returnVector.push_back(m1UT);
    returnVector.push_back(m2UT);
    returnVector.push_back(m3UT);

    return returnVector;
}

std::vector<maneuver> waypoint2maneuvers(pose robotPose, pose waypoint)
{
  std::vector<maneuver> myMan;
  // couple different scenarios, most credit to Sam Fehringer

  pose wp;  // waypoint in robot coordinates, by inverse transform
  wp = transformPoseToRobotCoord(robotPose, waypoint);

  float xintercept = -wp.y / tan(wp.theta) + wp.x;
  // if the waypoint were a line extended back,
  // this is where it would intersect on the robot's x axis

  // find which is closer, the robots position to the x-intercept, or the final waypoint
  // to the x-intercept. This will determine which strategy to use
  float distancetoendsq = (wp.x - xintercept) * (wp.x - xintercept) + wp.y * wp.y;
  float distancetostartsq = xintercept * xintercept;

  if (xintercept>=0 && SIGN(wp.theta) == SIGN(wp.y))
  {
    myMan = oneTurnSolution(robotPose,waypoint);
  }
  else //should do a double arc here
  {
      //but for right now, its unimplemented
  }

  /*
  if ((xintercept < 0 && wp.x > 0) || (xintercept > 0 && wp.x < 0))
  {
    myMan = doubleArcWithLinearSolution(robotPose, waypoint);
  }


  else if ((wp.y > 0 && wp.theta <= 0) || (wp.y < 0 && wp.theta >= 0) || (distancetostartsq < CUTOFFDIST4DOUBLEARC) ||
           (wp.x > 0 && xintercept < 0) || (wp.x < 0 && xintercept > 0) || (wp.x < 0 && wp.theta >= 0 && wp.y > 0) ||
           (wp.x < 0 && wp.theta <= 0 && wp.y < 0))
  {
    myMan = doubleArcSolution(robotPose, waypoint);
  }


  else if ((wp.y > 0 && wp.x > 0 && wp.theta > 0 && wp.theta < PI) ||
           (wp.y < 0 && wp.x > 0 && wp.theta < 0 && wp.theta > -PI))
  {
    myMan = oneTurnSolution(robotPose,waypoint);
  }
*/

  return myMan;
}

pose findCPP(pose robotPose, maneuver curManeuver)
{
  // find nearest point on path of maneuver to robot as well as theta of path at that point
  float Xp, Xpa, Xpb, Yp, Ypa, Ypb, M;
  pose CPP;
  if (std::abs(robotPose.x -curManeuver.xc) < CPPEQUALTOL)  // dont want to divide by zero
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
    //this is a shitfest
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
  CPP.theta = anglediff(atan2(Yp - curManeuver.yc, Xp - curManeuver.xc), PI / 2); //this needs fixed
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
      aPoint.first = (it->radius * cos(dTheta - PI / 2));
      aPoint.second = (it->radius * sin(dTheta - PI / 2)) + it->radius;
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
  //if there are no points, there were no maneuvers
  return allPoints;
}

pose endOfManeuver(pose robotPose, maneuver myMan) //finds the conclusion of a maneuver which starts at a pose
{
  pose endPoseUT, endPose;
  float NAD = myMan.distance/myMan.radius;

  //untransformed endpose
  endPoseUT.x = myMan.radius * cos(NAD - PI/2);
  endPoseUT.y = myMan.radius * sin(NAD - PI/2) + myMan.radius;

  endPose.x = cos(robotPose.theta) * endPoseUT.x - sin(robotPose.theta) * endPoseUT.y + robotPose.x;
  endPose.y = sin(robotPose.theta) * endPoseUT.x + cos(robotPose.theta) * endPoseUT.y + robotPose.y;
  endPose.theta = robotPose.theta + NAD; //TODO:: should check to make sure nothing went out of [-pi,pi]

  return endPose;
}

std::pair<float, float> speedAndRadius2WheelVels(float speed, float radius, float AxelLen, float maxSpeed)
{
  //positive radius is CCW, first vel is left wheel, second is right
  //speed is average of wheel velocities: TotalVel = .5*(LeftVel + RightVel)
  //Turn Radius = AxelLen/2 * (LeftVel+RightVel)/(RightVel - LeftVel)
  std::pair<float,float> Vels;
  float Sfactor, S2factor;
  if (radius != 0)
  {
    //Vels.first = (1 + (AxelLen/2 + radius)/(radius - AxelLen/2))/(2*speed);
    Vels.first = (4*radius*speed/(AxelLen) - 2*speed)*AxelLen/(radius*4);
    Sfactor =1; //comment out these two lines and watch the magic!
    S2factor =1;//is it some crazy scope thing?
    if (std::abs(Vels.first) > maxSpeed)
    {
        Sfactor = Vels.first/maxSpeed;
        Vels.first = Vels.first / Sfactor;
    }

    Vels.second = 2*(speed/Sfactor) - Vels.first;

    if (std::abs(Vels.second) > maxSpeed)
    {
        S2factor = Vels.second/maxSpeed;
        Vels.second = Vels.second/S2factor;
        Vels.first = Vels.first/S2factor;
    }
  }
  else
  {
      Vels.first = -.5*speed;
      Vels.second = .5*speed;
  }

  return Vels;
}
