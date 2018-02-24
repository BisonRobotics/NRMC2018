#include <gtest/gtest.h>
#include <waypoint_controller/waypoint_controller_helper.h>
#include <vector>
#include <utility>
#define _USE_MATH_DEFINES
#include <cmath>

#define WAYPOINT2MANEUVERTOL .05
#define SPEEDNRADIUSTOL .001

using namespace WaypointControllerHelper;

TEST(WaypointControllerHelperTests, anglediffWorks1)
{
  double angle1, angle2, result;
  angle1 = 2.0;
  angle2 = -2.0;
  result = -2.283;
  EXPECT_NEAR(anglediff(angle1, angle2), result, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, anglediffWorks2)
{
  double angle1, angle2, result;
  angle1 = 0;
  angle2 = 4;
  result = 2.283;
  EXPECT_NEAR(anglediff(angle1, angle2), result, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, signReturnsInt)
{
  int testnumber1 = 420;
  int result1 = 1;
  EXPECT_TRUE(sign(testnumber1) == result1);
}

TEST(WaypointControllerHelperTests, signReturnsInt2)
{
  int testnumber1 = 420;
  EXPECT_TRUE(sign(testnumber1) == 1);
}

TEST(WaypointControllerHelperTests, signReturnsInt3)
{
  int testnumber1 = -69;
  EXPECT_TRUE(sign(testnumber1) == -1);
}

TEST(WaypointControllerHelperTests, signReturnsInt4)
{
  int testnumber1 = -69;
  int result1 = -1;
  EXPECT_TRUE(sign(testnumber1) == result1);
}

TEST(WaypointControllerHelperTests, signReturnsInt5)
{
  int testnumber1 = 0;
  EXPECT_TRUE(sign(testnumber1) == 1);
}

TEST(WaypointControllerHelperTests, signReturnsInt6)
{
  int testnumber1 = 0;
  int result1 = 1;
  EXPECT_TRUE(sign(testnumber1) == result1);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint1)
{
  pose waypoint = {.x = 0, .y = -1, .theta = M_PI / 3.0f };
  pose robotpose = {.x = 0, .y = 0, .theta = 0 };
  pose expectedPose = {.x = 0, .y = 1, .theta = -M_PI / 3.0f };
  pose returnPose;
  returnPose = reflectWaypointAroundRobot(waypoint, robotpose);

  EXPECT_NEAR(returnPose.x, expectedPose.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, expectedPose.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.theta, expectedPose.theta, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypointWithManeuvers2PointsReturnsEnoughPoints)
{
  maneuver myMan1 = {.radius = 100, .xc = .29289, .yc = 100, .distance = .5858f };
  maneuver myMan2 = {.radius = 3.4142, .xc = .5858, .yc = 3.4142, .distance = 2.6815 };
  pose initialPose = {.x = 0, .y = 0, .theta = 0 };
  pose waypoint = {.x = 3, .y = 1, .theta = M_PI / 4 };

  std::vector<std::pair<double, double> > points;
  std::vector<maneuver> myMans;
  myMans.push_back(myMan1);
  myMans.push_back(myMan2);

  waypointWithManeuvers wpWithMan = {.initialPose = initialPose, .terminalPose = waypoint, .mans = myMans };

  points = waypointWithManeuvers2points(wpWithMan);

  EXPECT_TRUE(points.size() > (myMan1.distance + myMan2.distance) * 10)
      << "Expected at least " << (myMan1.distance + myMan2.distance) * 10 << " points, but got "
      << points.size();  // point at least every 10cm
}

TEST(WaypointControllerHelperTests, speedAndRadius2WheelVelsTests_fwdleft)
{
  std::pair<double, double> returnSpeeds;
  std::pair<double, double> expectedSpeeds;
  double AxelLen = .5f;
  double maxSpeed = .5f;
  double turnRadius = 1.0f;
  double speed = .3f;
  expectedSpeeds.first = .2250f;
  expectedSpeeds.second = .3750f;

  returnSpeeds = speedAndRadius2WheelVels(speed, turnRadius, AxelLen, maxSpeed);

  EXPECT_NEAR(returnSpeeds.first, expectedSpeeds.first, SPEEDNRADIUSTOL);
  EXPECT_NEAR(returnSpeeds.second, expectedSpeeds.second, SPEEDNRADIUSTOL);
}

TEST(WaypointControllerHelperTests, speedAndRadius2WheelVelsTests_bkwdright)
{
  std::pair<double, double> returnSpeeds;
  std::pair<double, double> expectedSpeeds;
  double AxelLen = .5f;
  double maxSpeed = .5f;
  double turnRadius = 1.0f;
  double speed = -.3f;
  expectedSpeeds.first = -.2250f;
  expectedSpeeds.second = -.3750f;

  returnSpeeds = speedAndRadius2WheelVels(speed, turnRadius, AxelLen, maxSpeed);

  EXPECT_NEAR(returnSpeeds.first, expectedSpeeds.first, SPEEDNRADIUSTOL);
  EXPECT_NEAR(returnSpeeds.second, expectedSpeeds.second, SPEEDNRADIUSTOL);
}

TEST(WaypointControllerHelperTests, speedAndRadius2WheelVelsTests_bkwdleft)
{
  std::pair<double, double> returnSpeeds;
  std::pair<double, double> expectedSpeeds;
  double AxelLen = .5f;
  double maxSpeed = .5f;
  double turnRadius = -1.0f;
  double speed = -.3f;
  expectedSpeeds.first = -.3750f;
  expectedSpeeds.second = -.2250f;

  returnSpeeds = speedAndRadius2WheelVels(speed, turnRadius, AxelLen, maxSpeed);

  EXPECT_NEAR(returnSpeeds.first, expectedSpeeds.first, SPEEDNRADIUSTOL);
  EXPECT_NEAR(returnSpeeds.second, expectedSpeeds.second, SPEEDNRADIUSTOL);
}

TEST(WaypointControllerHelperTests, speedAndRadius2WheelVelsTests_fwdright)
{
  std::pair<double, double> returnSpeeds;
  std::pair<double, double> expectedSpeeds;
  double AxelLen = .5f;
  double maxSpeed = .5f;
  double turnRadius = -1.0f;
  double speed = .3f;
  expectedSpeeds.first = .3750f;
  expectedSpeeds.second = .2250f;

  returnSpeeds = speedAndRadius2WheelVels(speed, turnRadius, AxelLen, maxSpeed);

  EXPECT_NEAR(returnSpeeds.first, expectedSpeeds.first, SPEEDNRADIUSTOL);
  EXPECT_NEAR(returnSpeeds.second, expectedSpeeds.second, SPEEDNRADIUSTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_FromOrigin_StraightThenTurnLeft)
{
  pose initialPose = {.x = 0.000000, .y = 0.000000, .theta = 0.000000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = 0.900000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 0.464920, .yc = 1000.000000, .distance = 0.929840 };
  // maneuver expected2 = {.radius = 2.642800, .xc = 0.929840, .yc = 2.642800, .distance = 2.378500 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
  /*
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);
  */
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_FromOrigin_StraightThenTurnRight)
{
  pose initialPose = {.x = 0.000000, .y = 0.000000, .theta = 0.000000 };
  pose waypoint = {.x = 3.000000, .y = -1.000000, .theta = -0.900000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 0.464920, .yc = 1000.000000, .distance = 0.929840 };
  // maneuver expected2 = {.radius = -2.642800, .xc = 0.929840, .yc = -2.642800, .distance = 2.378500 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_FromOrigin_TurnLeftThenStraight)
{
  pose initialPose = {.x = 0.000000, .y = 0.000000, .theta = 0.000000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = 0.500000 };
  // maneuver expected1 = {.radius = 4.580200, .xc = 0.000000, .yc = 4.580200, .distance = 2.290100 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = 880.180000, .yc = -1605.600000, .distance = 0.916320 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_FromOrigin_TurnRightThenStraight)
{
  pose initialPose = {.x = 0.000000, .y = 0.000000, .theta = 0.000000 };
  pose waypoint = {.x = 3.000000, .y = -1.000000, .theta = -0.500000 };
  // maneuver expected1 = {.radius = -4.580200, .xc = 0.000000, .yc = -4.580200, .distance = 2.290100 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = 880.180000, .yc = 1605.600000, .distance = 0.916320 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosY_StraightThenTurnLeft)
{
  pose initialPose = {.x = 2.000000, .y = 0.500000, .theta = 0.000000 };
  pose waypoint = {.x = 4.000000, .y = 1.000000, .theta = 0.700000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 2.315100, .yc = 1000.500000, .distance = 0.630240 };
  // maneuver expected2 = {.radius = 2.126200, .xc = 2.630200, .yc = 2.626200, .distance = 1.488400 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosY_StraightThenTurnRight)
{
  pose initialPose = {.x = 2.000000, .y = 0.500000, .theta = 0.000000 };
  pose waypoint = {.x = 4.000000, .y = 0.000000, .theta = -0.700000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 2.315100, .yc = 1000.500000, .distance = 0.630240 };
  // maneuver expected2 = {.radius = -2.126200, .xc = 2.630200, .yc = -1.626200, .distance = 1.488400 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransNegY_StraightThenTurnLeft)
{
  pose initialPose = {.x = 2.000000, .y = -0.500000, .theta = 0.000000 };
  pose waypoint = {.x = 4.000000, .y = 0.000000, .theta = 0.700000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 2.315100, .yc = 999.500000, .distance = 0.630240 };
  // maneuver expected2 = {.radius = 2.126200, .xc = 2.630200, .yc = 1.626200, .distance = 1.488400 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransNegY_StraightThenTurnRight)
{
  pose initialPose = {.x = 2.000000, .y = -0.500000, .theta = 0.000000 };
  pose waypoint = {.x = 4.000000, .y = -1.000000, .theta = -0.700000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 2.315100, .yc = 999.500000, .distance = 0.630240 };
  // maneuver expected2 = {.radius = -2.126200, .xc = 2.630200, .yc = -2.626200, .distance = 1.488400 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot1stQuad_StraightThenTurnLeft)
{
  pose initialPose = {.x = 2.000000, .y = 0.500000, .theta = 0.200000 };
  pose waypoint = {.x = 4.000000, .y = 1.000000, .theta = 0.700000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = -195.840000, .yc = 980.740000, .distance = 1.696400 };
  // maneuver expected2 = {.radius = 0.757200, .xc = 3.512200, .yc = 1.579100, .distance = 0.378600 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot1stQuad_StraightThenTurnRight)
{
  pose initialPose = {.x = 2.000000, .y = 0.500000, .theta = 0.200000 };
  pose waypoint = {.x = 4.036800, .y = 0.818310, .theta = -0.300000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = -195.840000, .yc = 980.740000, .distance = 1.696500 };
  // maneuver expected2 = {.radius = -0.757120, .xc = 3.813100, .yc = 0.095006, .distance = 0.378560 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot1stQuad_TurnLeftThenStraight)
{
  pose initialPose = {.x = 2.000000, .y = 0.200000, .theta = 0.100000 };
  pose waypoint = {.x = 2.500000, .y = 1.000000, .theta = 1.300000 };
  // maneuver expected1 = {.radius = 0.419950, .xc = 1.958100, .yc = 0.617860, .distance = 0.503940 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = 377.040000, .yc = -103.250000, .distance = 0.513180 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot1stQuad_TurnRightThenStraight)
{
  pose initialPose = {.x = 2.000000, .y = 0.200000, .theta = 0.100000 };
  pose waypoint = {.x = 2.629100, .y = -0.286710, .theta = -1.100000 };
  //  maneuver expected1 = {.radius = -0.533040, .xc = 2.053200, .yc = -0.330380, .distance = 0.639650 };
  //  maneuver expected2 = {.radius = 1000.000000, .xc = 349.060000, .yc = 176.160000, .distance = 0.222300 };  pose
  //  returnPose;

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotPI_2_StraightThenTurnLeft)
{
  pose initialPose = {.x = 2.000000, .y = 0.200000, .theta = M_PI_2 };
  pose waypoint = {.x = 1.500000, .y = 1.500000, .theta = 2.500000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = -998.000000, .yc = 0.351190, .distance = 0.302380 };
  // maneuver expected2 = {.radius = 1.245200, .xc = 0.754760, .yc = 0.502380, .distance = 1.157100 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotPI_2_StraightThenTurnRight)
{
  pose initialPose = {.x = 2.000000, .y = 0.200000, .theta = M_PI_2 };
  pose waypoint = {.x = 2.500000, .y = 1.500000, .theta = 0.641590 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = -998.000000, .yc = 0.351190, .distance = 0.302380 };
  // maneuver expected2 = {.radius = -1.245200, .xc = 3.245200, .yc = 0.502380, .distance = 1.157100 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotPI_2_TurnLeftThenStraight)
{
  pose initialPose = {.x = 2.000000, .y = 0.200000, .theta = M_PI_2 };
  pose waypoint = {.x = 1.500000, .y = 1.000000, .theta = 2.500000 };
  // maneuver expected1 = {.radius = 0.850950, .xc = 1.149100, .yc = 0.200000, .distance = 0.790700 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = 448.650000, .yc = 599.410000, .distance = 0.197620 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotPI_2_TurnRightThenStraight)
{
  pose initialPose = {.x = 2.000000, .y = 0.200000, .theta = M_PI_2 };
  pose waypoint = {.x = 2.500000, .y = 1.000000, .theta = 0.641590 };
  // maneuver expected1 = {.radius = -0.850950, .xc = 2.850900, .yc = 0.200000, .distance = 0.790700 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = -444.650000, .yc = 599.410000, .distance = 0.197620 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot2ndQuad_StraightThenTurnLeft)
{
  pose initialPose = {.x = 2.000000, .y = 0.200000, .theta = 1.700000 };
  pose waypoint = {.x = 1.500000, .y = 1.400000, .theta = 2.500000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = -989.690000, .yc = -128.420000, .distance = 0.447360 };
  // maneuver expected2 = {.radius = 1.125000, .xc = 0.826690, .yc = 0.498680, .distance = 0.900040 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot2ndQuad_StraightThenTurnRight)
{
  pose initialPose = {.x = 2.000000, .y = 0.200000, .theta = 1.700000 };
  pose waypoint = {.x = 2.176700, .y = 1.487900, .theta = 0.900000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = -989.690000, .yc = -128.420000, .distance = 0.447360 };
  // maneuver expected2 = {.radius = -1.125000, .xc = 3.058000, .yc = 0.788590, .distance = 0.900040 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot2ndQuad_TurnLeftThenStraight)
{
  pose initialPose = {.x = 2.000000, .y = 0.200000, .theta = 1.700000 };
  pose waypoint = {.x = 1.500000, .y = 0.800000, .theta = 2.500000 };
  // maneuver expected1 = {.radius = 0.598270, .xc = 1.406700, .yc = 0.122920, .distance = 0.478610 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = 582.880000, .yc = 778.780000, .distance = 0.330490 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot2ndQuad_TurnRightThenStraight)
{
  pose initialPose = {.x = 2.000000, .y = 0.200000, .theta = 1.700000 };
  pose waypoint = {.x = 2.330100, .y = 0.907850, .theta = 0.900000 };
  // maneuver expected1 = {.radius = -0.598270, .xc = 2.593300, .yc = 0.277080, .distance = 0.478610 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = -758.550000, .yc = 604.500000, .distance = 0.330490 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotPI_StraightThenTurnLeft)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = M_PI };
  pose waypoint = {.x = 1.000000, .y = 0.200000, .theta = -2.100000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 2.697300, .yc = -999.000000, .distance = 0.605350 };
  // maneuver expected2 = {.radius = 1.615700, .xc = 2.394700, .yc = -0.615660, .distance = 1.682900 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotPI_StraightThenTurnRight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = M_PI };
  pose waypoint = {.x = 1.000000, .y = 1.800000, .theta = 2.100000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 2.697300, .yc = -999.000000, .distance = 0.605350 };
  // maneuver expected2 = {.radius = -1.615700, .xc = 2.394700, .yc = 2.615700, .distance = 1.682900 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotPI_TurnLeftThenStraight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141600 };
  pose waypoint = {.x = 2.000000, .y = 0.200000, .theta = -2.100000 };
  // maneuver expected1 = {.radius = 0.927660, .xc = 3.000000, .yc = 0.072344, .distance = 0.966240 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = -502.750000, .yc = 295.630000, .distance = 0.394650 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotPI_TurnRightThenStraight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = M_PI };
  pose waypoint = {.x = 2.000000, .y = 1.800000, .theta = 2.100000 };
  // maneuver expected1 = {.radius = -0.927660, .xc = 3.000000, .yc = 1.927700, .distance = 0.966240 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = -502.750000, .yc = -293.630000, .distance = 0.394650 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotNegPI_StraightThenTurnLeft)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -M_PI };
  pose waypoint = {.x = 1.000000, .y = 0.200000, .theta = -2.100000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 2.697300, .yc = -999.000000, .distance = 0.605350 };
  // maneuver expected2 = {.radius = 1.615700, .xc = 2.394700, .yc = -0.615660, .distance = 1.682900 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotNegPI_StraightThenTurnRight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -M_PI };
  pose waypoint = {.x = 1.000000, .y = 1.800000, .theta = -4.183200 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 2.697300, .yc = -999.000000, .distance = 0.605350 };
  // maneuver expected2 = {.radius = -1.615700, .xc = 2.394700, .yc = 2.615700, .distance = 1.682900 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotNegPI_TurnLeftThenStraight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -M_PI };
  pose waypoint = {.x = 2.000000, .y = 0.200000, .theta = -2.100000 };
  // maneuver expected1 = {.radius = 0.927660, .xc = 3.000000, .yc = 0.072344, .distance = 0.966240 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = -502.750000, .yc = 295.630000, .distance = 0.394650 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotNegPI_TurnRightThenStraight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -M_PI };
  pose waypoint = {.x = 2.000000, .y = 1.800000, .theta = -4.183200 };
  // maneuver expected1 = {.radius = -0.927660, .xc = 3.000000, .yc = 1.927700, .distance = 0.966240 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = -502.750000, .yc = -293.630000, .distance = 0.394650 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot3rdQuad_StraightThenTurnLeft)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.900000 };
  pose waypoint = {.x = 1.000000, .y = 0.200000, .theta = -2.100000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 241.560000, .yc = -970.130000, .distance = 1.427800 };
  // maneuver expected2 = {.radius = 0.983430, .xc = 1.848900, .yc = -0.296480, .distance = 0.786740 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot3rdQuad_StraightThenTurnRight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.900000 };
  pose waypoint = {.x = 0.857280, .y = 0.779210, .theta = -3.700000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 241.560000, .yc = -970.130000, .distance = 1.427800 };
  // maneuver expected2 = {.radius = -0.983430, .xc = 1.378300, .yc = 1.613300, .distance = 0.786740 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot3rdQuad_TurnLeftThenStraight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.900000 };
  pose waypoint = {.x = 2.000000, .y = 0.200000, .theta = -2.100000 };
  // maneuver expected1 = {.radius = 1.514500, .xc = 3.362300, .yc = -0.470500, .distance = 1.211600 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = -836.330000, .yc = 490.560000, .distance = 0.108990 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot3rdQuad_TurnRightThenStraight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.900000 };
  pose waypoint = {.x = 1.742800, .y = 1.243800, .theta = -3.700000 };
  // maneuver expected1 = {.radius = -1.514500, .xc = 2.637700, .yc = 2.470500, .distance = 1.211600 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = -512.800000, .yc = -822.470000, .distance = 0.108990 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotNegPi_2_StraightThenTurnLeft)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -M_PI_2 };
  pose waypoint = {.x = 4.000000, .y = -1.000000, .theta = -0.400000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 1003.000000, .yc = 0.754250, .distance = 0.491500 };
  // maneuver expected2 = {.radius = 1.637800, .xc = 4.637800, .yc = 0.508500, .distance = 1.917500 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotNegPi_2_StraightThenTurnRight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -M_PI_2 };
  pose waypoint = {.x = 2.000000, .y = -1.000000, .theta = -2.741600 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 1003.000000, .yc = 0.754250, .distance = 0.491500 };
  // maneuver expected2 = {.radius = -1.637800, .xc = 1.362200, .yc = 0.508500, .distance = 1.917500 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotNegPi_2_TurnLeftThenStraight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -M_PI_2 };
  pose waypoint = {.x = 4.000000, .y = -0.500000, .theta = -0.800000 };
  // maneuver expected1 = {.radius = 1.159400, .xc = 4.159400, .yc = 1.000000, .distance = 0.893680 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = -734.950000, .yc = -717.510000, .distance = 0.964960 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRotNegPi_2_TurnRightThenStraight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570800 };
  pose waypoint = {.x = 2.000000, .y = -0.500000, .theta = -2.341600 };
  // maneuver expected1 = {.radius = -1.159400, .xc = 1.840600, .yc = 1.000000, .distance = 0.893680 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = 740.950000, .yc = -717.510000, .distance = 0.964960 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot4thQuad_StraightThenTurnLeft)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.400000 };
  pose waypoint = {.x = 4.000000, .y = -0.500000, .theta = -0.400000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 988.480000, .yc = 170.810000, .distance = 0.310970 };
  // maneuver expected2 = {.radius = 1.589100, .xc = 4.618800, .yc = 0.963640, .distance = 1.589100 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot4thQuad_StraightThenTurnRight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.400000 };
  pose waypoint = {.x = 2.560300, .y = -0.748320, .theta = -2.400000 };
  // maneuver expected1 = {.radius = 1000.000000, .xc = 988.480000, .yc = 170.810000, .distance = 0.310970 };
  // maneuver expected2 = {.radius = -1.589100, .xc = 1.486900, .yc = 0.423460, .distance = 1.589100 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot4thQuad_TurnLeftThenStraight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.400000 };
  pose waypoint = {.x = 4.000000, .y = 0.000000, .theta = -0.400000 };
  // maneuver expected1 = {.radius = 1.156500, .xc = 4.139700, .yc = 1.196600, .distance = 1.156500 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = -246.200000, .yc = -591.340000, .distance = 0.337310 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_OneTurn_TransPosYRot4thQuad_TurnRightThenStraight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.400000 };
  pose waypoint = {.x = 2.392800, .y = -0.277210, .theta = -2.400000 };
  // maneuver expected1 = {.radius = -1.156500, .xc = 1.860300, .yc = 0.803430, .distance = 1.156500 };
  // maneuver expected2 = {.radius = 1000.000000, .xc = 436.230000, .yc = -473.640000, .distance = 0.337310 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_FromOrigin_StraightThenTurnLeft)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.900000 };
  pose waypoint = {.x = 0.000000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 2.642800, .xc = 0.929840, .yc = 2.642800, .distance = -2.378500 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 0.464920, .yc = -793.550000, .distance = -0.929840 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_FromOrigin_StraightThenTurnRight)
{
  pose initialPose = {.x = 3.000000, .y = -1.000000, .theta = -0.900000 };
  pose waypoint = {.x = 0.000000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -2.642800, .xc = 0.929840, .yc = -2.642800, .distance = -2.378500 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 0.464920, .yc = 793.550000, .distance = -0.929840 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_FromOrigin_TurnLeftThenStraight)
{
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.500000 };
  pose waypoint = {.x = 0.000000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 482.020000, .yc = -876.800000, .distance = -0.916320 };
  maneuver expected2 = {.radius = 4.580200, .xc = 0.000000, .yc = 4.580200, .distance = -2.290100 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_FromOrigin_TurnRightThenStraight)
{
  pose initialPose = {.x = 3.000000, .y = -1.000000, .theta = -0.500000 };
  pose waypoint = {.x = 0.000000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -476.830000, .yc = -878.360000, .distance = -0.916320 };
  maneuver expected2 = {.radius = -4.580200, .xc = 0.000000, .yc = -4.580200, .distance = -2.290100 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosY_StraightThenTurnLeft)
{
  pose initialPose = {.x = 4.000000, .y = 1.000000, .theta = 0.700000 };
  pose waypoint = {.x = 2.000000, .y = 0.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 2.126200, .xc = 2.630200, .yc = 2.626200, .distance = -1.488400 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.315100, .yc = -1186.700000, .distance = -0.630240 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosY_StraightThenTurnRight)
{
  pose initialPose = {.x = 4.000000, .y = 0.000000, .theta = -0.700000 };
  pose waypoint = {.x = 2.000000, .y = 0.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -2.126200, .xc = 2.630200, .yc = -1.626200, .distance = -1.488400 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.315100, .yc = 1187.700000, .distance = -0.630240 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransNegY_StraightThenTurnLeft)
{
  pose initialPose = {.x = 4.000000, .y = 0.000000, .theta = 0.700000 };
  pose waypoint = {.x = 2.000000, .y = -0.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 2.126200, .xc = 2.630200, .yc = 1.626200, .distance = -1.488400 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.315100, .yc = -1187.700000, .distance = -0.630240 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransNegY_StraightThenTurnRight)
{
  pose initialPose = {.x = 4.000000, .y = -1.000000, .theta = -0.700000 };
  pose waypoint = {.x = 2.000000, .y = -0.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -2.126200, .xc = 2.630200, .yc = -2.626200, .distance = -1.488400 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.315100, .yc = 1186.700000, .distance = -0.630240 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot1stQuad_StraightThenTurnLeft)
{
  pose initialPose = {.x = 4.000000, .y = 1.000000, .theta = 0.700000 };
  pose waypoint = {.x = 2.000000, .y = 0.500000, .theta = 0.200000 };
  maneuver expected1 = {.radius = 0.757200, .xc = 3.512200, .yc = 1.579100, .distance = -0.378600 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 366.490000, .yc = -1793.300000, .distance = -1.696400 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot1stQuad_StraightThenTurnRight)
{
  pose initialPose = {.x = 4.036800, .y = 0.818310, .theta = -0.300000 };
  pose waypoint = {.x = 2.000000, .y = 0.500000, .theta = 0.200000 };
  maneuver expected1 = {.radius = -0.757120, .xc = 3.813100, .yc = 0.095006, .distance = -0.378560 };
  maneuver expected2 = {.radius = -1000.000000, .xc = -360.830000, .yc = 1794.700000, .distance = -1.696500 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot1stQuad_TurnLeftThenStraight)
{
  pose initialPose = {.x = 2.500000, .y = 1.000000, .theta = 1.300000 };
  pose waypoint = {.x = 2.000000, .y = 0.200000, .theta = 0.100000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 965.990000, .yc = -266.750000, .distance = -0.513180 };
  maneuver expected2 = {.radius = 0.419950, .xc = 1.958100, .yc = 0.617860, .distance = -0.503940 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot1stQuad_TurnRightThenStraight)
{
  pose initialPose = {.x = 2.629100, .y = -0.286710, .theta = -1.100000 };
  pose waypoint = {.x = 2.000000, .y = 0.200000, .theta = 0.100000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -888.630000, .yc = -453.780000, .distance = -0.222300 };
  maneuver expected2 = {.radius = -0.533040, .xc = 2.053200, .yc = -0.330380, .distance = -0.639650 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotPI_2_StraightThenTurnLeft)
{
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.500000 };
  pose waypoint = {.x = 2.000000, .y = 0.200000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 1.245200, .xc = 0.754760, .yc = 0.502380, .distance = -1.157100 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 749.020000, .yc = 0.350950, .distance = -0.302380 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotPI_2_StraightThenTurnRight)
{
  pose initialPose = {.x = 2.500000, .y = 1.500000, .theta = 0.641590 };
  pose waypoint = {.x = 2.000000, .y = 0.200000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -1.245200, .xc = 3.245200, .yc = 0.502380, .distance = -1.157100 };
  maneuver expected2 = {.radius = -1000.000000, .xc = -745.020000, .yc = 0.351440, .distance = -0.302380 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotPI_2_TurnLeftThenStraight)
{
  pose initialPose = {.x = 1.500000, .y = 1.000000, .theta = 2.500000 };
  pose waypoint = {.x = 2.000000, .y = 0.200000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 600.050000, .yc = 802.080000, .distance = -0.197620 };
  maneuver expected2 = {.radius = 0.850950, .xc = 1.149100, .yc = 0.200000, .distance = -0.790700 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotPI_2_TurnRightThenStraight)
{
  pose initialPose = {.x = 2.500000, .y = 1.000000, .theta = 0.641590 };
  pose waypoint = {.x = 2.000000, .y = 0.200000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 600.890000, .yc = -800.200000, .distance = -0.197620 };
  maneuver expected2 = {.radius = -0.850950, .xc = 2.850900, .yc = 0.200000, .distance = -0.790710 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot2ndQuad_StraightThenTurnLeft)
{
  pose initialPose = {.x = 1.500000, .y = 1.400000, .theta = 2.500000 };
  pose waypoint = {.x = 2.000000, .y = 0.200000, .theta = 1.700000 };
  maneuver expected1 = {.radius = 1.125000, .xc = 0.826690, .yc = 0.498680, .distance = -0.900040 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 965.090000, .yc = 125.560000, .distance = -0.447360 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot2ndQuad_StraightThenTurnRight)
{
  pose initialPose = {.x = 2.176700, .y = 1.487900, .theta = 0.900000 };
  pose waypoint = {.x = 2.000000, .y = 0.200000, .theta = 1.700000 };
  maneuver expected1 = {.radius = -1.124900, .xc = 3.057800, .yc = 0.788670, .distance = -0.899900 };
  maneuver expected2 = {.radius = -1000.000000, .xc = -961.150000, .yc = -124.710000, .distance = -0.447460 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot2ndQuad_TurnLeftThenStraight)
{
  pose initialPose = {.x = 1.500000, .y = 0.800000, .theta = 2.500000 };
  pose waypoint = {.x = 2.000000, .y = 0.200000, .theta = 1.700000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 600.100000, .yc = 801.840000, .distance = -0.330490 };
  maneuver expected2 = {.radius = 0.598270, .xc = 1.406700, .yc = 0.122920, .distance = -0.478610 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot2ndQuad_TurnRightThenStraight)
{
  pose initialPose = {.x = 2.330100, .y = 0.907850, .theta = 0.900000 };
  pose waypoint = {.x = 2.000000, .y = 0.200000, .theta = 1.700000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 785.550000, .yc = -620.830000, .distance = -0.330550 };
  maneuver expected2 = {.radius = -0.598200, .xc = 2.593200, .yc = 0.277070, .distance = -0.478560 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotPI_StraightThenTurnLeft)
{
  pose initialPose = {.x = 1.000000, .y = 0.200000, .theta = -2.100000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 1.615700, .xc = 2.394700, .yc = -0.615660, .distance = -1.682900 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.697100, .yc = 585.850000, .distance = -0.605350 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotPI_StraightThenTurnRight)
{
  pose initialPose = {.x = 1.000000, .y = 1.800000, .theta = 2.100000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -1.615700, .xc = 2.394700, .yc = 2.615700, .distance = -1.682900 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.697500, .yc = -583.850000, .distance = -0.605350 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotPI_TurnLeftThenStraight)
{
  pose initialPose = {.x = 2.000000, .y = 0.200000, .theta = -2.100000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -861.110000, .yc = 505.220000, .distance = -0.394650 };
  maneuver expected2 = {.radius = 0.927670, .xc = 3.000000, .yc = 0.072332, .distance = -0.966250 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotPI_TurnRightThenStraight)
{
  pose initialPose = {.x = 2.000000, .y = 1.800000, .theta = 2.100000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 865.310000, .yc = 506.480000, .distance = -0.394650 };
  maneuver expected2 = {.radius = -0.927660, .xc = 3.000000, .yc = 1.927700, .distance = -0.966240 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotNegPI_StraightThenTurnLeft)
{
  pose initialPose = {.x = 1.000000, .y = 0.200000, .theta = -2.100000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 1.615700, .xc = 2.394700, .yc = -0.615660, .distance = -1.682900 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.697500, .yc = 585.850000, .distance = -0.605350 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotNegPI_StraightThenTurnRight)
{
  pose initialPose = {.x = 1.000000, .y = 1.800000, .theta = -4.183200 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -1.615600, .xc = 2.394600, .yc = 2.615600, .distance = -1.682800 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.697100, .yc = -583.830000, .distance = -0.605370 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotNegPI_TurnLeftThenStraight)
{
  pose initialPose = {.x = 2.000000, .y = 0.200000, .theta = -2.100000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -861.110000, .yc = 505.220000, .distance = -0.394650 };
  maneuver expected2 = {.radius = 0.927660, .xc = 3.000000, .yc = 0.072345, .distance = -0.966240 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotNegPI_TurnRightThenStraight)
{
  pose initialPose = {.x = 2.000000, .y = 1.800000, .theta = -4.183200 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 865.320000, .yc = 506.460000, .distance = -0.394630 };
  maneuver expected2 = {.radius = -0.927670, .xc = 3.000000, .yc = 1.927700, .distance = -0.966270 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot3rdQuad_StraightThenTurnLeft)
{
  pose initialPose = {.x = 1.000000, .y = 0.200000, .theta = -2.100000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -2.900000 };
  maneuver expected1 = {.radius = 0.983430, .xc = 1.848900, .yc = -0.296480, .distance = -0.786740 };
  maneuver expected2 = {.radius = -1000.000000, .xc = -230.060000, .yc = 943.840000, .distance = -1.427800 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot3rdQuad_StraightThenTurnRight)
{
  pose initialPose = {.x = 0.857280, .y = 0.779210, .theta = -3.700000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -2.900000 };
  maneuver expected1 = {.radius = -0.983430, .xc = 1.378300, .yc = 1.613300, .distance = -0.786740 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 234.670000, .yc = -942.180000, .distance = -1.427800 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot3rdQuad_TurnLeftThenStraight)
{
  pose initialPose = {.x = 2.000000, .y = 0.200000, .theta = -2.100000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -2.900000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -861.180000, .yc = 505.090000, .distance = -0.108990 };
  maneuver expected2 = {.radius = 1.514500, .xc = 3.362300, .yc = -0.470500, .distance = -1.211600 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot3rdQuad_TurnRightThenStraight)
{
  pose initialPose = {.x = 1.742800, .y = 1.243800, .theta = -3.700000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -2.900000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 531.630000, .yc = 849.310000, .distance = -0.108960 };
  maneuver expected2 = {.radius = -1.514500, .xc = 2.637700, .yc = 2.470500, .distance = -1.211600 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotNegPi_2_StraightThenTurnLeft)
{
  pose initialPose = {.x = 4.000000, .y = -1.000000, .theta = -0.400000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 1.637800, .xc = 4.637800, .yc = 0.508500, .distance = -1.917500 };
  maneuver expected2 = {.radius = -1000.000000, .xc = -419.790000, .yc = 0.754110, .distance = -0.491500 };
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotNegPi_2_StraightThenTurnRight)
{
  pose initialPose = {.x = 2.000000, .y = -1.000000, .theta = -2.741600 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -1.637800, .xc = 1.362200, .yc = 0.508490, .distance = -1.917500 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 425.780000, .yc = 0.754380, .distance = -0.491510 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotNegPi_2_TurnLeftThenStraight)
{
  pose initialPose = {.x = 4.000000, .y = -0.500000, .theta = -0.800000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -713.690000, .yc = -696.860000, .distance = -0.964960 };
  maneuver expected2 = {.radius = 1.159400, .xc = 4.159400, .yc = 1.000000, .distance = -0.893680 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRotNegPi_2_TurnRightThenStraight)
{
  pose initialPose = {.x = 2.000000, .y = -0.500000, .theta = -2.341600 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -715.010000, .yc = 696.560000, .distance = -0.964930 };
  maneuver expected2 = {.radius = -1.159500, .xc = 1.840500, .yc = 1.000000, .distance = -0.893710 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot4thQuad_StraightThenTurnLeft)
{
  pose initialPose = {.x = 4.000000, .y = -0.500000, .theta = -0.400000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -1.400000 };
  maneuver expected1 = {.radius = 1.589100, .xc = 4.618800, .yc = 0.963640, .distance = -1.589100 };
  maneuver expected2 = {.radius = -1000.000000, .xc = -629.720000, .yc = -108.290000, .distance = -0.310970 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot4thQuad_StraightThenTurnRight)
{
  pose initialPose = {.x = 2.560300, .y = -0.748320, .theta = -2.400000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -1.400000 };
  maneuver expected1 = {.radius = -1.589000, .xc = 1.487000, .yc = 0.423400, .distance = -1.589000 };
  maneuver expected2 = {.radius = -1000.000000, .xc = 635.780000, .yc = 109.980000, .distance = -0.311050 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot4thQuad_TurnLeftThenStraight)
{
  pose initialPose = {.x = 4.000000, .y = 0.000000, .theta = -0.400000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -1.400000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -385.570000, .yc = -921.000000, .distance = -0.337310 };
  maneuver expected2 = {.radius = 1.156500, .xc = 4.139700, .yc = 1.196600, .distance = -1.156500 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest_InverseOneTurn_TransPosYRot4thQuad_TurnRightThenStraight)
{
  pose initialPose = {.x = 2.392800, .y = -0.277210, .theta = -2.400000 };
  pose waypoint = {.x = 3.000000, .y = 1.000000, .theta = -1.400000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -672.950000, .yc = 737.230000, .distance = -0.337250 };
  maneuver expected2 = {.radius = -1.156600, .xc = 1.860300, .yc = 0.803420, .distance = -1.156600 };

  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() == 2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta), 0.0f, WAYPOINT2MANEUVERTOL);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
