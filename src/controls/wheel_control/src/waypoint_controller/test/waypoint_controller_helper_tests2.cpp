#include <gtest/gtest.h>
#include <waypoint_controller/waypoint_controller_helper.h>
#include <vector>
#include <utility>
#define _USE_MATH_DEFINES
#include <cmath>

#define WAYPOINT2MANEUVERTOL .05

using namespace WaypointControllerHelper;

TEST(WaypointControllerHelperTests, waypointGenerationTest1)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest2)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest3)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest4)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest5)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest6)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest7)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest8)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest9)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest10)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest11)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest12)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest13)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest14)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest15)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest16)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest17)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest18)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest19)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest20)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest21)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest22)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest23)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest24)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest25)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest26)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest27)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest28)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest29)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest30)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest31)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest32)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest33)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest34)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest35)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest36)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest37)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest38)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest39)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest40)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest41)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest42)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest43)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest44)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest45)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest46)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest47)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest48)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest49)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest50)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest51)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest52)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest53)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest54)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest55)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest56)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest57)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest58)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest59)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest60)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest61)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest62)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest63)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest64)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest65)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest66)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest67)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest68)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest69)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest70)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest71)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest72)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest73)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest74)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest75)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest76)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest77)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest78)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest79)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest80)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest81)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest82)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest83)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest84)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest85)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest86)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest87)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest88)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest89)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest90)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest91)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest92)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest93)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest94)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest95)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest96)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest97)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest98)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest99)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest100)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest101)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest102)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest103)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest104)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest105)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest106)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest107)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest108)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest109)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest110)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest111)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest112)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest113)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest114)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest115)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest116)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest117)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest118)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest119)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest120)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest121)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest122)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest123)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest124)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest125)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest126)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest127)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest128)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest129)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest130)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest131)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest132)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest133)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest134)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest135)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest136)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest137)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest138)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest139)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest140)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest141)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest142)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest143)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest144)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest145)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest146)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest147)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest148)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest149)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest150)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest151)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest152)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest153)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest154)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest155)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest156)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest157)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest158)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest159)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest160)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest161)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest162)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest163)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest164)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest165)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest166)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest167)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest168)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest169)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest170)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest171)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest172)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest173)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest174)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest175)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest176)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest177)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest178)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest179)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest180)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest181)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest182)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest183)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest184)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest185)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest186)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest187)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest188)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest189)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest190)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest191)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest192)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest193)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest194)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest195)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest196)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest197)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest198)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest199)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest200)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest201)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest202)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest203)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest204)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest205)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest206)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest207)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest208)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest209)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest210)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest211)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest212)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest213)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest214)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest215)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest216)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest217)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest218)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest219)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest220)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest221)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest222)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest223)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest224)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest225)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest226)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest227)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest228)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest229)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest230)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest231)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest232)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest233)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest234)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest235)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest236)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest237)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest238)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest239)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest240)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest241)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest242)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest243)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest244)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest245)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest246)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest247)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest248)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest249)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest250)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest251)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest252)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest253)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest254)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest255)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest256)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest257)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest258)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest259)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest260)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest261)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest262)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest263)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest264)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest265)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest266)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest267)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest268)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest269)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest270)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest271)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest272)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest273)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest274)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest275)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest276)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest277)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest278)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest279)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest280)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest281)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest282)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest283)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest284)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest285)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest286)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest287)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest288)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest289)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest290)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest291)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest292)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest293)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest294)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest295)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest296)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest297)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest298)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest299)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest300)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest301)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest302)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest303)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest304)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest305)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest306)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest307)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest308)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest309)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest310)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest311)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest312)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest313)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest314)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest315)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest316)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest317)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest318)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest319)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest320)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest321)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest322)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest323)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest324)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest325)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest326)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest327)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest328)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest329)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest330)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest331)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest332)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest333)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest334)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest335)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest336)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest337)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest338)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest339)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest340)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest341)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest342)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest343)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest344)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest345)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest346)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest347)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest348)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest349)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest350)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest351)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest352)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest353)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest354)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest355)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest356)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest357)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest358)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest359)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest360)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest361)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest362)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest363)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest364)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest365)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest366)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest367)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest368)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest369)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest370)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest371)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest372)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest373)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest374)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest375)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest376)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest377)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest378)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest379)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest380)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest381)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest382)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest383)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest384)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest385)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest386)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest387)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest388)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest389)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest390)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest391)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest392)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest393)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest394)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest395)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest396)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest397)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest398)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest399)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest400)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest401)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest402)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest403)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest404)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest405)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest406)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest407)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest408)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest409)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest410)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest411)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest412)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest413)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest414)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest415)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest416)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest417)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest418)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest419)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest420)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest421)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest422)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest423)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest424)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest425)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest426)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest427)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest428)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest429)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest430)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest431)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest432)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest433)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest434)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest435)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest436)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest437)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest438)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest439)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest440)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest441)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest442)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest443)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest444)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest445)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest446)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest447)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest448)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest449)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest450)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest451)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest452)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest453)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest454)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest455)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest456)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest457)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest458)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest459)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest460)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest461)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest462)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest463)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest464)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest465)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest466)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest467)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest468)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest469)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest470)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest471)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest472)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest473)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest474)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest475)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest476)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest477)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest478)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest479)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest480)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest481)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest482)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest483)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest484)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest485)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest486)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest487)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest488)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest489)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest490)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest491)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest492)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest493)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest494)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest495)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest496)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest497)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest498)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest499)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest500)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest501)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest502)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest503)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest504)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest505)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest506)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest507)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest508)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest509)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest510)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest511)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest512)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest513)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest514)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest515)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest516)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest517)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest518)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest519)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest520)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest521)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest522)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest523)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest524)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest525)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest526)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest527)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest528)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest529)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest530)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest531)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest532)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest533)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest534)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest535)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest536)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest537)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest538)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest539)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest540)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest541)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest542)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest543)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest544)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest545)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest546)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest547)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest548)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest549)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest550)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest551)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest552)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest553)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest554)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest555)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest556)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest557)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest558)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest559)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest560)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest561)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest562)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest563)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest564)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest565)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest566)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest567)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest568)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest569)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest570)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest571)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest572)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest573)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest574)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest575)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest576)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest577)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest578)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest579)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest580)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest581)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest582)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest583)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest584)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest585)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest586)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest587)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest588)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest589)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest590)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest591)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest592)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest593)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest594)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest595)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest596)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest597)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest598)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest599)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest600)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest601)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest602)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest603)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest604)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest605)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest606)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest607)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest608)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest609)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest610)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest611)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest612)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest613)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest614)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest615)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest616)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest617)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest618)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest619)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest620)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest621)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest622)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest623)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest624)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest625)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest626)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest627)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest628)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest629)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest630)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest631)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest632)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest633)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest634)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest635)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest636)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest637)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest638)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest639)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest640)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest641)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest642)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest643)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest644)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest645)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest646)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest647)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796 };
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

TEST(WaypointControllerHelperTests, waypointGenerationTest648)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398 };
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398 };
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