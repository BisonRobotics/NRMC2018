#include <gtest/gtest.h>
#include <waypoint_controller/waypoint_controller_helper.h>
#include <vector>
#include <utility>
#define _USE_MATH_DEFINES
#include <cmath>

#define WAYPOINT2MANEUVERTOL .05

TEST(WaypointControllerHelperTests, ableToReflectWaypoint1)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint2)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint3)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint4)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint5)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint6)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint7)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint8)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint9)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint10)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint11)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint12)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint13)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint14)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint15)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint16)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint17)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint18)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint19)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint20)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint21)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint22)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint23)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint24)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint25)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint26)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint27)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint28)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint29)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint30)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint31)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint32)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint33)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint34)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint35)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint36)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint37)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint38)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint39)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint40)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint41)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint42)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint43)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint44)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint45)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint46)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint47)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint48)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint49)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint50)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint51)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint52)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint53)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint54)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint55)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint56)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint57)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint58)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint59)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint60)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint61)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint62)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint63)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint64)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint65)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint66)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint67)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint68)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint69)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint70)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint71)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint72)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.000000};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint73)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint74)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint75)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint76)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint77)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint78)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint79)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint80)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint81)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint82)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint83)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint84)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint85)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint86)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint87)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint88)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint89)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint90)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint91)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint92)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint93)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint94)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint95)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint96)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint97)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint98)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint99)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint100)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint101)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint102)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint103)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint104)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint105)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint106)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint107)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint108)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint109)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint110)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint111)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint112)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint113)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint114)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint115)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint116)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint117)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint118)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint119)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint120)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint121)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint122)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint123)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint124)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint125)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint126)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint127)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint128)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint129)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint130)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint131)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint132)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint133)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint134)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint135)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint136)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint137)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint138)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint139)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint140)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint141)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint142)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint143)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint144)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint145)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint146)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint147)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint148)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint149)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint150)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint151)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint152)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint153)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint154)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint155)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint156)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint157)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint158)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint159)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint160)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint161)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint162)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint163)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint164)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint165)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint166)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint167)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint168)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint169)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint170)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint171)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint172)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint173)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint174)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint175)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint176)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint177)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint178)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint179)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint180)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint181)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint182)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint183)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint184)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint185)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint186)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint187)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint188)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint189)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint190)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint191)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint192)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint193)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint194)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint195)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint196)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint197)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint198)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint199)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint200)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint201)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint202)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint203)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint204)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint205)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint206)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint207)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint208)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint209)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint210)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint211)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint212)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint213)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint214)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint215)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint216)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint217)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint218)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint219)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint220)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint221)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint222)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint223)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint224)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint225)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint226)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint227)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint228)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint229)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint230)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint231)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint232)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint233)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint234)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint235)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint236)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint237)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint238)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint239)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint240)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint241)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint242)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint243)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint244)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint245)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint246)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint247)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint248)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint249)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint250)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint251)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint252)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint253)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint254)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint255)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint256)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint257)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint258)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint259)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint260)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint261)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint262)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint263)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint264)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint265)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint266)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint267)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint268)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint269)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint270)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint271)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint272)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint273)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint274)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint275)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint276)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint277)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint278)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint279)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint280)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint281)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint282)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint283)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint284)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint285)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint286)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint287)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint288)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint289)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint290)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint291)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint292)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint293)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint294)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint295)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint296)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint297)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint298)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint299)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint300)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint301)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint302)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint303)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint304)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint305)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint306)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint307)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint308)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint309)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint310)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint311)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint312)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint313)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint314)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint315)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint316)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint317)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint318)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint319)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint320)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint321)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint322)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint323)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint324)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint325)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint326)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint327)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint328)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint329)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint330)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint331)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint332)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint333)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint334)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint335)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint336)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint337)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint338)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint339)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint340)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint341)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint342)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint343)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint344)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint345)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint346)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint347)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint348)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint349)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint350)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint351)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint352)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint353)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint354)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint355)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint356)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint357)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint358)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint359)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint360)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = 3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint361)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint362)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint363)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint364)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint365)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint366)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint367)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint368)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint369)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint370)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint371)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint372)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint373)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint374)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint375)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint376)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint377)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint378)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint379)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint380)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint381)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint382)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint383)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint384)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint385)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint386)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint387)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint388)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint389)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint390)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint391)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint392)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint393)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint394)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint395)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint396)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint397)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint398)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint399)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint400)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint401)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint402)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint403)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint404)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint405)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint406)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint407)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint408)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint409)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint410)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint411)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint412)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint413)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint414)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint415)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint416)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint417)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint418)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint419)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint420)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint421)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint422)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint423)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint424)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint425)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint426)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint427)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint428)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint429)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint430)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint431)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint432)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -3.141593};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint433)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint434)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint435)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint436)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint437)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint438)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint439)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint440)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint441)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint442)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint443)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint444)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint445)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint446)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint447)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint448)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint449)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint450)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint451)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint452)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint453)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint454)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint455)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint456)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint457)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint458)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint459)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint460)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint461)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint462)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint463)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint464)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint465)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint466)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint467)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint468)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint469)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint470)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint471)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint472)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint473)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint474)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint475)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint476)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint477)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint478)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint479)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint480)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint481)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint482)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint483)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint484)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint485)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint486)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint487)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint488)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint489)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint490)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint491)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint492)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint493)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint494)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint495)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint496)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint497)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint498)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint499)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint500)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint501)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint502)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint503)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint504)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -2.356194};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint505)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint506)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint507)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint508)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint509)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint510)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint511)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint512)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint513)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint514)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint515)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint516)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint517)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint518)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint519)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint520)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint521)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint522)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint523)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint524)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint525)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint526)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint527)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint528)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint529)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint530)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint531)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint532)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint533)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint534)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint535)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint536)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint537)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint538)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint539)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint540)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint541)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint542)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint543)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint544)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint545)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint546)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint547)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint548)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint549)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint550)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint551)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint552)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint553)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint554)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint555)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint556)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint557)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint558)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint559)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint560)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint561)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint562)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint563)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint564)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint565)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint566)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint567)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint568)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint569)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint570)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint571)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint572)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint573)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint574)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint575)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint576)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -1.570796};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint577)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint578)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint579)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint580)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint581)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint582)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint583)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint584)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint585)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint586)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint587)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint588)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint589)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint590)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint591)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint592)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint593)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint594)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.500000, .y = 0.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint595)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint596)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint597)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint598)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint599)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint600)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint601)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint602)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint603)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint604)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint605)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint606)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint607)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint608)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint609)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint610)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint611)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint612)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 3.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint613)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint614)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint615)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint616)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint617)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint618)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint619)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint620)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint621)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 2.000000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint622)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint623)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint624)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint625)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint626)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint627)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint628)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint629)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint630)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.500000, .y = 1.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint631)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint632)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint633)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint634)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint635)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint636)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint637)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint638)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint639)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 1.000000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint640)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.000000};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint641)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint642)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint643)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint644)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = 3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint645)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -3.141593};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint646)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -2.356194};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint647)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -1.570796};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, ableToReflectWaypoint648)
{
  pose waypoint = {.x = 2.000000, .y = 1.000000, .theta = -0.785398};
  pose initialPose = {.x = 1.000000, .y = 0.500000, .theta = -0.785398};
  pose returnPose;

  std::vector<maneuver> myMans;
  myMans = waypoint2maneuvers(initialPose, waypoint);

  EXPECT_TRUE(myMans.size() ==2);
  returnPose = endOfManeuver(initialPose, myMans.at(0));
  returnPose = endOfManeuver(returnPose, myMans.at(1));
  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);
}





// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}