#include <gtest/gtest.h>
#include <waypoint_controller/waypoint_controller_helper.h>
#include <vector>
#include <utility>
#define _USE_MATH_DEFINES
#include <cmath>

#define WAYPOINT2MANEUVERTOL .05


TEST(WaypointControllerHelperTests2, waypoint2maneuversTest1)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.250010, .xc = 2.002500, .yc = 0.750010, .distance = -0.781670};
  maneuver expected2 = {.radius = 0.250010, .xc = 2.000000, .yc = 0.250010, .distance = -0.784170};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest2)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.279490, .xc = 2.000000, .yc = 0.720510, .distance = -0.777050};
  maneuver expected2 = {.radius = 0.279490, .xc = 1.802400, .yc = 0.197630, .distance = -0.557540};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest3)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.029702, .xc = 2.020000, .yc = 0.970300, .distance = -0.046952};
  maneuver expected2 = {.radius = -1000.000000, .xc = -8.004400, .yc = 0.385010, .distance = -0.970050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest4)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.414210, .xc = 2.000000, .yc = 0.585790, .distance = -0.975970};
  maneuver expected2 = {.radius = -1000.000000, .xc = -705.250000, .yc = -706.960000, .distance = -0.414210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest5)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.499990, .xc = 2.000000, .yc = 0.500010, .distance = 1.565800};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1002.000000, .yc = -99988.000000, .distance = 0.005000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest6)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.499990, .xc = 2.000000, .yc = 0.500010, .distance = 1.565800};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1002.000000, .yc = -99995.000000, .distance = 0.005000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest7)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.414210, .xc = 2.000000, .yc = 0.585790, .distance = 0.975970};
  maneuver expected2 = {.radius = 1000.000000, .xc = 709.250000, .yc = -706.960000, .distance = 0.414210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest8)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.360630, .xc = 2.020000, .yc = 0.639370, .distance = 0.955580};
  maneuver expected2 = {.radius = 0.360630, .xc = 2.360600, .yc = 0.003606, .distance = 0.392710};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest9)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.279490, .xc = 2.000000, .yc = 0.720510, .distance = 0.777050};
  maneuver expected2 = {.radius = 0.279490, .xc = 2.197600, .yc = 0.197630, .distance = 0.557540};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest10)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.311730, .xc = 2.003100, .yc = 0.688290, .distance = 0.694940};
  maneuver expected2 = {.radius = 0.311730, .xc = 2.500000, .yc = 0.311730, .distance = 0.691820};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest11)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.291240, .xc = 2.000000, .yc = 0.708760, .distance = 0.760850};
  maneuver expected2 = {.radius = 0.291240, .xc = 2.294100, .yc = 0.205940, .distance = 0.989590};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest12)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.340760, .xc = 2.000000, .yc = 0.659240, .distance = 0.990150};
  maneuver expected2 = {.radius = 0.340760, .xc = 2.159300, .yc = -0.003407, .distance = 1.528800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest13)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.207110, .xc = 2.000000, .yc = 0.792890, .distance = -0.487980};
  maneuver expected2 = {.radius = -1000.000000, .xc = -704.930000, .yc = -706.780000, .distance = -0.914210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest14)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 2.247500, .yc = 1001.000000, .distance = 0.495000};
  maneuver expected2 = {.radius = -0.500010, .xc = 2.495000, .yc = 0.499990, .distance = 1.565800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest15)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 2.247500, .yc = 1001.000000, .distance = 0.495000};
  maneuver expected2 = {.radius = -0.500010, .xc = 2.495000, .yc = 0.499990, .distance = 1.565800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest16)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 2.042900, .yc = 1001.000000, .distance = 0.085786};
  maneuver expected2 = {.radius = -0.585790, .xc = 2.085800, .yc = 0.414210, .distance = 1.380200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest17)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.494920, .xc = 2.000000, .yc = 0.505080, .distance = 0.772480};
  maneuver expected2 = {.radius = 1000.000000, .xc = 12.498000, .yc = 0.355020, .distance = 0.510050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest18)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.423020, .xc = 2.000000, .yc = 0.576980, .distance = 0.806040};
  maneuver expected2 = {.radius = 0.423020, .xc = 2.799100, .yc = 0.299120, .distance = 0.473800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest19)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -11.808000, .xc = 1.881900, .yc = -10.788000, .distance = 0.382070};
  maneuver expected2 = {.radius = 11.808000, .xc = 2.881900, .yc = 12.808000, .distance = 0.618230};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest20)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.553680, .xc = 2.000000, .yc = 0.466320, .distance = 0.322110};
  maneuver expected2 = {.radius = 0.553680, .xc = 2.608500, .yc = 1.391500, .distance = 0.756970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest21)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.361950, .xc = 2.000000, .yc = 0.658050, .distance = 0.390480};
  maneuver expected2 = {.radius = 0.361950, .xc = 2.638000, .yc = 1.000000, .distance = 0.959030};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest22)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.371360, .xc = 2.000000, .yc = 0.648640, .distance = 0.538850};
  maneuver expected2 = {.radius = 0.371360, .xc = 2.737400, .yc = 0.737410, .distance = 1.413900};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest23)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 2.500000, .yc = 1001.000000, .distance = 1.000000};
  maneuver expected2 = {.radius = -0.010002, .xc = 3.000000, .yc = 1.010000, .distance = 0.031423};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest24)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.010000, .xc = 2.000000, .yc = 1.010000, .distance = -0.031415};
  maneuver expected2 = {.radius = -1000.000000, .xc = -997.500000, .yc = -2886800000.000000, .distance = -1.000000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest25)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 2.495900, .yc = 1001.000000, .distance = 0.991720};
  maneuver expected2 = {.radius = -0.011716, .xc = 2.991700, .yc = 1.008300, .distance = 0.027605};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest26)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 2.490000, .yc = 1001.000000, .distance = 0.980000};
  maneuver expected2 = {.radius = -0.020000, .xc = 2.980000, .yc = 1.000000, .distance = 0.031416};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest27)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 2.475900, .yc = 1001.000000, .distance = 0.951720};
  maneuver expected2 = {.radius = -0.068284, .xc = 2.951700, .yc = 0.951720, .distance = 0.053630};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest28)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.631330, .xc = 1.993700, .yc = 1.631300, .distance = 0.575960};
  maneuver expected2 = {.radius = -0.631330, .xc = 3.000000, .yc = 0.868670, .distance = 0.582270};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest29)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 1.207100, .xc = 2.000000, .yc = 2.207100, .distance = 0.948060};
  maneuver expected2 = {.radius = 1000.000000, .xc = 710.030000, .yc = -705.680000, .distance = 0.207110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest30)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 2.252500, .yc = 1001.000000, .distance = 0.504980};
  maneuver expected2 = {.radius = 0.495050, .xc = 2.505000, .yc = 1.495000, .distance = 0.782570};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest31)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 2.396400, .yc = 1001.000000, .distance = 0.792890};
  maneuver expected2 = {.radius = 0.292890, .xc = 2.792900, .yc = 1.292900, .distance = 0.690110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest32)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.244990, .xc = 2.000000, .yc = 1.245000, .distance = -0.767220};
  maneuver expected2 = {.radius = -1000.000000, .xc = -997.450000, .yc = 99990.000000, .distance = -1.002500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest33)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.244990, .xc = 2.000000, .yc = 1.245000, .distance = -0.767220};
  maneuver expected2 = {.radius = -1000.000000, .xc = -997.450000, .yc = 99997.000000, .distance = -1.002500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest34)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.404690, .xc = 2.000000, .yc = 1.404700, .distance = 0.834290};
  maneuver expected2 = {.radius = -0.404690, .xc = 2.713800, .yc = 1.786200, .distance = 1.787800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest35)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.339480, .xc = 2.000000, .yc = 1.339500, .distance = 0.612540};
  maneuver expected2 = {.radius = -0.339480, .xc = 2.660500, .yc = 1.496600, .distance = 1.142400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest36)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.374190, .xc = 2.000000, .yc = 1.374200, .distance = 0.517980};
  maneuver expected2 = {.radius = -0.374190, .xc = 2.735400, .yc = 1.235400, .distance = 0.811870};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest37)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.125000, .xc = 1.998700, .yc = 1.125000, .distance = 0.390830};
  maneuver expected2 = {.radius = -0.125000, .xc = 2.000000, .yc = 1.375000, .distance = 0.392080};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest38)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.139750, .xc = 2.000000, .yc = 1.139700, .distance = 0.388520};
  maneuver expected2 = {.radius = -0.139750, .xc = 2.098800, .yc = 1.401200, .distance = 0.278770};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest39)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.179750, .xc = 2.020000, .yc = 1.179700, .distance = 0.481920};
  maneuver expected2 = {.radius = -0.179750, .xc = 2.179700, .yc = 1.501800, .distance = 0.197780};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest40)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.207110, .xc = 2.000000, .yc = 1.207100, .distance = 0.487980};
  maneuver expected2 = {.radius = 1000.000000, .xc = 709.180000, .yc = 708.530000, .distance = 0.207110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest41)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.249990, .xc = 2.000000, .yc = 1.250000, .distance = -0.782880};
  maneuver expected2 = {.radius = -1000.000000, .xc = -997.950000, .yc = 99990.000000, .distance = -0.002500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest42)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.249990, .xc = 2.000000, .yc = 1.250000, .distance = -0.782880};
  maneuver expected2 = {.radius = -1000.000000, .xc = -997.950000, .yc = 99997.000000, .distance = -0.002500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest43)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.207110, .xc = 2.000000, .yc = 1.207100, .distance = -0.487980};
  maneuver expected2 = {.radius = -1000.000000, .xc = -705.180000, .yc = 708.530000, .distance = -0.207110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest44)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.015150, .xc = 2.020000, .yc = 1.015200, .distance = -0.023647};
  maneuver expected2 = {.radius = -1000.000000, .xc = -7.997700, .yc = 1.157500, .distance = -0.485030};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest45)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.139750, .xc = 2.000000, .yc = 1.139700, .distance = -0.388520};
  maneuver expected2 = {.radius = -0.139750, .xc = 1.901200, .yc = 1.401200, .distance = -0.278770};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest46)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.248760, .xc = 1.997500, .yc = 1.248700, .distance = -0.394490};
  maneuver expected2 = {.radius = -0.248760, .xc = 1.500000, .yc = 1.251200, .distance = -0.392000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest47)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.197630, .xc = 2.000000, .yc = 1.197600, .distance = -0.394240};
  maneuver expected2 = {.radius = -0.197630, .xc = 1.639700, .yc = 1.360300, .distance = -0.549460};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest48)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.207540, .xc = 2.000000, .yc = 1.207500, .distance = -0.489730};
  maneuver expected2 = {.radius = -0.207540, .xc = 1.707500, .yc = 1.502100, .distance = -0.817810};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest49)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.257330, .xc = 2.000000, .yc = 1.277300, .distance = -0.637010};
  maneuver expected2 = {.radius = -0.257330, .xc = 1.682000, .yc = 1.682000, .distance = -1.243300};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest50)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1.751300, .yc = -999.000000, .distance = -0.497500};
  maneuver expected2 = {.radius = 0.250010, .xc = 1.502500, .yc = 1.250000, .distance = -0.782920};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest51)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1.751200, .yc = -999.000000, .distance = -0.497500};
  maneuver expected2 = {.radius = 0.250010, .xc = 1.502500, .yc = 1.250000, .distance = -0.782920};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest52)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1.853600, .yc = -999.000000, .distance = -0.292890};
  maneuver expected2 = {.radius = 0.292890, .xc = 1.707100, .yc = 1.292900, .distance = -0.690110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest53)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.499970, .xc = 2.000000, .yc = 1.500000, .distance = -0.780360};
  maneuver expected2 = {.radius = -1000.000000, .xc = -8.500100, .yc = 1.397500, .distance = -0.005025};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest54)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.048284, .xc = 2.000000, .yc = 1.068300, .distance = -0.037922};
  maneuver expected2 = {.radius = -1000.000000, .xc = -705.370000, .yc = -705.840000, .distance = -0.658820};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest55)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -11.808000, .xc = 1.881900, .yc = -10.788000, .distance = -0.618230};
  maneuver expected2 = {.radius = 11.808000, .xc = 0.881920, .yc = 12.808000, .distance = -0.382070};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest56)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1.524100, .yc = -998.980000, .distance = -0.951720};
  maneuver expected2 = {.radius = -0.068284, .xc = 1.048300, .yc = 0.951720, .distance = -0.053630};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest57)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1.510000, .yc = -998.980000, .distance = -0.980000};
  maneuver expected2 = {.radius = -0.020000, .xc = 1.020000, .yc = 1.000000, .distance = -0.031416};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest58)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1.504100, .yc = -998.980000, .distance = -0.991720};
  maneuver expected2 = {.radius = -0.011716, .xc = 1.008300, .yc = 1.008300, .distance = -0.027605};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest59)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.010000, .xc = 2.000000, .yc = 1.010000, .distance = 0.031415};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1001.500000, .yc = -2886800000.000000, .distance = 1.000000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest60)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1.500000, .yc = -998.980000, .distance = -1.000000};
  maneuver expected2 = {.radius = -0.010002, .xc = 1.000000, .yc = 1.010000, .distance = -0.031423};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest61)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.371360, .xc = 2.000000, .yc = 0.648640, .distance = -0.538850};
  maneuver expected2 = {.radius = 0.371360, .xc = 1.262600, .yc = 0.737410, .distance = -1.413900};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest62)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.361950, .xc = 2.000000, .yc = 0.658050, .distance = -0.390480};
  maneuver expected2 = {.radius = 0.361950, .xc = 1.362000, .yc = 1.000000, .distance = -0.959030};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest63)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.553680, .xc = 2.000000, .yc = 0.466320, .distance = -0.322110};
  maneuver expected2 = {.radius = 0.553680, .xc = 1.391500, .yc = 1.391500, .distance = -0.756970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest64)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.631330, .xc = 2.006300, .yc = 0.368700, .distance = -0.575960};
  maneuver expected2 = {.radius = 0.631330, .xc = 1.000000, .yc = 1.131300, .distance = -0.582270};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest65)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -1.207100, .xc = 2.000000, .yc = -0.207110, .distance = -0.948060};
  maneuver expected2 = {.radius = -1000.000000, .xc = -706.030000, .yc = 707.680000, .distance = -0.207110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest66)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1.747500, .yc = -999.000000, .distance = -0.504980};
  maneuver expected2 = {.radius = -0.495050, .xc = 1.495000, .yc = 0.504950, .distance = -0.782570};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest67)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1.603600, .yc = -999.000000, .distance = -0.792890};
  maneuver expected2 = {.radius = -0.292890, .xc = 1.207100, .yc = 0.707110, .distance = -0.690110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest68)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.244990, .xc = 2.000000, .yc = 0.755010, .distance = 0.767220};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1001.500000, .yc = -99988.000000, .distance = 1.002500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest69)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.244990, .xc = 2.000000, .yc = 0.755010, .distance = 0.767220};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1001.500000, .yc = -99995.000000, .distance = 1.002500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest70)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.404690, .xc = 2.000000, .yc = 0.595310, .distance = -0.834290};
  maneuver expected2 = {.radius = 0.404690, .xc = 1.286200, .yc = 0.213840, .distance = -1.787800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest71)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.339480, .xc = 2.000000, .yc = 0.660520, .distance = -0.612540};
  maneuver expected2 = {.radius = 0.339480, .xc = 1.339500, .yc = 0.503390, .distance = -1.142400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest72)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.374190, .xc = 2.000000, .yc = 0.625810, .distance = -0.517980};
  maneuver expected2 = {.radius = 0.374190, .xc = 1.264600, .yc = 0.764590, .distance = -0.811870};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest73)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.279490, .xc = 2.197600, .yc = 0.802370, .distance = -0.557540};
  maneuver expected2 = {.radius = 0.279490, .xc = 2.000000, .yc = 0.279490, .distance = -0.777050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest74)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.355340, .xc = 2.253800, .yc = 0.751260, .distance = -0.552840};
  maneuver expected2 = {.radius = 0.355340, .xc = 1.748700, .yc = 0.251260, .distance = -0.556390};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest75)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.566900, .xc = 2.386700, .yc = 0.613280, .distance = -0.769240};
  maneuver expected2 = {.radius = 0.566900, .xc = 1.433100, .yc = 0.000000, .distance = -0.323990};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest76)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 709.100000, .yc = -706.110000, .distance = -0.007036};
  maneuver expected2 = {.radius = -0.700110, .xc = 2.490100, .yc = 0.499980, .distance = -1.106700};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest77)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 708.960000, .yc = -706.250000, .distance = -0.414210};
  maneuver expected2 = {.radius = -0.414210, .xc = 2.000000, .yc = 0.414210, .distance = -0.975970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest78)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 708.960000, .yc = -706.250000, .distance = -0.414210};
  maneuver expected2 = {.radius = -0.414210, .xc = 2.000000, .yc = 0.414210, .distance = -0.975970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest79)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.350010, .xc = 2.247500, .yc = 0.752510, .distance = 1.096100};
  maneuver expected2 = {.radius = 1000.000000, .xc = 71409.000000, .yc = -69993.000000, .distance = 0.710640};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest80)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.008284, .xc = 1.991700, .yc = 1.008300, .distance = 0.019519};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1002.000000, .yc = 0.504470, .distance = 1.008300};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest81)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.292290, .xc = 2.206700, .yc = 0.793320, .distance = -0.687660};
  maneuver expected2 = {.radius = 0.292290, .xc = 2.204600, .yc = 0.208740, .distance = -1.143900};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest82)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.291240, .xc = 2.205900, .yc = 0.794060, .distance = -0.840350};
  maneuver expected2 = {.radius = 0.291240, .xc = 2.500000, .yc = 0.291240, .distance = -1.069100};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest83)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.295130, .xc = 2.210800, .yc = 0.793410, .distance = -0.732830};
  maneuver expected2 = {.radius = 0.295130, .xc = 2.291300, .yc = 0.208690, .distance = -0.735780};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest84)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.374190, .xc = 2.264600, .yc = 0.735410, .distance = -0.811870};
  maneuver expected2 = {.radius = 0.374190, .xc = 2.125800, .yc = 0.000000, .distance = -0.517980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest85)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.360540, .xc = 2.254900, .yc = 0.745060, .distance = -0.569930};
  maneuver expected2 = {.radius = -1000.000000, .xc = -4.748700, .yc = -6.897600, .distance = -0.696550};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest86)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.585790, .xc = 2.414200, .yc = 0.585790, .distance = -1.380200};
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.457500, .yc = -1000.000000, .distance = -0.085786};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest87)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.585790, .xc = 2.414200, .yc = 0.585790, .distance = -1.380200};
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.456800, .yc = -1000.000000, .distance = -0.085787};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest88)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.528550, .xc = 2.373700, .yc = 0.626260, .distance = 1.655200};
  maneuver expected2 = {.radius = 1000.000000, .xc = 71410.000000, .yc = -69993.000000, .distance = 0.358860};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest89)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.292890, .xc = 2.207100, .yc = 0.792890, .distance = 0.690110};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1002.500000, .yc = 0.396770, .distance = 0.792890};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest90)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.352380, .xc = 2.249200, .yc = 0.750830, .distance = -1.106400};
  maneuver expected2 = {.radius = 0.352380, .xc = 2.746700, .yc = 0.251650, .distance = -1.656400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest91)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.566900, .xc = 2.386700, .yc = 0.613280, .distance = 0.769240};
  maneuver expected2 = {.radius = 0.566900, .xc = 3.000000, .yc = 1.566900, .distance = 0.323990};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest92)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.351800, .xc = 2.251200, .yc = 0.753740, .distance = 0.557890};
  maneuver expected2 = {.radius = 0.351800, .xc = 2.751200, .yc = 1.248800, .distance = 0.554370};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest93)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.279490, .xc = 2.197600, .yc = 0.802370, .distance = 0.557540};
  maneuver expected2 = {.radius = 0.279490, .xc = 2.720500, .yc = 1.000000, .distance = 0.777050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest94)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.293510, .xc = 2.207500, .yc = 0.792460, .distance = 0.692590};
  maneuver expected2 = {.radius = 0.293510, .xc = 2.794500, .yc = 0.790400, .distance = 1.156600};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest95)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.008285, .xc = 1.991700, .yc = 1.008300, .distance = -0.019520};
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.496200, .yc = -999.000000, .distance = -1.008300};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest96)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.008284, .xc = 1.991700, .yc = 1.008300, .distance = -0.019519};
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.495500, .yc = -999.000000, .distance = -1.008300};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest97)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -704.860000, .yc = 708.360000, .distance = 0.703570};
  maneuver expected2 = {.radius = -0.353560, .xc = 2.747500, .yc = 1.247500, .distance = 1.107200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest98)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -704.960000, .yc = 708.250000, .distance = 0.414210};
  maneuver expected2 = {.radius = -0.414210, .xc = 2.585800, .yc = 1.000000, .distance = 0.975970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest99)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.707070, .xc = 2.500000, .yc = 0.500030, .distance = 1.103600};
  maneuver expected2 = {.radius = 1000.000000, .xc = 9.997900, .yc = 8.144400, .distance = 0.007107};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest100)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -705.030000, .yc = 708.180000, .distance = 0.207110};
  maneuver expected2 = {.radius = -1.207100, .xc = 3.000000, .yc = 0.292890, .distance = 0.948060};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest101)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.870840, .xc = 2.621900, .yc = 0.390410, .distance = 0.573450};
  maneuver expected2 = {.radius = 0.870840, .xc = 2.384200, .yc = 2.115800, .distance = 0.564740};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest102)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.423020, .xc = 2.299100, .yc = 0.700880, .distance = 0.473800};
  maneuver expected2 = {.radius = 0.423020, .xc = 2.577000, .yc = 1.500000, .distance = 0.806040};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest103)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.353560, .xc = 2.250000, .yc = 0.749990, .distance = 0.557140};
  maneuver expected2 = {.radius = 0.353560, .xc = 2.752500, .yc = 1.247500, .distance = 1.116000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest104)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.404690, .xc = 2.286200, .yc = 0.713840, .distance = 0.754910};
  maneuver expected2 = {.radius = 0.404690, .xc = 3.000000, .yc = 1.095300, .distance = 1.708400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest105)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.404690, .xc = 2.286200, .yc = 0.713840, .distance = 0.754910};
  maneuver expected2 = {.radius = 0.404690, .xc = 3.000000, .yc = 1.095300, .distance = 1.708400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest106)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -704.730000, .yc = 708.480000, .distance = 1.058900};
  maneuver expected2 = {.radius = -0.176780, .xc = 2.873800, .yc = 1.623700, .distance = 0.553610};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest107)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -704.780000, .yc = 708.430000, .distance = 0.914210};
  maneuver expected2 = {.radius = -0.207110, .xc = 2.792900, .yc = 1.500000, .distance = 0.487980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest108)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -704.860000, .yc = 708.360000, .distance = 0.703550};
  maneuver expected2 = {.radius = -0.357120, .xc = 2.750000, .yc = 1.245000, .distance = 0.557400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest109)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.139750, .xc = 1.901200, .yc = 1.098800, .distance = 0.278770};
  maneuver expected2 = {.radius = -0.139750, .xc = 2.000000, .yc = 1.360300, .distance = 0.388520};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest110)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.177670, .xc = 1.873100, .yc = 1.124400, .distance = 0.276420};
  maneuver expected2 = {.radius = -0.177670, .xc = 2.125600, .yc = 1.374400, .distance = 0.278190};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest111)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.048284, .xc = 1.951700, .yc = 1.048300, .distance = 0.037922};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1002.000000, .yc = 1.273800, .distance = 0.451720};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest112)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -705.110000, .yc = 708.110000, .distance = 0.003518};
  maneuver expected2 = {.radius = 0.350050, .xc = 1.755000, .yc = 1.250000, .distance = 0.553360};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest113)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -705.030000, .yc = 708.180000, .distance = 0.207110};
  maneuver expected2 = {.radius = 0.207110, .xc = 2.000000, .yc = 1.292900, .distance = 0.487980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest114)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -705.030000, .yc = 708.180000, .distance = 0.207110};
  maneuver expected2 = {.radius = 0.207110, .xc = 2.000000, .yc = 1.292900, .distance = 0.487980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest115)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.175000, .xc = 1.876300, .yc = 1.123700, .distance = -0.548040};
  maneuver expected2 = {.radius = -1000.000000, .xc = -71405.000000, .yc = 69994.000000, .distance = -0.355320};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest116)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.180240, .xc = 1.858400, .yc = 1.141600, .distance = 0.444050};
  maneuver expected2 = {.radius = -0.180240, .xc = 1.819800, .yc = 1.500000, .distance = 0.868730};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest117)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.146150, .xc = 1.896700, .yc = 1.103300, .distance = 0.343830};
  maneuver expected2 = {.radius = -0.146150, .xc = 1.897700, .yc = 1.395600, .distance = 0.571930};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest118)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.197630, .xc = 1.860300, .yc = 1.139700, .distance = -0.549460};
  maneuver expected2 = {.radius = -0.197630, .xc = 1.500000, .yc = 1.302400, .distance = -0.394240};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest119)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.176780, .xc = 1.873800, .yc = 1.123700, .distance = 0.552720};
  maneuver expected2 = {.radius = -0.176780, .xc = 1.625000, .yc = 1.375000, .distance = 0.554490};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest120)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.197630, .xc = 1.860300, .yc = 1.139700, .distance = 0.549460};
  maneuver expected2 = {.radius = -0.197630, .xc = 1.697600, .yc = 1.500000, .distance = 0.394240};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest121)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.255740, .xc = 1.833300, .yc = 1.195000, .distance = 0.680940};
  maneuver expected2 = {.radius = -0.255740, .xc = 1.679000, .yc = 1.682600, .distance = 0.276670};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest122)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.292890, .xc = 1.792900, .yc = 1.207100, .distance = 0.690110};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1.646100, .yc = 1001.500000, .distance = 0.292890};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest123)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.292890, .xc = 1.792900, .yc = 1.207100, .distance = 0.690110};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1.646800, .yc = 1001.500000, .distance = 0.292890};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest124)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.353540, .xc = 1.750000, .yc = 1.250000, .distance = -1.107200};
  maneuver expected2 = {.radius = -1000.000000, .xc = -71406.000000, .yc = 69995.000000, .distance = -0.003536};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest125)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.292890, .xc = 1.792900, .yc = 1.207100, .distance = -0.690110};
  maneuver expected2 = {.radius = -1000.000000, .xc = -998.500000, .yc = 1.353200, .distance = -0.292890};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest126)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.013059, .xc = 2.004900, .yc = 1.023400, .distance = -0.020382};
  maneuver expected2 = {.radius = -1000.000000, .xc = -5.252600, .yc = -5.884900, .distance = -0.694210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest127)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.048284, .xc = 1.951700, .yc = 1.048300, .distance = -0.037922};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1.475900, .yc = -999.000000, .distance = -0.951720};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest128)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.351800, .xc = 1.748800, .yc = 1.246300, .distance = -0.557890};
  maneuver expected2 = {.radius = -0.351800, .xc = 1.248800, .yc = 0.751240, .distance = -0.554370};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest129)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.279490, .xc = 1.802400, .yc = 1.197600, .distance = -0.557540};
  maneuver expected2 = {.radius = -0.279490, .xc = 1.279500, .yc = 1.000000, .distance = -0.777050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest130)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.293510, .xc = 1.792500, .yc = 1.207500, .distance = -0.692590};
  maneuver expected2 = {.radius = -0.293510, .xc = 1.205500, .yc = 1.209600, .distance = -1.156600};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest131)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.366370, .xc = 1.726800, .yc = 1.273200, .distance = -0.909950};
  maneuver expected2 = {.radius = -0.366370, .xc = 1.000000, .yc = 1.366400, .distance = -1.773200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest132)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.366370, .xc = 1.726800, .yc = 1.273200, .distance = -0.909950};
  maneuver expected2 = {.radius = -0.366370, .xc = 1.000000, .yc = 1.366400, .distance = -1.773200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest133)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 708.860000, .yc = -706.360000, .distance = -0.703570};
  maneuver expected2 = {.radius = 0.353560, .xc = 1.252500, .yc = 0.752510, .distance = -1.107200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest134)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 708.960000, .yc = -706.250000, .distance = -0.414210};
  maneuver expected2 = {.radius = 0.414210, .xc = 1.414200, .yc = 1.000000, .distance = -0.975970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest135)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.707070, .xc = 1.500000, .yc = 1.500000, .distance = -1.103600};
  maneuver expected2 = {.radius = -1000.000000, .xc = -5.997900, .yc = -6.144400, .distance = -0.007107};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest136)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 709.030000, .yc = -706.180000, .distance = -0.207110};
  maneuver expected2 = {.radius = 1.207100, .xc = 1.000000, .yc = 1.707100, .distance = -0.948060};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest137)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.870840, .xc = 1.378100, .yc = 1.609600, .distance = -0.573450};
  maneuver expected2 = {.radius = -0.870840, .xc = 1.615800, .yc = -0.115780, .distance = -0.564740};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest138)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.423020, .xc = 1.700900, .yc = 1.299100, .distance = -0.473800};
  maneuver expected2 = {.radius = -0.423020, .xc = 1.423000, .yc = 0.500000, .distance = -0.806040};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest139)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.353560, .xc = 1.750000, .yc = 1.250000, .distance = -0.557140};
  maneuver expected2 = {.radius = -0.353560, .xc = 1.247500, .yc = 0.752490, .distance = -1.116000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest140)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.404690, .xc = 1.713800, .yc = 1.286200, .distance = -0.754910};
  maneuver expected2 = {.radius = -0.404690, .xc = 1.000000, .yc = 0.904690, .distance = -1.708400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest141)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.404690, .xc = 1.713800, .yc = 1.286200, .distance = -0.754910};
  maneuver expected2 = {.radius = -0.404690, .xc = 1.000000, .yc = 0.904690, .distance = -1.708400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest142)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 708.730000, .yc = -706.480000, .distance = -1.058900};
  maneuver expected2 = {.radius = 0.176780, .xc = 1.126200, .yc = 0.376250, .distance = -0.553610};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest143)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 708.780000, .yc = -706.430000, .distance = -0.914210};
  maneuver expected2 = {.radius = 0.207110, .xc = 1.207100, .yc = 0.500000, .distance = -0.487980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest144)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 708.860000, .yc = -706.360000, .distance = -0.703550};
  maneuver expected2 = {.radius = 0.357120, .xc = 1.250000, .yc = 0.755040, .distance = -0.557400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest145)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.361950, .xc = 2.342000, .yc = 1.000000, .distance = -0.390480};
  maneuver expected2 = {.radius = 0.361950, .xc = 2.000000, .yc = 0.361950, .distance = -0.959030};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest146)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.553680, .xc = 2.533700, .yc = 1.000000, .distance = -0.322110};
  maneuver expected2 = {.radius = 0.553680, .xc = 1.608500, .yc = 0.391510, .distance = -0.756970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest147)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -11.808000, .xc = 13.788000, .yc = 0.881920, .distance = -0.618230};
  maneuver expected2 = {.radius = 11.808000, .xc = -9.807700, .yc = -0.118080, .distance = -0.382070};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest148)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1002.000000, .yc = 0.523820, .distance = -0.951710};
  maneuver expected2 = {.radius = -0.068285, .xc = 2.048300, .yc = 0.048285, .distance = -0.053631};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest149)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1002.000000, .yc = 0.509670, .distance = -0.980000};
  maneuver expected2 = {.radius = -0.020000, .xc = 2.000000, .yc = 0.020000, .distance = -0.031416};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest150)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1002.000000, .yc = 0.509670, .distance = -0.980000};
  maneuver expected2 = {.radius = -0.020000, .xc = 2.000000, .yc = 0.020000, .distance = -0.031416};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest151)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1002.000000, .yc = 0.503820, .distance = -0.991720};
  maneuver expected2 = {.radius = -0.011716, .xc = 1.991700, .yc = 0.008284, .distance = -0.027605};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest152)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.010000, .xc = 1.990000, .yc = 1.000000, .distance = 0.031415};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1530000000.000000, .yc = 500.500000, .distance = 1.000000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest153)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.371360, .xc = 2.351400, .yc = 1.000000, .distance = -0.538850};
  maneuver expected2 = {.radius = 0.371360, .xc = 2.262600, .yc = 0.262590, .distance = -1.413900};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest154)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.339480, .xc = 2.339500, .yc = 1.000000, .distance = -0.612540};
  maneuver expected2 = {.radius = 0.339480, .xc = 2.496600, .yc = 0.339470, .distance = -1.142400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest155)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.374190, .xc = 2.374200, .yc = 1.000000, .distance = -0.517980};
  maneuver expected2 = {.radius = 0.374190, .xc = 2.235400, .yc = 0.264590, .distance = -0.811870};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest156)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.631330, .xc = 2.631300, .yc = 1.006300, .distance = -0.575960};
  maneuver expected2 = {.radius = 0.631330, .xc = 1.868700, .yc = 0.000000, .distance = -0.582270};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest157)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -1.207100, .xc = 3.207100, .yc = 1.000000, .distance = -0.948060};
  maneuver expected2 = {.radius = -1000.000000, .xc = -704.680000, .yc = -707.030000, .distance = -0.207110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest158)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1002.000000, .yc = 0.747190, .distance = -0.504980};
  maneuver expected2 = {.radius = -0.495050, .xc = 2.495000, .yc = 0.495020, .distance = -0.782570};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest159)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1002.000000, .yc = 0.747190, .distance = -0.504970};
  maneuver expected2 = {.radius = -0.495050, .xc = 2.495000, .yc = 0.495030, .distance = -0.782570};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest160)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1002.000000, .yc = 0.603230, .distance = -0.792890};
  maneuver expected2 = {.radius = -0.292890, .xc = 2.292900, .yc = 0.207110, .distance = -0.690110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest161)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.244990, .xc = 2.245000, .yc = 1.000000, .distance = 0.767220};
  maneuver expected2 = {.radius = 1000.000000, .xc = 99988.000000, .yc = 1000.400000, .distance = 1.002500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest162)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.404690, .xc = 2.404700, .yc = 1.000000, .distance = -0.834300};
  maneuver expected2 = {.radius = 0.404690, .xc = 2.786200, .yc = 0.286160, .distance = -1.787800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest163)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.360630, .xc = 2.360600, .yc = 1.020000, .distance = 0.955580};
  maneuver expected2 = {.radius = 0.360630, .xc = 2.996400, .yc = 1.360600, .distance = 0.392710};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest164)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.279490, .xc = 2.279500, .yc = 1.000000, .distance = 0.777050};
  maneuver expected2 = {.radius = 0.279490, .xc = 2.802400, .yc = 1.197600, .distance = 0.557540};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest165)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.250010, .xc = 2.250000, .yc = 1.002500, .distance = -0.781670};
  maneuver expected2 = {.radius = 0.250010, .xc = 2.750000, .yc = 1.000000, .distance = -0.784170};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest166)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.279490, .xc = 2.279500, .yc = 1.000000, .distance = -0.777050};
  maneuver expected2 = {.radius = 0.279490, .xc = 2.802400, .yc = 0.802370, .distance = -0.557540};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest167)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.029702, .xc = 2.029700, .yc = 1.020000, .distance = -0.046953};
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.615000, .yc = -9.005400, .distance = -0.970050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest168)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.029701, .xc = 2.029700, .yc = 1.020000, .distance = -0.046952};
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.615000, .yc = -9.004700, .distance = -0.970050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest169)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.414210, .xc = 2.414200, .yc = 1.000000, .distance = -0.975970};
  maneuver expected2 = {.radius = -1000.000000, .xc = 709.960000, .yc = -706.250000, .distance = -0.414210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest170)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.499990, .xc = 2.500000, .yc = 1.000000, .distance = 1.565800};
  maneuver expected2 = {.radius = 1000.000000, .xc = 99988.000000, .yc = 1000.900000, .distance = 0.005000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest171)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.414210, .xc = 2.414200, .yc = 1.000000, .distance = 0.975970};
  maneuver expected2 = {.radius = 1000.000000, .xc = 709.960000, .yc = 708.250000, .distance = 0.414210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest172)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.494920, .xc = 2.494900, .yc = 1.000000, .distance = 0.772480};
  maneuver expected2 = {.radius = 1000.000000, .xc = 2.645000, .yc = 11.498000, .distance = 0.510050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest173)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.423020, .xc = 2.423000, .yc = 1.000000, .distance = 0.806040};
  maneuver expected2 = {.radius = 0.423020, .xc = 2.700900, .yc = 1.799100, .distance = 0.473800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest174)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.311730, .xc = 2.311700, .yc = 1.003100, .distance = 0.694940};
  maneuver expected2 = {.radius = 0.311730, .xc = 2.688300, .yc = 1.500000, .distance = 0.691820};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest175)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.291240, .xc = 2.291200, .yc = 1.000000, .distance = 0.760850};
  maneuver expected2 = {.radius = 0.291240, .xc = 2.794100, .yc = 1.294100, .distance = 0.989590};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest176)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.340760, .xc = 2.340800, .yc = 1.000000, .distance = 0.990150};
  maneuver expected2 = {.radius = 0.340760, .xc = 3.003400, .yc = 1.159300, .distance = 1.528800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest177)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.340760, .xc = 2.340800, .yc = 1.000000, .distance = 0.990150};
  maneuver expected2 = {.radius = 0.340760, .xc = 3.003400, .yc = 1.159300, .distance = 1.528800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest178)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.207110, .xc = 2.207100, .yc = 1.000000, .distance = -0.487980};
  maneuver expected2 = {.radius = -1000.000000, .xc = 709.790000, .yc = -705.930000, .distance = -0.914210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest179)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -998.000000, .yc = 1.247800, .distance = 0.495000};
  maneuver expected2 = {.radius = -0.500010, .xc = 2.500000, .yc = 1.495000, .distance = 1.565800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest180)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -998.000000, .yc = 1.043200, .distance = 0.085787};
  maneuver expected2 = {.radius = -0.585790, .xc = 2.585800, .yc = 1.085800, .distance = 1.380200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest181)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -998.020000, .yc = 1.240300, .distance = 0.480000};
  maneuver expected2 = {.radius = -0.020000, .xc = 2.000000, .yc = 1.480000, .distance = 0.031416};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest182)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -998.020000, .yc = 1.226200, .distance = 0.451720};
  maneuver expected2 = {.radius = -0.068284, .xc = 2.048300, .yc = 1.451700, .distance = 0.053630};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest183)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -3.082700, .xc = 5.062500, .yc = 0.969170, .distance = 0.219450};
  maneuver expected2 = {.radius = 3.082700, .xc = -1.082500, .yc = 1.469200, .distance = 0.281100};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest184)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.268480, .xc = 2.248500, .yc = 1.000000, .distance = 0.165330};
  maneuver expected2 = {.radius = 0.268480, .xc = 1.810200, .yc = 1.310200, .distance = 0.376190};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest185)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.179090, .xc = 2.159100, .yc = 1.000000, .distance = 0.198890};
  maneuver expected2 = {.radius = 0.179090, .xc = 2.000000, .yc = 1.320900, .distance = 0.480200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest186)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.179090, .xc = 2.159100, .yc = 1.000000, .distance = 0.198890};
  maneuver expected2 = {.radius = 0.179090, .xc = 2.000000, .yc = 1.320900, .distance = 0.480200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest187)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.185290, .xc = 2.165300, .yc = 1.000000, .distance = 0.273890};
  maneuver expected2 = {.radius = 0.185290, .xc = 2.131000, .yc = 1.369000, .distance = 0.710460};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest188)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -998.020000, .yc = 1.250300, .distance = 0.500000};
  maneuver expected2 = {.radius = -0.009997, .xc = 1.990000, .yc = 1.500000, .distance = 0.031408};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest189)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -998.020000, .yc = 1.246200, .distance = 0.491720};
  maneuver expected2 = {.radius = -0.011716, .xc = 1.991700, .yc = 1.491700, .distance = 0.027604};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest190)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.206680, .xc = 1.793300, .yc = 1.000000, .distance = 0.486250};
  maneuver expected2 = {.radius = -0.206680, .xc = 1.502100, .yc = 1.293300, .distance = 0.808840};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest191)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.197630, .xc = 1.802400, .yc = 1.000000, .distance = 0.394240};
  maneuver expected2 = {.radius = -0.197630, .xc = 1.639700, .yc = 1.360300, .distance = 0.549460};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest192)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.251260, .xc = 1.748800, .yc = 0.997490, .distance = 0.390910};
  maneuver expected2 = {.radius = -0.251260, .xc = 1.751300, .yc = 1.500000, .distance = 0.393430};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest193)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.048283, .xc = 1.931700, .yc = 1.000000, .distance = 0.037921};
  maneuver expected2 = {.radius = 1000.000000, .xc = 708.840000, .yc = 708.370000, .distance = 0.658820};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest194)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -998.000000, .yc = 1.002800, .distance = 0.004975};
  maneuver expected2 = {.radius = 0.495050, .xc = 1.505000, .yc = 1.005000, .distance = 0.782570};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest195)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -998.000000, .yc = 1.002800, .distance = 0.004975};
  maneuver expected2 = {.radius = 0.495050, .xc = 1.505000, .yc = 1.005000, .distance = 0.782570};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest196)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -998.000000, .yc = 1.146800, .distance = 0.292890};
  maneuver expected2 = {.radius = 0.292890, .xc = 1.707100, .yc = 1.292900, .distance = 0.690110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest197)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.247490, .xc = 1.752500, .yc = 1.000000, .distance = -0.775050};
  maneuver expected2 = {.radius = -1000.000000, .xc = -99984.000000, .yc = -998.670000, .distance = -0.502500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest198)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.257330, .xc = 1.722700, .yc = 1.000000, .distance = 0.637010};
  maneuver expected2 = {.radius = -0.257330, .xc = 1.318000, .yc = 1.318000, .distance = 1.243300};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest199)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.010100, .xc = 1.989900, .yc = 1.020000, .distance = -0.015764};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1.595000, .yc = -8.995200, .distance = -0.990050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest200)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.279490, .xc = 1.720500, .yc = 1.000000, .distance = -0.777050};
  maneuver expected2 = {.radius = -0.279490, .xc = 1.197600, .yc = 0.802370, .distance = -0.557540};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest201)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.250010, .xc = 1.750000, .yc = 0.997500, .distance = 0.781670};
  maneuver expected2 = {.radius = -0.250010, .xc = 1.250000, .yc = 1.000000, .distance = 0.784170};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest202)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.279490, .xc = 1.720500, .yc = 1.000000, .distance = 0.777050};
  maneuver expected2 = {.radius = -0.279490, .xc = 1.197600, .yc = 1.197600, .distance = 0.557540};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest203)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.363280, .xc = 1.636700, .yc = 1.020000, .distance = 0.962510};
  maneuver expected2 = {.radius = -0.363280, .xc = 0.996370, .yc = 1.363300, .distance = 0.388250};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest204)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.363280, .xc = 1.636700, .yc = 1.020000, .distance = 0.962510};
  maneuver expected2 = {.radius = -0.363280, .xc = 0.996370, .yc = 1.363300, .distance = 0.388250};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest205)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.414210, .xc = 1.585800, .yc = 1.000000, .distance = 0.975970};
  maneuver expected2 = {.radius = 1000.000000, .xc = -705.960000, .yc = 708.250000, .distance = 0.414210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest206)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.499990, .xc = 1.500000, .yc = 1.000000, .distance = -1.565800};
  maneuver expected2 = {.radius = -1000.000000, .xc = -99984.000000, .yc = -998.920000, .distance = -0.005000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest207)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.414210, .xc = 1.585800, .yc = 1.000000, .distance = -0.975970};
  maneuver expected2 = {.radius = -1000.000000, .xc = -705.960000, .yc = -706.250000, .distance = -0.414210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest208)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.494920, .xc = 1.505100, .yc = 1.000000, .distance = -0.772480};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1.355000, .yc = -9.497600, .distance = -0.510050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest209)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.423020, .xc = 1.577000, .yc = 1.000000, .distance = -0.806040};
  maneuver expected2 = {.radius = -0.423020, .xc = 1.299100, .yc = 0.200880, .distance = -0.473800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest210)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.311730, .xc = 1.688300, .yc = 0.996880, .distance = -0.694940};
  maneuver expected2 = {.radius = -0.311730, .xc = 1.311700, .yc = 0.500000, .distance = -0.691820};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest211)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.291240, .xc = 1.708800, .yc = 1.000000, .distance = -0.760850};
  maneuver expected2 = {.radius = -0.291240, .xc = 1.205900, .yc = 0.705940, .distance = -0.989590};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest212)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.340760, .xc = 1.659200, .yc = 1.000000, .distance = -0.990150};
  maneuver expected2 = {.radius = -0.340760, .xc = 0.996590, .yc = 0.840740, .distance = -1.528800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest213)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.340760, .xc = 1.659200, .yc = 1.000000, .distance = -0.990150};
  maneuver expected2 = {.radius = -0.340760, .xc = 0.996590, .yc = 0.840740, .distance = -1.528800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest214)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.207110, .xc = 1.792900, .yc = 1.000000, .distance = 0.487980};
  maneuver expected2 = {.radius = 1000.000000, .xc = -705.790000, .yc = 707.930000, .distance = 0.914210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest215)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1002.000000, .yc = 0.752170, .distance = -0.495000};
  maneuver expected2 = {.radius = 0.500010, .xc = 1.500000, .yc = 0.505000, .distance = -1.565800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest216)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 1002.000000, .yc = 0.956780, .distance = -0.085787};
  maneuver expected2 = {.radius = 0.585790, .xc = 1.414200, .yc = 0.914210, .distance = -1.380200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest217)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 709.250000, .yc = 707.960000, .distance = -0.414210};
  maneuver expected2 = {.radius = 0.414210, .xc = 2.000000, .yc = 0.414210, .distance = -0.975970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest218)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.707070, .xc = 1.500000, .yc = 0.500030, .distance = -1.103600};
  maneuver expected2 = {.radius = -1000.000000, .xc = 9.144400, .yc = -6.997900, .distance = -0.007106};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest219)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.048285, .xc = 1.951700, .yc = 0.951710, .distance = -0.037923};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1002.000000, .yc = 0.475530, .distance = -0.951710};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest220)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.351800, .xc = 1.753700, .yc = 0.748760, .distance = -0.557890};
  maneuver expected2 = {.radius = -0.351800, .xc = 2.248800, .yc = 0.248760, .distance = -0.554370};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest221)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.279490, .xc = 1.802400, .yc = 0.802370, .distance = -0.557540};
  maneuver expected2 = {.radius = -0.279490, .xc = 2.000000, .yc = 0.279490, .distance = -0.777050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest222)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.279490, .xc = 1.802400, .yc = 0.802370, .distance = -0.557540};
  maneuver expected2 = {.radius = -0.279490, .xc = 2.000000, .yc = 0.279490, .distance = -0.777050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest223)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.293510, .xc = 1.792500, .yc = 0.792460, .distance = -0.692590};
  maneuver expected2 = {.radius = -0.293510, .xc = 1.790400, .yc = 0.205450, .distance = -1.156600};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest224)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.366370, .xc = 1.726800, .yc = 0.726790, .distance = -0.909950};
  maneuver expected2 = {.radius = -0.366370, .xc = 1.633600, .yc = -0.000000, .distance = -1.773200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest225)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 709.360000, .yc = 707.860000, .distance = -0.703570};
  maneuver expected2 = {.radius = 0.353560, .xc = 2.247500, .yc = 0.252490, .distance = -1.107200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest226)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 709.430000, .yc = 707.780000, .distance = -0.914210};
  maneuver expected2 = {.radius = 0.207110, .xc = 2.500000, .yc = 0.207110, .distance = -0.487980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest227)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 709.360000, .yc = 707.860000, .distance = -0.703550};
  maneuver expected2 = {.radius = 0.357120, .xc = 2.245000, .yc = 0.249990, .distance = -0.557400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest228)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 709.180000, .yc = 708.030000, .distance = -0.207110};
  maneuver expected2 = {.radius = 1.207100, .xc = 1.292900, .yc = 0.000000, .distance = -0.948060};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest229)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.870840, .xc = 1.390400, .yc = 0.378090, .distance = -0.573450};
  maneuver expected2 = {.radius = -0.870840, .xc = 3.115800, .yc = 0.615780, .distance = -0.564740};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest230)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.423020, .xc = 1.700900, .yc = 0.700880, .distance = -0.473800};
  maneuver expected2 = {.radius = -0.423020, .xc = 2.500000, .yc = 0.423020, .distance = -0.806040};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest231)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.423030, .xc = 1.700900, .yc = 0.700880, .distance = -0.473800};
  maneuver expected2 = {.radius = -0.423030, .xc = 2.500000, .yc = 0.423030, .distance = -0.806040};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest232)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.353560, .xc = 1.750000, .yc = 0.749990, .distance = -0.557140};
  maneuver expected2 = {.radius = -0.353560, .xc = 2.247500, .yc = 0.247490, .distance = -1.116000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest233)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.404690, .xc = 1.713800, .yc = 0.713840, .distance = -0.754910};
  maneuver expected2 = {.radius = -0.404690, .xc = 2.095300, .yc = -0.000000, .distance = -1.708400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest234)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 709.480000, .yc = 707.730000, .distance = -1.058900};
  maneuver expected2 = {.radius = 0.176780, .xc = 2.623700, .yc = 0.126250, .distance = -0.553610};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest235)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.008284, .xc = 1.991700, .yc = 0.991720, .distance = 0.019519};
  maneuver expected2 = {.radius = 1000.000000, .xc = 2.495900, .yc = 1001.000000, .distance = 1.008300};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest236)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.292290, .xc = 2.206700, .yc = 1.206700, .distance = -0.687660};
  maneuver expected2 = {.radius = 0.292290, .xc = 2.791300, .yc = 1.204600, .distance = -1.143900};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest237)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.279490, .xc = 2.197600, .yc = 1.197600, .distance = -0.557540};
  maneuver expected2 = {.radius = 0.279490, .xc = 2.720500, .yc = 1.000000, .distance = -0.777050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest238)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.355340, .xc = 2.248700, .yc = 1.253800, .distance = -0.552840};
  maneuver expected2 = {.radius = 0.355340, .xc = 2.748700, .yc = 0.748740, .distance = -0.556390};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest239)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.566900, .xc = 2.386700, .yc = 1.386700, .distance = -0.769240};
  maneuver expected2 = {.radius = 0.566900, .xc = 3.000000, .yc = 0.433100, .distance = -0.323990};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest240)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.566900, .xc = 2.386700, .yc = 1.386700, .distance = -0.769240};
  maneuver expected2 = {.radius = 0.566900, .xc = 3.000000, .yc = 0.433100, .distance = -0.323990};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest241)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 709.110000, .yc = 708.100000, .distance = -0.007036};
  maneuver expected2 = {.radius = -0.700110, .xc = 2.500000, .yc = 1.490100, .distance = -1.106700};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest242)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 709.250000, .yc = 707.960000, .distance = -0.414210};
  maneuver expected2 = {.radius = -0.414210, .xc = 2.585800, .yc = 1.000000, .distance = -0.975970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest243)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.350010, .xc = 2.247500, .yc = 1.247500, .distance = 1.096100};
  maneuver expected2 = {.radius = 1000.000000, .xc = 69996.000000, .yc = 71408.000000, .distance = 0.710640};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest244)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.292890, .xc = 2.207100, .yc = 1.207100, .distance = 0.690110};
  maneuver expected2 = {.radius = 1000.000000, .xc = 2.603600, .yc = 1001.500000, .distance = 0.792890};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest245)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.352380, .xc = 2.249200, .yc = 1.249200, .distance = -1.106400};
  maneuver expected2 = {.radius = 0.352380, .xc = 2.748400, .yc = 1.746700, .distance = -1.656400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest246)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.291240, .xc = 2.205900, .yc = 1.205900, .distance = -0.840350};
  maneuver expected2 = {.radius = 0.291240, .xc = 2.708800, .yc = 1.500000, .distance = -1.069100};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest247)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.295130, .xc = 2.206600, .yc = 1.210800, .distance = -0.732830};
  maneuver expected2 = {.radius = 0.295130, .xc = 2.791300, .yc = 1.291300, .distance = -0.735780};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest248)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.374190, .xc = 2.264600, .yc = 1.264600, .distance = -0.811870};
  maneuver expected2 = {.radius = 0.374190, .xc = 3.000000, .yc = 1.125800, .distance = -0.517980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest249)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.374190, .xc = 2.264600, .yc = 1.264600, .distance = -0.811870};
  maneuver expected2 = {.radius = 0.374190, .xc = 3.000000, .yc = 1.125800, .distance = -0.517980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest250)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.360540, .xc = 2.254900, .yc = 1.254900, .distance = -0.569940};
  maneuver expected2 = {.radius = -1000.000000, .xc = 9.898600, .yc = -5.749600, .distance = -0.696550};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest251)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.585790, .xc = 2.414200, .yc = 1.414200, .distance = -1.380200};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1003.000000, .yc = 1.457400, .distance = -0.085787};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest252)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.528550, .xc = 2.373700, .yc = 1.373700, .distance = 1.655200};
  maneuver expected2 = {.radius = 1000.000000, .xc = 69996.000000, .yc = 71409.000000, .distance = 0.358860};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest253)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -705.180000, .yc = -706.030000, .distance = 0.207110};
  maneuver expected2 = {.radius = -0.207110, .xc = 2.000000, .yc = 1.292900, .distance = 0.487980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest254)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.353540, .xc = 2.250000, .yc = 1.250000, .distance = 0.551800};
  maneuver expected2 = {.radius = 1000.000000, .xc = -5.143100, .yc = 8.499200, .distance = 0.003553};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest255)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.281360, .xc = 2.184800, .yc = 1.184800, .distance = 0.388260};
  maneuver expected2 = {.radius = 0.281360, .xc = 1.718600, .yc = 1.500000, .distance = 0.167270};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest256)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.175900, .xc = 2.123100, .yc = 1.125600, .distance = 0.278940};
  maneuver expected2 = {.radius = 0.175900, .xc = 1.875600, .yc = 1.375600, .distance = 0.277190};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest257)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.139750, .xc = 2.098800, .yc = 1.098800, .distance = 0.278770};
  maneuver expected2 = {.radius = 0.139750, .xc = 2.000000, .yc = 1.360300, .distance = 0.388520};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest258)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.139750, .xc = 2.098800, .yc = 1.098800, .distance = 0.278770};
  maneuver expected2 = {.radius = 0.139750, .xc = 2.000000, .yc = 1.360300, .distance = 0.388520};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest259)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.146750, .xc = 2.103800, .yc = 1.103800, .distance = 0.346290};
  maneuver expected2 = {.radius = 0.146750, .xc = 2.104800, .yc = 1.397300, .distance = 0.578280};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest260)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.008284, .xc = 1.991700, .yc = 0.991720, .distance = -0.019520};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1002.000000, .yc = 1.246200, .distance = -0.508280};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest261)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -705.230000, .yc = -705.980000, .distance = 0.351790};
  maneuver expected2 = {.radius = -0.176780, .xc = 1.876300, .yc = 1.373800, .distance = 0.553610};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest262)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -705.370000, .yc = -705.870000, .distance = 0.698820};
  maneuver expected2 = {.radius = -0.011716, .xc = 1.500000, .yc = 1.488300, .distance = 0.027604};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest263)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -705.360000, .yc = -705.880000, .distance = 0.687110};
  maneuver expected2 = {.radius = -0.020000, .xc = 1.514100, .yc = 1.485900, .distance = 0.031415};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest264)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -705.350000, .yc = -705.890000, .distance = 0.658820};
  maneuver expected2 = {.radius = -0.068283, .xc = 1.568300, .yc = 1.500000, .distance = 0.053629};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest265)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -6.071100, .xc = 6.321500, .yc = 5.235600, .distance = 0.293040};
  maneuver expected2 = {.radius = 6.071100, .xc = -2.749800, .yc = -2.835600, .distance = 0.414460};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest266)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.386500, .xc = 2.259200, .yc = 1.259200, .distance = 0.230290};
  maneuver expected2 = {.radius = 0.386500, .xc = 1.500000, .yc = 1.113500, .distance = 0.533850};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest267)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.386500, .xc = 2.259200, .yc = 1.259200, .distance = 0.230290};
  maneuver expected2 = {.radius = 0.386500, .xc = 1.500000, .yc = 1.113500, .distance = 0.533850};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest268)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.254810, .xc = 2.166000, .yc = 1.166000, .distance = 0.278240};
  maneuver expected2 = {.radius = 0.254810, .xc = 1.680200, .yc = 1.319800, .distance = 0.678490};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest269)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.262340, .xc = 2.171400, .yc = 1.171400, .distance = 0.383610};
  maneuver expected2 = {.radius = 0.262340, .xc = 1.762300, .yc = 1.500000, .distance = 1.001700};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest270)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -705.370000, .yc = -705.870000, .distance = 0.707110};
  maneuver expected2 = {.radius = -0.010000, .xc = 1.492900, .yc = 1.492900, .distance = 0.031417};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest271)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.366370, .xc = 1.726800, .yc = 0.726790, .distance = 0.909950};
  maneuver expected2 = {.radius = -0.366370, .xc = 1.000000, .yc = 0.633630, .distance = 1.773200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest272)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.292290, .xc = 1.793300, .yc = 0.793320, .distance = 0.687660};
  maneuver expected2 = {.radius = -0.292290, .xc = 1.208700, .yc = 0.795400, .distance = 1.143900};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest273)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.279490, .xc = 1.802400, .yc = 0.802370, .distance = 0.557540};
  maneuver expected2 = {.radius = -0.279490, .xc = 1.279500, .yc = 1.000000, .distance = 0.777050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest274)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.355340, .xc = 1.751300, .yc = 0.746240, .distance = 0.552840};
  maneuver expected2 = {.radius = -0.355340, .xc = 1.251300, .yc = 1.251300, .distance = 0.556390};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest275)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.048285, .xc = 1.951700, .yc = 0.951720, .distance = 0.037923};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1.475500, .yc = 1001.000000, .distance = 0.951710};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest276)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.048283, .xc = 1.951700, .yc = 0.951720, .distance = 0.037921};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1.476200, .yc = 1001.000000, .distance = 0.951720};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest277)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -705.110000, .yc = -706.100000, .distance = 0.007036};
  maneuver expected2 = {.radius = 0.700110, .xc = 1.500000, .yc = 0.509930, .distance = 1.106700};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest278)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = -705.250000, .yc = -705.960000, .distance = 0.414210};
  maneuver expected2 = {.radius = 0.414210, .xc = 1.414200, .yc = 1.000000, .distance = 0.975970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest279)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.350010, .xc = 1.752500, .yc = 0.752510, .distance = -1.096100};
  maneuver expected2 = {.radius = -1000.000000, .xc = -69992.000000, .yc = -71406.000000, .distance = -0.710640};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest280)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.292890, .xc = 1.792900, .yc = 0.792890, .distance = -0.690110};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1.396400, .yc = -999.500000, .distance = -0.792890};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest281)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.352380, .xc = 1.750800, .yc = 0.750830, .distance = 1.106400};
  maneuver expected2 = {.radius = -0.352380, .xc = 1.251600, .yc = 0.253330, .distance = 1.656400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest282)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.291240, .xc = 1.794100, .yc = 0.794060, .distance = 0.840350};
  maneuver expected2 = {.radius = -0.291240, .xc = 1.291200, .yc = 0.500000, .distance = 1.069100};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest283)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.295130, .xc = 1.793400, .yc = 0.789240, .distance = 0.732830};
  maneuver expected2 = {.radius = -0.295130, .xc = 1.208700, .yc = 0.708690, .distance = 0.735780};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest284)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.374190, .xc = 1.735400, .yc = 0.735410, .distance = 0.811870};
  maneuver expected2 = {.radius = -0.374190, .xc = 1.000000, .yc = 0.874190, .distance = 0.517980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest285)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.374190, .xc = 1.735400, .yc = 0.735410, .distance = 0.811870};
  maneuver expected2 = {.radius = -0.374190, .xc = 1.000000, .yc = 0.874190, .distance = 0.517980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest286)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.360540, .xc = 1.745100, .yc = 0.745060, .distance = 0.569940};
  maneuver expected2 = {.radius = 1000.000000, .xc = -5.898600, .yc = 7.749600, .distance = 0.696550};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest287)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.585790, .xc = 1.585800, .yc = 0.585790, .distance = 1.380200};
  maneuver expected2 = {.radius = 1000.000000, .xc = -999.000000, .yc = 0.542570, .distance = 0.085787};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest288)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.528550, .xc = 1.626300, .yc = 0.626260, .distance = -1.655200};
  maneuver expected2 = {.radius = -1000.000000, .xc = -69992.000000, .yc = -71407.000000, .distance = -0.358860};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest289)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.499990, .xc = 2.000000, .yc = 0.500010, .distance = -1.565800};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1002.000000, .yc = -99995.000000, .distance = -0.005000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest290)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.414210, .xc = 2.000000, .yc = 0.585790, .distance = -0.975970};
  maneuver expected2 = {.radius = -1000.000000, .xc = 709.250000, .yc = -706.960000, .distance = -0.414210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest291)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.010100, .xc = 1.980000, .yc = 0.989900, .distance = -0.015765};
  maneuver expected2 = {.radius = -1000.000000, .xc = 11.994000, .yc = 0.594990, .distance = -0.990050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest292)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.279490, .xc = 2.000000, .yc = 0.720510, .distance = -0.777050};
  maneuver expected2 = {.radius = -0.279490, .xc = 2.197600, .yc = 0.197630, .distance = -0.557540};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest293)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.250010, .xc = 2.002500, .yc = 0.750010, .distance = 0.781670};
  maneuver expected2 = {.radius = -0.250010, .xc = 2.000000, .yc = 0.250010, .distance = 0.784170};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest294)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.250000, .xc = 2.002500, .yc = 0.750010, .distance = 0.781660};
  maneuver expected2 = {.radius = -0.250000, .xc = 2.000000, .yc = 0.250000, .distance = 0.784160};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest295)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.279490, .xc = 2.000000, .yc = 0.720510, .distance = 0.777050};
  maneuver expected2 = {.radius = -0.279490, .xc = 1.802400, .yc = 0.197630, .distance = 0.557540};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest296)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.363280, .xc = 1.980000, .yc = 0.636720, .distance = 0.962510};
  maneuver expected2 = {.radius = -0.363280, .xc = 1.636700, .yc = -0.003633, .distance = 0.388250};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest297)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.414210, .xc = 2.000000, .yc = 0.585790, .distance = 0.975970};
  maneuver expected2 = {.radius = 1000.000000, .xc = -705.250000, .yc = -706.960000, .distance = 0.414210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest298)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 2.247200, .yc = 1001.000000, .distance = -0.495000};
  maneuver expected2 = {.radius = 0.500010, .xc = 2.495000, .yc = 0.499990, .distance = -1.565800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest299)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 2.042500, .yc = 1001.000000, .distance = -0.085786};
  maneuver expected2 = {.radius = 0.585790, .xc = 2.085800, .yc = 0.414210, .distance = -1.380200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest300)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.494920, .xc = 2.000000, .yc = 0.505080, .distance = -0.772480};
  maneuver expected2 = {.radius = -1000.000000, .xc = 12.497000, .yc = 0.355000, .distance = -0.510050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest301)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.423020, .xc = 2.000000, .yc = 0.576980, .distance = -0.806040};
  maneuver expected2 = {.radius = -0.423020, .xc = 2.799100, .yc = 0.299120, .distance = -0.473800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest302)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.311730, .xc = 2.003100, .yc = 0.688290, .distance = -0.694940};
  maneuver expected2 = {.radius = -0.311730, .xc = 2.500000, .yc = 0.311730, .distance = -0.691820};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest303)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.311730, .xc = 2.003100, .yc = 0.688290, .distance = -0.694930};
  maneuver expected2 = {.radius = -0.311730, .xc = 2.500000, .yc = 0.311730, .distance = -0.691810};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest304)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.291240, .xc = 2.000000, .yc = 0.708760, .distance = -0.760850};
  maneuver expected2 = {.radius = -0.291240, .xc = 2.294100, .yc = 0.205940, .distance = -0.989590};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest305)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.340760, .xc = 2.000000, .yc = 0.659240, .distance = -0.990150};
  maneuver expected2 = {.radius = -0.340760, .xc = 2.159300, .yc = -0.003408, .distance = -1.528800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest306)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.207110, .xc = 2.000000, .yc = 0.792890, .distance = 0.487980};
  maneuver expected2 = {.radius = 1000.000000, .xc = -704.930000, .yc = -706.780000, .distance = 0.914210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest307)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 2.499700, .yc = 1001.000000, .distance = -1.000000};
  maneuver expected2 = {.radius = -0.010002, .xc = 3.000000, .yc = 0.990000, .distance = -0.031422};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest308)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.371360, .xc = 2.000000, .yc = 1.351400, .distance = -0.538850};
  maneuver expected2 = {.radius = 0.371360, .xc = 2.737400, .yc = 1.262600, .distance = -1.413900};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest309)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.361950, .xc = 2.000000, .yc = 1.342000, .distance = -0.390480};
  maneuver expected2 = {.radius = 0.361950, .xc = 2.638000, .yc = 1.000000, .distance = -0.959030};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest310)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.553680, .xc = 2.000000, .yc = 1.533700, .distance = -0.322110};
  maneuver expected2 = {.radius = 0.553680, .xc = 2.608500, .yc = 0.608490, .distance = -0.756970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest311)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -11.809000, .xc = 2.118100, .yc = 12.788000, .distance = -0.618240};
  maneuver expected2 = {.radius = 11.809000, .xc = 3.118100, .yc = -10.808000, .distance = -0.382060};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest312)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -11.808000, .xc = 2.118100, .yc = 12.788000, .distance = -0.618230};
  maneuver expected2 = {.radius = 11.808000, .xc = 3.118100, .yc = -10.808000, .distance = -0.382070};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest313)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 2.475500, .yc = 1001.000000, .distance = -0.951720};
  maneuver expected2 = {.radius = -0.068283, .xc = 2.951700, .yc = 1.048300, .distance = -0.053629};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest314)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 2.489700, .yc = 1001.000000, .distance = -0.980000};
  maneuver expected2 = {.radius = -0.020000, .xc = 2.980000, .yc = 1.000000, .distance = -0.031415};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest315)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 2.495500, .yc = 1001.000000, .distance = -0.991720};
  maneuver expected2 = {.radius = -0.011716, .xc = 2.991700, .yc = 0.991720, .distance = -0.027604};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest316)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.244990, .xc = 2.000000, .yc = 1.245000, .distance = 0.767220};
  maneuver expected2 = {.radius = 1000.000000, .xc = -997.490000, .yc = 99997.000000, .distance = 1.002500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest317)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.404690, .xc = 2.000000, .yc = 1.404700, .distance = -0.834300};
  maneuver expected2 = {.radius = 0.404690, .xc = 2.713800, .yc = 1.786200, .distance = -1.787800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest318)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.339480, .xc = 2.000000, .yc = 1.339500, .distance = -0.612540};
  maneuver expected2 = {.radius = 0.339480, .xc = 2.660500, .yc = 1.496600, .distance = -1.142400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest319)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.374190, .xc = 2.000000, .yc = 1.374200, .distance = -0.517980};
  maneuver expected2 = {.radius = 0.374190, .xc = 2.735400, .yc = 1.235400, .distance = -0.811870};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest320)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.631330, .xc = 1.993700, .yc = 1.631300, .distance = -0.575960};
  maneuver expected2 = {.radius = 0.631330, .xc = 3.000000, .yc = 0.868670, .distance = -0.582270};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest321)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.631310, .xc = 1.993700, .yc = 1.631300, .distance = -0.575960};
  maneuver expected2 = {.radius = 0.631310, .xc = 3.000000, .yc = 0.868690, .distance = -0.582270};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest322)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -1.207100, .xc = 2.000000, .yc = 2.207100, .distance = -0.948060};
  maneuver expected2 = {.radius = -1000.000000, .xc = 710.030000, .yc = -705.680000, .distance = -0.207110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest323)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 2.252100, .yc = 1001.000000, .distance = -0.504980};
  maneuver expected2 = {.radius = -0.495050, .xc = 2.505000, .yc = 1.495000, .distance = -0.782570};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest324)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 2.396100, .yc = 1001.000000, .distance = -0.792890};
  maneuver expected2 = {.radius = -0.292890, .xc = 2.792900, .yc = 1.292900, .distance = -0.690110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest325)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.249990, .xc = 2.000000, .yc = 1.250000, .distance = 0.782880};
  maneuver expected2 = {.radius = 1000.000000, .xc = -997.990000, .yc = 99997.000000, .distance = 0.002500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest326)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.207110, .xc = 2.000000, .yc = 1.207100, .distance = 0.487980};
  maneuver expected2 = {.radius = 1000.000000, .xc = -705.180000, .yc = 708.530000, .distance = 0.207110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest327)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.178440, .xc = 1.980000, .yc = 1.178400, .distance = 0.478500};
  maneuver expected2 = {.radius = 0.178440, .xc = 1.821600, .yc = 1.498200, .distance = 0.199990};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest328)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.139750, .xc = 2.000000, .yc = 1.139700, .distance = 0.388520};
  maneuver expected2 = {.radius = 0.139750, .xc = 1.901200, .yc = 1.401200, .distance = 0.278770};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest329)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.125000, .xc = 1.998700, .yc = 1.125000, .distance = -0.390830};
  maneuver expected2 = {.radius = 0.125000, .xc = 2.000000, .yc = 1.375000, .distance = -0.392080};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest330)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.125000, .xc = 1.998700, .yc = 1.125000, .distance = -0.390830};
  maneuver expected2 = {.radius = 0.125000, .xc = 2.000000, .yc = 1.375000, .distance = -0.392080};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest331)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.139750, .xc = 2.000000, .yc = 1.139700, .distance = -0.388520};
  maneuver expected2 = {.radius = 0.139750, .xc = 2.098800, .yc = 1.401200, .distance = -0.278770};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest332)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.024752, .xc = 1.980000, .yc = 1.024800, .distance = -0.039127};
  maneuver expected2 = {.radius = -1000.000000, .xc = 12.002000, .yc = 1.362500, .distance = -0.475020};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest333)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.207110, .xc = 2.000000, .yc = 1.207100, .distance = -0.487980};
  maneuver expected2 = {.radius = -1000.000000, .xc = 709.180000, .yc = 708.530000, .distance = -0.207110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest334)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1.751600, .yc = -999.000000, .distance = 0.497500};
  maneuver expected2 = {.radius = -0.250010, .xc = 1.502500, .yc = 1.250000, .distance = 0.782920};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest335)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1.853900, .yc = -999.000000, .distance = 0.292890};
  maneuver expected2 = {.radius = -0.292890, .xc = 1.707100, .yc = 1.292900, .distance = 0.690110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest336)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.499970, .xc = 2.000000, .yc = 1.500000, .distance = 0.780360};
  maneuver expected2 = {.radius = 1000.000000, .xc = -8.499100, .yc = 1.397500, .distance = 0.005025};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest337)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.399580, .xc = 2.000000, .yc = 1.379600, .distance = 0.546030};
  maneuver expected2 = {.radius = 0.399580, .xc = 1.217500, .yc = 1.217500, .distance = 0.232200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest338)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.248760, .xc = 1.997500, .yc = 1.248700, .distance = 0.394490};
  maneuver expected2 = {.radius = 0.248760, .xc = 1.500000, .yc = 1.251200, .distance = 0.392000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest339)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.248760, .xc = 1.997500, .yc = 1.248700, .distance = 0.394480};
  maneuver expected2 = {.radius = 0.248760, .xc = 1.500000, .yc = 1.251200, .distance = 0.392000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest340)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.197630, .xc = 2.000000, .yc = 1.197600, .distance = 0.394240};
  maneuver expected2 = {.radius = 0.197630, .xc = 1.639700, .yc = 1.360300, .distance = 0.549460};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest341)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.207540, .xc = 2.000000, .yc = 1.207500, .distance = 0.489730};
  maneuver expected2 = {.radius = 0.207540, .xc = 1.707500, .yc = 1.502100, .distance = 0.817810};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest342)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.008284, .xc = 2.000000, .yc = 0.988280, .distance = -0.019520};
  maneuver expected2 = {.radius = -1000.000000, .xc = 708.860000, .yc = 708.350000, .distance = -0.715390};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest343)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.010000, .xc = 2.000000, .yc = 0.990000, .distance = -0.031416};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1.500000, .yc = 2886800000.000000, .distance = -1.000000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest344)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1.504500, .yc = -999.020000, .distance = 0.991720};
  maneuver expected2 = {.radius = -0.011716, .xc = 1.008300, .yc = 0.991720, .distance = 0.027605};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest345)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1.510300, .yc = -999.020000, .distance = 0.980000};
  maneuver expected2 = {.radius = -0.020000, .xc = 1.020000, .yc = 1.000000, .distance = 0.031416};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest346)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1.524500, .yc = -999.020000, .distance = 0.951710};
  maneuver expected2 = {.radius = -0.068285, .xc = 1.048300, .yc = 1.048300, .distance = 0.053631};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest347)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -11.808000, .xc = 2.118100, .yc = 12.788000, .distance = 0.382070};
  maneuver expected2 = {.radius = 11.808000, .xc = 1.118100, .yc = -10.808000, .distance = 0.618230};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest348)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -11.808000, .xc = 2.118100, .yc = 12.788000, .distance = 0.382070};
  maneuver expected2 = {.radius = 11.808000, .xc = 1.118100, .yc = -10.808000, .distance = 0.618230};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest349)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.553680, .xc = 2.000000, .yc = 1.533700, .distance = 0.322110};
  maneuver expected2 = {.radius = 0.553680, .xc = 1.391500, .yc = 0.608490, .distance = 0.756970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest350)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.361950, .xc = 2.000000, .yc = 1.342000, .distance = 0.390480};
  maneuver expected2 = {.radius = 0.361950, .xc = 1.362000, .yc = 1.000000, .distance = 0.959030};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest351)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.371360, .xc = 2.000000, .yc = 1.351400, .distance = 0.538850};
  maneuver expected2 = {.radius = 0.371360, .xc = 1.262600, .yc = 1.262600, .distance = 1.413900};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest352)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.244990, .xc = 2.000000, .yc = 0.755010, .distance = -0.767220};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1001.500000, .yc = -99995.000000, .distance = -1.002500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest353)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.404690, .xc = 2.000000, .yc = 0.595310, .distance = 0.834300};
  maneuver expected2 = {.radius = -0.404690, .xc = 1.286200, .yc = 0.213840, .distance = 1.787800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest354)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.339480, .xc = 2.000000, .yc = 0.660520, .distance = 0.612540};
  maneuver expected2 = {.radius = -0.339480, .xc = 1.339500, .yc = 0.503390, .distance = 1.142400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest355)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.374190, .xc = 2.000000, .yc = 0.625810, .distance = 0.517980};
  maneuver expected2 = {.radius = -0.374190, .xc = 1.264600, .yc = 0.764590, .distance = 0.811870};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest356)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.631330, .xc = 2.006300, .yc = 0.368700, .distance = 0.575960};
  maneuver expected2 = {.radius = -0.631330, .xc = 1.000000, .yc = 1.131300, .distance = 0.582270};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest357)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.631310, .xc = 2.006300, .yc = 0.368720, .distance = 0.575960};
  maneuver expected2 = {.radius = -0.631310, .xc = 1.000000, .yc = 1.131300, .distance = 0.582270};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest358)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 1.207100, .xc = 2.000000, .yc = -0.207110, .distance = 0.948060};
  maneuver expected2 = {.radius = 1000.000000, .xc = -706.030000, .yc = 707.680000, .distance = 0.207110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest359)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1.747900, .yc = -999.000000, .distance = 0.504980};
  maneuver expected2 = {.radius = 0.495050, .xc = 1.495000, .yc = 0.504950, .distance = 0.782570};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest360)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1.603900, .yc = -999.000000, .distance = 0.792890};
  maneuver expected2 = {.radius = 0.292890, .xc = 1.207100, .yc = 0.707110, .distance = 0.690110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest361)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.499990, .xc = 2.000000, .yc = 0.500010, .distance = -1.565800};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1001.900000, .yc = -99988.000000, .distance = -0.005000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest362)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.414210, .xc = 2.000000, .yc = 0.585790, .distance = -0.975970};
  maneuver expected2 = {.radius = -1000.000000, .xc = 709.250000, .yc = -706.960000, .distance = -0.414210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest363)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.010100, .xc = 1.980000, .yc = 0.989900, .distance = -0.015765};
  maneuver expected2 = {.radius = -1000.000000, .xc = 11.995000, .yc = 0.595000, .distance = -0.990050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest364)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.279490, .xc = 2.000000, .yc = 0.720510, .distance = -0.777050};
  maneuver expected2 = {.radius = -0.279490, .xc = 2.197600, .yc = 0.197630, .distance = -0.557540};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest365)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.250010, .xc = 2.002500, .yc = 0.750010, .distance = 0.781670};
  maneuver expected2 = {.radius = -0.250010, .xc = 2.000000, .yc = 0.250010, .distance = 0.784170};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest366)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.250010, .xc = 2.002500, .yc = 0.750010, .distance = 0.781670};
  maneuver expected2 = {.radius = -0.250010, .xc = 2.000000, .yc = 0.250010, .distance = 0.784170};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest367)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.279490, .xc = 2.000000, .yc = 0.720510, .distance = 0.777050};
  maneuver expected2 = {.radius = -0.279490, .xc = 1.802400, .yc = 0.197630, .distance = 0.557540};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest368)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.363280, .xc = 1.980000, .yc = 0.636720, .distance = 0.962510};
  maneuver expected2 = {.radius = -0.363280, .xc = 1.636700, .yc = -0.003633, .distance = 0.388250};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest369)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.414210, .xc = 2.000000, .yc = 0.585790, .distance = 0.975970};
  maneuver expected2 = {.radius = 1000.000000, .xc = -705.250000, .yc = -706.960000, .distance = 0.414210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest370)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 2.247800, .yc = 1001.000000, .distance = -0.495000};
  maneuver expected2 = {.radius = 0.500010, .xc = 2.495000, .yc = 0.499990, .distance = -1.565800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest371)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 2.043200, .yc = 1001.000000, .distance = -0.085787};
  maneuver expected2 = {.radius = 0.585790, .xc = 2.085800, .yc = 0.414210, .distance = -1.380200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest372)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.494920, .xc = 2.000000, .yc = 0.505080, .distance = -0.772480};
  maneuver expected2 = {.radius = -1000.000000, .xc = 12.497000, .yc = 0.355010, .distance = -0.510050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest373)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.423030, .xc = 2.000000, .yc = 0.576970, .distance = -0.806040};
  maneuver expected2 = {.radius = -0.423030, .xc = 2.799100, .yc = 0.299120, .distance = -0.473800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest374)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.311730, .xc = 2.003100, .yc = 0.688290, .distance = -0.694940};
  maneuver expected2 = {.radius = -0.311730, .xc = 2.500000, .yc = 0.311730, .distance = -0.691820};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest375)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.311730, .xc = 2.003100, .yc = 0.688290, .distance = -0.694940};
  maneuver expected2 = {.radius = -0.311730, .xc = 2.500000, .yc = 0.311730, .distance = -0.691820};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest376)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.291240, .xc = 2.000000, .yc = 0.708760, .distance = -0.760850};
  maneuver expected2 = {.radius = -0.291240, .xc = 2.294100, .yc = 0.205940, .distance = -0.989590};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest377)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.340760, .xc = 2.000000, .yc = 0.659240, .distance = -0.990150};
  maneuver expected2 = {.radius = -0.340760, .xc = 2.159300, .yc = -0.003408, .distance = -1.528800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest378)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.207110, .xc = 2.000000, .yc = 0.792890, .distance = 0.487980};
  maneuver expected2 = {.radius = 1000.000000, .xc = -704.930000, .yc = -706.780000, .distance = 0.914210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest379)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.010000, .xc = 2.000000, .yc = 0.990000, .distance = 0.031416};
  maneuver expected2 = {.radius = 1000.000000, .xc = 2.500000, .yc = 2886800000.000000, .distance = 1.000000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest380)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.371360, .xc = 2.000000, .yc = 1.351400, .distance = -0.538850};
  maneuver expected2 = {.radius = 0.371360, .xc = 2.737400, .yc = 1.262600, .distance = -1.413900};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest381)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.361950, .xc = 2.000000, .yc = 1.342000, .distance = -0.390480};
  maneuver expected2 = {.radius = 0.361950, .xc = 2.638000, .yc = 1.000000, .distance = -0.959030};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest382)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.553680, .xc = 2.000000, .yc = 1.533700, .distance = -0.322110};
  maneuver expected2 = {.radius = 0.553680, .xc = 2.608500, .yc = 0.608490, .distance = -0.756970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest383)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -11.808000, .xc = 2.118100, .yc = 12.788000, .distance = -0.618240};
  maneuver expected2 = {.radius = 11.808000, .xc = 3.118100, .yc = -10.808000, .distance = -0.382060};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest384)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -11.808000, .xc = 2.118100, .yc = 12.788000, .distance = -0.618230};
  maneuver expected2 = {.radius = 11.808000, .xc = 3.118100, .yc = -10.808000, .distance = -0.382070};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest385)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 2.476200, .yc = 1001.000000, .distance = -0.951710};
  maneuver expected2 = {.radius = -0.068285, .xc = 2.951700, .yc = 1.048300, .distance = -0.053631};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest386)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 2.490300, .yc = 1001.000000, .distance = -0.980000};
  maneuver expected2 = {.radius = -0.020000, .xc = 2.980000, .yc = 1.000000, .distance = -0.031416};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest387)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 2.496200, .yc = 1001.000000, .distance = -0.991720};
  maneuver expected2 = {.radius = -0.011716, .xc = 2.991700, .yc = 0.991720, .distance = -0.027605};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest388)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.244990, .xc = 2.000000, .yc = 1.245000, .distance = 0.767220};
  maneuver expected2 = {.radius = 1000.000000, .xc = -997.420000, .yc = 99990.000000, .distance = 1.002500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest389)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.404690, .xc = 2.000000, .yc = 1.404700, .distance = -0.834300};
  maneuver expected2 = {.radius = 0.404690, .xc = 2.713800, .yc = 1.786200, .distance = -1.787800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest390)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.339480, .xc = 2.000000, .yc = 1.339500, .distance = -0.612540};
  maneuver expected2 = {.radius = 0.339480, .xc = 2.660500, .yc = 1.496600, .distance = -1.142400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest391)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.374190, .xc = 2.000000, .yc = 1.374200, .distance = -0.517980};
  maneuver expected2 = {.radius = 0.374190, .xc = 2.735400, .yc = 1.235400, .distance = -0.811870};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest392)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.631330, .xc = 1.993700, .yc = 1.631300, .distance = -0.575960};
  maneuver expected2 = {.radius = 0.631330, .xc = 3.000000, .yc = 0.868670, .distance = -0.582270};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest393)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.631330, .xc = 1.993700, .yc = 1.631300, .distance = -0.575960};
  maneuver expected2 = {.radius = 0.631330, .xc = 3.000000, .yc = 0.868670, .distance = -0.582270};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest394)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -1.207100, .xc = 2.000000, .yc = 2.207100, .distance = -0.948060};
  maneuver expected2 = {.radius = -1000.000000, .xc = 710.030000, .yc = -705.680000, .distance = -0.207110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest395)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 2.252800, .yc = 1001.000000, .distance = -0.504970};
  maneuver expected2 = {.radius = -0.495050, .xc = 2.505000, .yc = 1.495000, .distance = -0.782570};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest396)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = 2.396800, .yc = 1001.000000, .distance = -0.792890};
  maneuver expected2 = {.radius = -0.292890, .xc = 2.792900, .yc = 1.292900, .distance = -0.690110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest397)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.249990, .xc = 2.000000, .yc = 1.250000, .distance = 0.782880};
  maneuver expected2 = {.radius = 1000.000000, .xc = -997.920000, .yc = 99990.000000, .distance = 0.002500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest398)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.207110, .xc = 2.000000, .yc = 1.207100, .distance = 0.487980};
  maneuver expected2 = {.radius = 1000.000000, .xc = -705.180000, .yc = 708.530000, .distance = 0.207110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest399)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.178440, .xc = 1.980000, .yc = 1.178400, .distance = 0.478500};
  maneuver expected2 = {.radius = 0.178440, .xc = 1.821600, .yc = 1.498200, .distance = 0.199990};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest400)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.139750, .xc = 2.000000, .yc = 1.139700, .distance = 0.388520};
  maneuver expected2 = {.radius = 0.139750, .xc = 1.901200, .yc = 1.401200, .distance = 0.278770};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest401)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.125000, .xc = 1.998800, .yc = 1.125000, .distance = -0.390830};
  maneuver expected2 = {.radius = 0.125000, .xc = 2.000000, .yc = 1.375000, .distance = -0.392080};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest402)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.125000, .xc = 1.998800, .yc = 1.125000, .distance = -0.390830};
  maneuver expected2 = {.radius = 0.125000, .xc = 2.000000, .yc = 1.375000, .distance = -0.392080};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest403)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.139750, .xc = 2.000000, .yc = 1.139700, .distance = -0.388520};
  maneuver expected2 = {.radius = 0.139750, .xc = 2.098800, .yc = 1.401200, .distance = -0.278770};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest404)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.024752, .xc = 1.980000, .yc = 1.024800, .distance = -0.039127};
  maneuver expected2 = {.radius = -1000.000000, .xc = 12.003000, .yc = 1.362500, .distance = -0.475020};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest405)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.207110, .xc = 2.000000, .yc = 1.207100, .distance = -0.487980};
  maneuver expected2 = {.radius = -1000.000000, .xc = 709.180000, .yc = 708.530000, .distance = -0.207110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest406)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1.750900, .yc = -999.000000, .distance = 0.497500};
  maneuver expected2 = {.radius = -0.250010, .xc = 1.502500, .yc = 1.250000, .distance = 0.782920};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest407)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1.853200, .yc = -999.000000, .distance = 0.292890};
  maneuver expected2 = {.radius = -0.292890, .xc = 1.707100, .yc = 1.292900, .distance = 0.690110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest408)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.499970, .xc = 2.000000, .yc = 1.500000, .distance = 0.780360};
  maneuver expected2 = {.radius = 1000.000000, .xc = -8.499800, .yc = 1.397500, .distance = 0.005025};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest409)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.399580, .xc = 2.000000, .yc = 1.379600, .distance = 0.546030};
  maneuver expected2 = {.radius = 0.399580, .xc = 1.217500, .yc = 1.217500, .distance = 0.232200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest410)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.248760, .xc = 1.997500, .yc = 1.248700, .distance = 0.394490};
  maneuver expected2 = {.radius = 0.248760, .xc = 1.500000, .yc = 1.251200, .distance = 0.392000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest411)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.248760, .xc = 1.997500, .yc = 1.248800, .distance = 0.394490};
  maneuver expected2 = {.radius = 0.248760, .xc = 1.500000, .yc = 1.251200, .distance = 0.392000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest412)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.197630, .xc = 2.000000, .yc = 1.197600, .distance = 0.394240};
  maneuver expected2 = {.radius = 0.197630, .xc = 1.639700, .yc = 1.360300, .distance = 0.549460};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest413)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.207540, .xc = 2.000000, .yc = 1.207500, .distance = 0.489730};
  maneuver expected2 = {.radius = 0.207540, .xc = 1.707500, .yc = 1.502100, .distance = 0.817810};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest414)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.008284, .xc = 2.000000, .yc = 0.988280, .distance = -0.019520};
  maneuver expected2 = {.radius = -1000.000000, .xc = 708.860000, .yc = 708.350000, .distance = -0.715390};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest415)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1.499700, .yc = -999.020000, .distance = 1.000000};
  maneuver expected2 = {.radius = -0.010002, .xc = 1.000000, .yc = 0.990000, .distance = 0.031422};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest416)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1.503800, .yc = -999.020000, .distance = 0.991720};
  maneuver expected2 = {.radius = -0.011716, .xc = 1.008300, .yc = 0.991720, .distance = 0.027604};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest417)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1.509700, .yc = -999.020000, .distance = 0.980000};
  maneuver expected2 = {.radius = -0.020000, .xc = 1.020000, .yc = 1.000000, .distance = 0.031415};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest418)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1.523800, .yc = -999.020000, .distance = 0.951720};
  maneuver expected2 = {.radius = -0.068283, .xc = 1.048300, .yc = 1.048300, .distance = 0.053629};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest419)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -11.808000, .xc = 2.118100, .yc = 12.788000, .distance = 0.382060};
  maneuver expected2 = {.radius = 11.808000, .xc = 1.118100, .yc = -10.808000, .distance = 0.618240};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest420)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -11.809000, .xc = 2.118100, .yc = 12.788000, .distance = 0.382060};
  maneuver expected2 = {.radius = 11.809000, .xc = 1.118100, .yc = -10.808000, .distance = 0.618240};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest421)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.553680, .xc = 2.000000, .yc = 1.533700, .distance = 0.322110};
  maneuver expected2 = {.radius = 0.553680, .xc = 1.391500, .yc = 0.608490, .distance = 0.756970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest422)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.361950, .xc = 2.000000, .yc = 1.342000, .distance = 0.390480};
  maneuver expected2 = {.radius = 0.361950, .xc = 1.362000, .yc = 1.000000, .distance = 0.959030};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest423)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.371360, .xc = 2.000000, .yc = 1.351400, .distance = 0.538850};
  maneuver expected2 = {.radius = 0.371360, .xc = 1.262600, .yc = 1.262600, .distance = 1.413900};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest424)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.244990, .xc = 2.000000, .yc = 0.755010, .distance = -0.767220};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1001.400000, .yc = -99988.000000, .distance = -1.002500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest425)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.404690, .xc = 2.000000, .yc = 0.595310, .distance = 0.834300};
  maneuver expected2 = {.radius = -0.404690, .xc = 1.286200, .yc = 0.213840, .distance = 1.787800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest426)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.339480, .xc = 2.000000, .yc = 0.660520, .distance = 0.612540};
  maneuver expected2 = {.radius = -0.339480, .xc = 1.339500, .yc = 0.503390, .distance = 1.142400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest427)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.374190, .xc = 2.000000, .yc = 0.625810, .distance = 0.517980};
  maneuver expected2 = {.radius = -0.374190, .xc = 1.264600, .yc = 0.764590, .distance = 0.811870};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest428)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.631330, .xc = 2.006300, .yc = 0.368700, .distance = 0.575960};
  maneuver expected2 = {.radius = -0.631330, .xc = 1.000000, .yc = 1.131300, .distance = 0.582270};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest429)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.631330, .xc = 2.006300, .yc = 0.368700, .distance = 0.575960};
  maneuver expected2 = {.radius = -0.631330, .xc = 1.000000, .yc = 1.131300, .distance = 0.582270};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest430)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 1.207100, .xc = 2.000000, .yc = -0.207110, .distance = 0.948060};
  maneuver expected2 = {.radius = 1000.000000, .xc = -706.030000, .yc = 707.680000, .distance = 0.207110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest431)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1.747200, .yc = -999.000000, .distance = 0.504970};
  maneuver expected2 = {.radius = 0.495050, .xc = 1.495000, .yc = 0.504950, .distance = 0.782570};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest432)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -3.141600 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1.603200, .yc = -999.000000, .distance = 0.792890};
  maneuver expected2 = {.radius = 0.292890, .xc = 1.207100, .yc = 0.707110, .distance = 0.690110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest433)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 708.960000, .yc = -706.250000, .distance = 0.414210};
  maneuver expected2 = {.radius = 0.414210, .xc = 2.000000, .yc = 0.414210, .distance = 0.975970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest434)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.350010, .xc = 2.247500, .yc = 0.752510, .distance = -1.096100};
  maneuver expected2 = {.radius = -1000.000000, .xc = 71419.000000, .yc = -70002.000000, .distance = -0.710640};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest435)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.366370, .xc = 2.273200, .yc = 0.726790, .distance = 0.909950};
  maneuver expected2 = {.radius = -0.366370, .xc = 2.366400, .yc = -0.000000, .distance = 1.773200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest436)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.292290, .xc = 2.206700, .yc = 0.793320, .distance = 0.687660};
  maneuver expected2 = {.radius = -0.292290, .xc = 2.204600, .yc = 0.208740, .distance = 1.143900};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest437)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.279490, .xc = 2.197600, .yc = 0.802370, .distance = 0.557540};
  maneuver expected2 = {.radius = -0.279490, .xc = 2.000000, .yc = 0.279490, .distance = 0.777050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest438)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.279490, .xc = 2.197600, .yc = 0.802370, .distance = 0.557540};
  maneuver expected2 = {.radius = -0.279490, .xc = 2.000000, .yc = 0.279490, .distance = 0.777050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest439)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.355340, .xc = 2.253800, .yc = 0.751260, .distance = 0.552830};
  maneuver expected2 = {.radius = -0.355340, .xc = 1.748700, .yc = 0.251260, .distance = 0.556390};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest440)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.048285, .xc = 2.048300, .yc = 0.951710, .distance = 0.037923};
  maneuver expected2 = {.radius = 1000.000000, .xc = -998.000000, .yc = 0.475530, .distance = 0.951710};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest441)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 709.100000, .yc = -706.110000, .distance = 0.007036};
  maneuver expected2 = {.radius = 0.700110, .xc = 2.490100, .yc = 0.499980, .distance = 1.106700};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest442)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.585790, .xc = 2.414200, .yc = 0.585790, .distance = 1.380200};
  maneuver expected2 = {.radius = 1000.000000, .xc = 2.457100, .yc = -1000.000000, .distance = 0.085786};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest443)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.528550, .xc = 2.373700, .yc = 0.626260, .distance = -1.655200};
  maneuver expected2 = {.radius = -1000.000000, .xc = 71419.000000, .yc = -70002.000000, .distance = -0.358860};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest444)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.292890, .xc = 2.207100, .yc = 0.792890, .distance = -0.690110};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1002.500000, .yc = 0.396120, .distance = -0.792890};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest445)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.352380, .xc = 2.249200, .yc = 0.750830, .distance = 1.106400};
  maneuver expected2 = {.radius = -0.352380, .xc = 2.746700, .yc = 0.251650, .distance = 1.656400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest446)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.291240, .xc = 2.205900, .yc = 0.794060, .distance = 0.840350};
  maneuver expected2 = {.radius = -0.291240, .xc = 2.500000, .yc = 0.291240, .distance = 1.069100};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest447)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.291240, .xc = 2.205900, .yc = 0.794060, .distance = 0.840350};
  maneuver expected2 = {.radius = -0.291240, .xc = 2.500000, .yc = 0.291240, .distance = 1.069100};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest448)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.295130, .xc = 2.210800, .yc = 0.793410, .distance = 0.732830};
  maneuver expected2 = {.radius = -0.295130, .xc = 2.291300, .yc = 0.208690, .distance = 0.735780};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest449)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.374190, .xc = 2.264600, .yc = 0.735410, .distance = 0.811870};
  maneuver expected2 = {.radius = -0.374190, .xc = 2.125800, .yc = -0.000000, .distance = 0.517980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest450)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.360540, .xc = 2.254900, .yc = 0.745060, .distance = 0.569940};
  maneuver expected2 = {.radius = 1000.000000, .xc = -4.748700, .yc = -6.897600, .distance = 0.696550};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest451)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.366370, .xc = 2.273200, .yc = 0.726790, .distance = -0.909950};
  maneuver expected2 = {.radius = -0.366370, .xc = 3.000000, .yc = 0.633630, .distance = -1.773200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest452)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -704.860000, .yc = 708.360000, .distance = -0.703570};
  maneuver expected2 = {.radius = 0.353560, .xc = 2.747500, .yc = 1.247500, .distance = -1.107200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest453)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -704.960000, .yc = 708.250000, .distance = -0.414210};
  maneuver expected2 = {.radius = 0.414210, .xc = 2.585800, .yc = 1.000000, .distance = -0.975970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest454)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.707070, .xc = 2.500000, .yc = 0.500030, .distance = -1.103600};
  maneuver expected2 = {.radius = -1000.000000, .xc = 9.997000, .yc = 8.143400, .distance = -0.007107};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest455)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.048283, .xc = 2.048300, .yc = 0.951720, .distance = -0.037921};
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.523800, .yc = 1001.000000, .distance = -0.951720};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest456)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.048285, .xc = 2.048300, .yc = 0.951720, .distance = -0.037923};
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.524500, .yc = 1001.000000, .distance = -0.951710};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest457)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.351800, .xc = 2.251200, .yc = 0.753740, .distance = -0.557890};
  maneuver expected2 = {.radius = -0.351800, .xc = 2.751200, .yc = 1.248800, .distance = -0.554370};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest458)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.279490, .xc = 2.197600, .yc = 0.802370, .distance = -0.557540};
  maneuver expected2 = {.radius = -0.279490, .xc = 2.720500, .yc = 1.000000, .distance = -0.777050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest459)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.293510, .xc = 2.207500, .yc = 0.792460, .distance = -0.692590};
  maneuver expected2 = {.radius = -0.293510, .xc = 2.794500, .yc = 0.790400, .distance = -1.156600};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest460)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.404690, .xc = 2.286200, .yc = 0.713840, .distance = -0.754910};
  maneuver expected2 = {.radius = -0.404690, .xc = 3.000000, .yc = 1.095300, .distance = -1.708400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest461)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -704.730000, .yc = 708.480000, .distance = -1.058900};
  maneuver expected2 = {.radius = 0.176780, .xc = 2.873800, .yc = 1.623700, .distance = -0.553610};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest462)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -704.780000, .yc = 708.430000, .distance = -0.914210};
  maneuver expected2 = {.radius = 0.207110, .xc = 2.792900, .yc = 1.500000, .distance = -0.487980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest463)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -704.860000, .yc = 708.360000, .distance = -0.703550};
  maneuver expected2 = {.radius = 0.357120, .xc = 2.750000, .yc = 1.245000, .distance = -0.557400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest464)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -705.030000, .yc = 708.180000, .distance = -0.207110};
  maneuver expected2 = {.radius = 1.207100, .xc = 3.000000, .yc = 0.292890, .distance = -0.948060};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest465)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -705.030000, .yc = 708.180000, .distance = -0.207110};
  maneuver expected2 = {.radius = 1.207100, .xc = 3.000000, .yc = 0.292890, .distance = -0.948060};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest466)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.870840, .xc = 2.621900, .yc = 0.390410, .distance = -0.573450};
  maneuver expected2 = {.radius = -0.870840, .xc = 2.384200, .yc = 2.115800, .distance = -0.564740};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest467)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.423020, .xc = 2.299100, .yc = 0.700880, .distance = -0.473800};
  maneuver expected2 = {.radius = -0.423020, .xc = 2.577000, .yc = 1.500000, .distance = -0.806040};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest468)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.353560, .xc = 2.250000, .yc = 0.749990, .distance = -0.557140};
  maneuver expected2 = {.radius = -0.353560, .xc = 2.752500, .yc = 1.247500, .distance = -1.116000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest469)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -705.030000, .yc = 708.180000, .distance = -0.207110};
  maneuver expected2 = {.radius = -0.207110, .xc = 2.000000, .yc = 1.292900, .distance = -0.487980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest470)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.175000, .xc = 1.876300, .yc = 1.123700, .distance = 0.548040};
  maneuver expected2 = {.radius = 1000.000000, .xc = -71415.000000, .yc = 70004.000000, .distance = 0.355320};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest471)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.008284, .xc = 2.008300, .yc = 0.991720, .distance = 0.019520};
  maneuver expected2 = {.radius = 1000.000000, .xc = -998.000000, .yc = 1.246200, .distance = 0.508280};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest472)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.146150, .xc = 1.896700, .yc = 1.103300, .distance = -0.343830};
  maneuver expected2 = {.radius = 0.146150, .xc = 1.897700, .yc = 1.395600, .distance = -0.571930};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest473)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.139750, .xc = 1.901200, .yc = 1.098800, .distance = -0.278770};
  maneuver expected2 = {.radius = 0.139750, .xc = 2.000000, .yc = 1.360300, .distance = -0.388520};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest474)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.139750, .xc = 1.901200, .yc = 1.098800, .distance = -0.278770};
  maneuver expected2 = {.radius = 0.139750, .xc = 2.000000, .yc = 1.360300, .distance = -0.388520};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest475)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.177670, .xc = 1.873100, .yc = 1.124400, .distance = -0.276420};
  maneuver expected2 = {.radius = 0.177670, .xc = 2.125600, .yc = 1.374400, .distance = -0.278190};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest476)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.281360, .xc = 1.815200, .yc = 1.184800, .distance = -0.388260};
  maneuver expected2 = {.radius = 0.281360, .xc = 2.281400, .yc = 1.500000, .distance = -0.167270};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest477)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -705.110000, .yc = 708.110000, .distance = -0.003518};
  maneuver expected2 = {.radius = -0.350050, .xc = 1.755000, .yc = 1.250000, .distance = -0.553360};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest478)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.292890, .xc = 1.792900, .yc = 1.207100, .distance = -0.690110};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1.646400, .yc = 1001.500000, .distance = -0.292890};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest479)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.353540, .xc = 1.750000, .yc = 1.250000, .distance = 1.107200};
  maneuver expected2 = {.radius = 1000.000000, .xc = -71415.000000, .yc = 70004.000000, .distance = 0.003536};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest480)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.292890, .xc = 1.792900, .yc = 1.207100, .distance = 0.690110};
  maneuver expected2 = {.radius = 1000.000000, .xc = -998.500000, .yc = 1.353900, .distance = 0.292890};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest481)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.253880, .xc = 1.806300, .yc = 1.165400, .distance = 0.676060};
  maneuver expected2 = {.radius = 0.253880, .xc = 1.322300, .yc = 1.318700, .distance = 0.279810};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest482)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.197630, .xc = 1.860300, .yc = 1.139700, .distance = 0.549460};
  maneuver expected2 = {.radius = 0.197630, .xc = 1.500000, .yc = 1.302400, .distance = 0.394240};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest483)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.197630, .xc = 1.860300, .yc = 1.139700, .distance = 0.549460};
  maneuver expected2 = {.radius = 0.197630, .xc = 1.500000, .yc = 1.302400, .distance = 0.394240};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest484)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.176780, .xc = 1.873800, .yc = 1.123700, .distance = -0.552720};
  maneuver expected2 = {.radius = 0.176780, .xc = 1.625000, .yc = 1.375000, .distance = -0.554490};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest485)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.197630, .xc = 1.860300, .yc = 1.139700, .distance = -0.549460};
  maneuver expected2 = {.radius = 0.197630, .xc = 1.697600, .yc = 1.500000, .distance = -0.394240};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest486)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.026802, .xc = 1.966900, .yc = 1.004800, .distance = -0.042369};
  maneuver expected2 = {.radius = -1000.000000, .xc = 8.742800, .yc = 8.403400, .distance = -0.680070};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest487)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.008284, .xc = 2.008300, .yc = 0.991720, .distance = -0.019519};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1.504100, .yc = 1001.000000, .distance = -1.008300};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest488)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 708.860000, .yc = -706.360000, .distance = 0.703570};
  maneuver expected2 = {.radius = -0.353560, .xc = 1.252500, .yc = 0.752510, .distance = 1.107200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest489)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 708.960000, .yc = -706.250000, .distance = 0.414210};
  maneuver expected2 = {.radius = -0.414210, .xc = 1.414200, .yc = 1.000000, .distance = 0.975970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest490)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.707070, .xc = 1.500000, .yc = 1.500000, .distance = 1.103600};
  maneuver expected2 = {.radius = 1000.000000, .xc = -5.997000, .yc = -6.143400, .distance = 0.007107};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest491)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.566900, .xc = 1.613300, .yc = 1.386700, .distance = 0.769240};
  maneuver expected2 = {.radius = 0.566900, .xc = 1.000000, .yc = 0.433100, .distance = 0.323990};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest492)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.566900, .xc = 1.613300, .yc = 1.386700, .distance = 0.769240};
  maneuver expected2 = {.radius = 0.566900, .xc = 1.000000, .yc = 0.433100, .distance = 0.323990};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest493)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.351800, .xc = 1.748800, .yc = 1.246300, .distance = 0.557890};
  maneuver expected2 = {.radius = 0.351800, .xc = 1.248800, .yc = 0.751240, .distance = 0.554370};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest494)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.279490, .xc = 1.802400, .yc = 1.197600, .distance = 0.557540};
  maneuver expected2 = {.radius = 0.279490, .xc = 1.279500, .yc = 1.000000, .distance = 0.777050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest495)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.293510, .xc = 1.792500, .yc = 1.207500, .distance = 0.692590};
  maneuver expected2 = {.radius = 0.293510, .xc = 1.205500, .yc = 1.209600, .distance = 1.156600};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest496)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.404690, .xc = 1.713800, .yc = 1.286200, .distance = 0.754910};
  maneuver expected2 = {.radius = 0.404690, .xc = 1.000000, .yc = 0.904690, .distance = 1.708400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest497)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 708.730000, .yc = -706.480000, .distance = 1.058900};
  maneuver expected2 = {.radius = -0.176780, .xc = 1.126200, .yc = 0.376250, .distance = 0.553610};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest498)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 708.780000, .yc = -706.430000, .distance = 0.914210};
  maneuver expected2 = {.radius = -0.207110, .xc = 1.207100, .yc = 0.500000, .distance = 0.487980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest499)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 708.860000, .yc = -706.360000, .distance = 0.703550};
  maneuver expected2 = {.radius = -0.357120, .xc = 1.250000, .yc = 0.755040, .distance = 0.557400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest500)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 709.030000, .yc = -706.180000, .distance = 0.207110};
  maneuver expected2 = {.radius = -1.207100, .xc = 1.000000, .yc = 1.707100, .distance = 0.948060};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest501)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 709.030000, .yc = -706.180000, .distance = 0.207110};
  maneuver expected2 = {.radius = -1.207100, .xc = 1.000000, .yc = 1.707100, .distance = 0.948060};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest502)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.870840, .xc = 1.378100, .yc = 1.609600, .distance = 0.573450};
  maneuver expected2 = {.radius = 0.870840, .xc = 1.615800, .yc = -0.115780, .distance = 0.564740};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest503)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.423020, .xc = 1.700900, .yc = 1.299100, .distance = 0.473800};
  maneuver expected2 = {.radius = 0.423020, .xc = 1.423000, .yc = 0.500000, .distance = 0.806040};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest504)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -2.356200 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.353560, .xc = 1.750000, .yc = 1.250000, .distance = 0.557140};
  maneuver expected2 = {.radius = 0.353560, .xc = 1.247500, .yc = 0.752490, .distance = 1.116000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest505)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.361950, .xc = 1.658000, .yc = 1.000000, .distance = 0.390480};
  maneuver expected2 = {.radius = 0.361950, .xc = 2.000000, .yc = 0.361950, .distance = 0.959030};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest506)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.371360, .xc = 1.648600, .yc = 1.000000, .distance = 0.538850};
  maneuver expected2 = {.radius = 0.371360, .xc = 1.737400, .yc = 0.262590, .distance = 1.413900};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest507)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.010000, .xc = 2.010000, .yc = 1.000000, .distance = -0.031415};
  maneuver expected2 = {.radius = -1000.000000, .xc = -1530000000.000000, .yc = 500.500000, .distance = -1.000000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest508)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1002.000000, .yc = 0.504470, .distance = 0.991720};
  maneuver expected2 = {.radius = -0.011716, .xc = 2.008300, .yc = 0.008284, .distance = 0.027605};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest509)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1002.000000, .yc = 0.510330, .distance = 0.980000};
  maneuver expected2 = {.radius = -0.020000, .xc = 2.000000, .yc = 0.020000, .distance = 0.031416};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest510)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1002.000000, .yc = 0.510330, .distance = 0.980000};
  maneuver expected2 = {.radius = -0.020000, .xc = 2.000000, .yc = 0.020000, .distance = 0.031416};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest511)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1002.000000, .yc = 0.524470, .distance = 0.951710};
  maneuver expected2 = {.radius = -0.068285, .xc = 1.951700, .yc = 0.048285, .distance = 0.053631};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest512)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -11.808000, .xc = -9.787700, .yc = 1.118100, .distance = 0.382070};
  maneuver expected2 = {.radius = 11.808000, .xc = 13.808000, .yc = 0.118080, .distance = 0.618230};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest513)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.553680, .xc = 1.466300, .yc = 1.000000, .distance = 0.322110};
  maneuver expected2 = {.radius = 0.553680, .xc = 2.391500, .yc = 0.391510, .distance = 0.756970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest514)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1002.000000, .yc = 0.747840, .distance = 0.504980};
  maneuver expected2 = {.radius = 0.495050, .xc = 2.495000, .yc = 0.495020, .distance = 0.782570};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest515)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1002.000000, .yc = 0.603880, .distance = 0.792890};
  maneuver expected2 = {.radius = 0.292890, .xc = 2.292900, .yc = 0.207110, .distance = 0.690110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest516)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.244990, .xc = 2.245000, .yc = 1.000000, .distance = -0.767220};
  maneuver expected2 = {.radius = -1000.000000, .xc = 100000.000000, .yc = 1000.500000, .distance = -1.002500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest517)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.404690, .xc = 2.404700, .yc = 1.000000, .distance = 0.834300};
  maneuver expected2 = {.radius = -0.404690, .xc = 2.786200, .yc = 0.286160, .distance = 1.787800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest518)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.339480, .xc = 2.339500, .yc = 1.000000, .distance = 0.612540};
  maneuver expected2 = {.radius = -0.339480, .xc = 2.496600, .yc = 0.339470, .distance = 1.142400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest519)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.339480, .xc = 2.339500, .yc = 1.000000, .distance = 0.612540};
  maneuver expected2 = {.radius = -0.339480, .xc = 2.496600, .yc = 0.339470, .distance = 1.142400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest520)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.374190, .xc = 2.374200, .yc = 1.000000, .distance = 0.517980};
  maneuver expected2 = {.radius = -0.374190, .xc = 2.235400, .yc = 0.264590, .distance = 0.811870};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest521)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.631330, .xc = 2.631300, .yc = 1.006300, .distance = 0.575960};
  maneuver expected2 = {.radius = -0.631330, .xc = 1.868700, .yc = -0.000000, .distance = 0.582270};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest522)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 1.207100, .xc = 3.207100, .yc = 1.000000, .distance = 0.948060};
  maneuver expected2 = {.radius = 1000.000000, .xc = -704.680000, .yc = -707.030000, .distance = 0.207110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest523)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.363280, .xc = 2.363300, .yc = 0.980000, .distance = 0.962510};
  maneuver expected2 = {.radius = -0.363280, .xc = 3.003600, .yc = 0.636740, .distance = 0.388250};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest524)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.414210, .xc = 2.414200, .yc = 1.000000, .distance = 0.975970};
  maneuver expected2 = {.radius = 1000.000000, .xc = 709.960000, .yc = -706.250000, .distance = 0.414210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest525)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.499990, .xc = 2.500000, .yc = 1.000000, .distance = -1.565800};
  maneuver expected2 = {.radius = -1000.000000, .xc = 100000.000000, .yc = 1001.000000, .distance = -0.005000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest526)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.414210, .xc = 2.414200, .yc = 1.000000, .distance = -0.975970};
  maneuver expected2 = {.radius = -1000.000000, .xc = 709.960000, .yc = 708.250000, .distance = -0.414210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest527)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.010100, .xc = 2.010100, .yc = 0.980000, .distance = -0.015764};
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.405000, .yc = 10.995000, .distance = -0.990050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest528)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.010101, .xc = 2.010100, .yc = 0.980000, .distance = -0.015765};
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.405000, .yc = 10.994000, .distance = -0.990050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest529)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.279490, .xc = 2.279500, .yc = 1.000000, .distance = -0.777050};
  maneuver expected2 = {.radius = -0.279490, .xc = 2.802400, .yc = 1.197600, .distance = -0.557540};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest530)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.250010, .xc = 2.250000, .yc = 1.002500, .distance = 0.781670};
  maneuver expected2 = {.radius = -0.250010, .xc = 2.750000, .yc = 1.000000, .distance = 0.784170};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest531)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.279490, .xc = 2.279500, .yc = 1.000000, .distance = 0.777050};
  maneuver expected2 = {.radius = -0.279490, .xc = 2.802400, .yc = 0.802370, .distance = 0.557540};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest532)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.340760, .xc = 2.340800, .yc = 1.000000, .distance = -0.990150};
  maneuver expected2 = {.radius = -0.340760, .xc = 3.003400, .yc = 1.159300, .distance = -1.528800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest533)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.207110, .xc = 2.207100, .yc = 1.000000, .distance = 0.487980};
  maneuver expected2 = {.radius = 1000.000000, .xc = 709.780000, .yc = -705.930000, .distance = 0.914210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest534)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -998.000000, .yc = 1.247200, .distance = -0.495000};
  maneuver expected2 = {.radius = 0.500010, .xc = 2.500000, .yc = 1.495000, .distance = -1.565800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest535)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -998.000000, .yc = 1.042600, .distance = -0.085787};
  maneuver expected2 = {.radius = 0.585790, .xc = 2.585800, .yc = 1.085800, .distance = -1.380200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest536)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.494920, .xc = 2.494900, .yc = 1.000000, .distance = -0.772480};
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.645000, .yc = 11.497000, .distance = -0.510050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest537)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.494920, .xc = 2.494900, .yc = 1.000000, .distance = -0.772480};
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.645000, .yc = 11.497000, .distance = -0.510050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest538)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.423020, .xc = 2.423000, .yc = 1.000000, .distance = -0.806040};
  maneuver expected2 = {.radius = -0.423020, .xc = 2.700900, .yc = 1.799100, .distance = -0.473800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest539)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.311730, .xc = 2.311700, .yc = 1.003100, .distance = -0.694940};
  maneuver expected2 = {.radius = -0.311730, .xc = 2.688300, .yc = 1.500000, .distance = -0.691820};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest540)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.291240, .xc = 2.291200, .yc = 1.000000, .distance = -0.760850};
  maneuver expected2 = {.radius = -0.291240, .xc = 2.794100, .yc = 1.294100, .distance = -0.989590};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest541)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -997.980000, .yc = 1.239700, .distance = -0.480000};
  maneuver expected2 = {.radius = -0.020000, .xc = 2.000000, .yc = 1.480000, .distance = -0.031416};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest542)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -997.980000, .yc = 1.245500, .distance = -0.491720};
  maneuver expected2 = {.radius = -0.011716, .xc = 2.008300, .yc = 1.491700, .distance = -0.027604};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest543)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -997.980000, .yc = 1.249700, .distance = -0.500000};
  maneuver expected2 = {.radius = -0.009997, .xc = 2.010000, .yc = 1.500000, .distance = -0.031408};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest544)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.185290, .xc = 1.834700, .yc = 1.000000, .distance = -0.273890};
  maneuver expected2 = {.radius = 0.185290, .xc = 1.869000, .yc = 1.369000, .distance = -0.710460};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest545)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.179090, .xc = 1.840900, .yc = 1.000000, .distance = -0.198890};
  maneuver expected2 = {.radius = 0.179090, .xc = 2.000000, .yc = 1.320900, .distance = -0.480200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest546)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.179090, .xc = 1.840900, .yc = 1.000000, .distance = -0.198890};
  maneuver expected2 = {.radius = 0.179090, .xc = 2.000000, .yc = 1.320900, .distance = -0.480200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest547)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.268480, .xc = 1.751500, .yc = 1.000000, .distance = -0.165330};
  maneuver expected2 = {.radius = 0.268480, .xc = 2.189800, .yc = 1.310200, .distance = -0.376190};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest548)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -3.082700, .xc = -1.062500, .yc = 1.030800, .distance = -0.281100};
  maneuver expected2 = {.radius = 3.082700, .xc = 5.082500, .yc = 1.530800, .distance = -0.219450};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest549)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -997.980000, .yc = 1.225500, .distance = -0.451720};
  maneuver expected2 = {.radius = -0.068284, .xc = 1.951700, .yc = 1.451700, .distance = -0.053630};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest550)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -998.000000, .yc = 1.002200, .distance = -0.004975};
  maneuver expected2 = {.radius = -0.495050, .xc = 1.505000, .yc = 1.005000, .distance = -0.782570};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest551)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -998.000000, .yc = 1.146100, .distance = -0.292890};
  maneuver expected2 = {.radius = -0.292890, .xc = 1.707100, .yc = 1.292900, .distance = -0.690110};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest552)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.247490, .xc = 1.752500, .yc = 1.000000, .distance = 0.775050};
  maneuver expected2 = {.radius = 1000.000000, .xc = -99997.000000, .yc = -998.730000, .distance = 0.502500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest553)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.008285, .xc = 2.011700, .yc = 1.000000, .distance = 0.019520};
  maneuver expected2 = {.radius = 1000.000000, .xc = -705.360000, .yc = -705.860000, .distance = 0.715390};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest554)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.206680, .xc = 1.793300, .yc = 1.000000, .distance = -0.486250};
  maneuver expected2 = {.radius = 0.206680, .xc = 1.502100, .yc = 1.293300, .distance = -0.808840};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest555)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.206680, .xc = 1.793300, .yc = 1.000000, .distance = -0.486250};
  maneuver expected2 = {.radius = 0.206680, .xc = 1.502100, .yc = 1.293300, .distance = -0.808840};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest556)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.197630, .xc = 1.802400, .yc = 1.000000, .distance = -0.394240};
  maneuver expected2 = {.radius = 0.197630, .xc = 1.639700, .yc = 1.360300, .distance = -0.549460};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest557)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.251260, .xc = 1.748700, .yc = 0.997490, .distance = -0.390910};
  maneuver expected2 = {.radius = 0.251260, .xc = 1.751300, .yc = 1.500000, .distance = -0.393430};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest558)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.399580, .xc = 1.620400, .yc = 1.000000, .distance = -0.546030};
  maneuver expected2 = {.radius = 0.399580, .xc = 1.782500, .yc = 1.782500, .distance = -0.232200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest559)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.029702, .xc = 1.970300, .yc = 0.980000, .distance = -0.046953};
  maneuver expected2 = {.radius = -1000.000000, .xc = 1.385000, .yc = 11.004000, .distance = -0.970050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest560)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.414210, .xc = 1.585800, .yc = 1.000000, .distance = -0.975970};
  maneuver expected2 = {.radius = -1000.000000, .xc = -705.960000, .yc = 708.250000, .distance = -0.414210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest561)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.499990, .xc = 1.500000, .yc = 1.000000, .distance = 1.565800};
  maneuver expected2 = {.radius = 1000.000000, .xc = -99997.000000, .yc = -998.990000, .distance = 0.005000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest562)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.414210, .xc = 1.585800, .yc = 1.000000, .distance = 0.975970};
  maneuver expected2 = {.radius = 1000.000000, .xc = -705.960000, .yc = -706.250000, .distance = 0.414210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest563)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.360630, .xc = 1.639400, .yc = 0.980000, .distance = 0.955580};
  maneuver expected2 = {.radius = 0.360630, .xc = 1.003600, .yc = 0.639390, .distance = 0.392710};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest564)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.360630, .xc = 1.639400, .yc = 0.980000, .distance = 0.955580};
  maneuver expected2 = {.radius = 0.360630, .xc = 1.003600, .yc = 0.639390, .distance = 0.392710};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest565)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.279490, .xc = 1.720500, .yc = 1.000000, .distance = 0.777050};
  maneuver expected2 = {.radius = 0.279490, .xc = 1.197600, .yc = 0.802370, .distance = 0.557540};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest566)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.250010, .xc = 1.750000, .yc = 0.997500, .distance = -0.781670};
  maneuver expected2 = {.radius = 0.250010, .xc = 1.250000, .yc = 1.000000, .distance = -0.784170};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest567)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.279490, .xc = 1.720500, .yc = 1.000000, .distance = -0.777050};
  maneuver expected2 = {.radius = 0.279490, .xc = 1.197600, .yc = 1.197600, .distance = -0.557540};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest568)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.340760, .xc = 1.659200, .yc = 1.000000, .distance = 0.990150};
  maneuver expected2 = {.radius = 0.340760, .xc = 0.996590, .yc = 0.840740, .distance = 1.528800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest569)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.207110, .xc = 1.792900, .yc = 1.000000, .distance = -0.487980};
  maneuver expected2 = {.radius = -1000.000000, .xc = -705.780000, .yc = 707.930000, .distance = -0.914210};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest570)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1002.000000, .yc = 0.752830, .distance = 0.495000};
  maneuver expected2 = {.radius = -0.500010, .xc = 1.500000, .yc = 0.505000, .distance = 1.565800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest571)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 1002.000000, .yc = 0.957430, .distance = 0.085787};
  maneuver expected2 = {.radius = -0.585790, .xc = 1.414200, .yc = 0.914210, .distance = 1.380200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest572)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.494920, .xc = 1.505100, .yc = 1.000000, .distance = 0.772480};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1.355000, .yc = -9.497300, .distance = 0.510050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest573)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.494920, .xc = 1.505100, .yc = 1.000000, .distance = 0.772480};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1.355000, .yc = -9.496600, .distance = 0.510050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest574)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.423020, .xc = 1.577000, .yc = 1.000000, .distance = 0.806040};
  maneuver expected2 = {.radius = 0.423020, .xc = 1.299100, .yc = 0.200880, .distance = 0.473800};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest575)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.311730, .xc = 1.688300, .yc = 0.996880, .distance = 0.694940};
  maneuver expected2 = {.radius = 0.311730, .xc = 1.311700, .yc = 0.500000, .distance = 0.691820};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest576)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -1.570800 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.291240, .xc = 1.708800, .yc = 1.000000, .distance = 0.760850};
  maneuver expected2 = {.radius = 0.291240, .xc = 1.205900, .yc = 0.705940, .distance = 0.989590};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest577)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.279490, .xc = 1.802400, .yc = 0.802370, .distance = 0.557540};
  maneuver expected2 = {.radius = 0.279490, .xc = 2.000000, .yc = 0.279490, .distance = 0.777050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest578)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.293510, .xc = 1.792500, .yc = 0.792460, .distance = 0.692590};
  maneuver expected2 = {.radius = 0.293510, .xc = 1.790400, .yc = 0.205450, .distance = 1.156600};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest579)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.008284, .xc = 2.008300, .yc = 1.008300, .distance = -0.019519};
  maneuver expected2 = {.radius = -1000.000000, .xc = -998.000000, .yc = 0.504470, .distance = -1.008300};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest580)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 709.360000, .yc = 707.860000, .distance = 0.703570};
  maneuver expected2 = {.radius = -0.353560, .xc = 2.247500, .yc = 0.252490, .distance = 1.107200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest581)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 709.250000, .yc = 707.960000, .distance = 0.414210};
  maneuver expected2 = {.radius = -0.414210, .xc = 2.000000, .yc = 0.414210, .distance = 0.975970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest582)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 709.250000, .yc = 707.960000, .distance = 0.414210};
  maneuver expected2 = {.radius = -0.414210, .xc = 2.000000, .yc = 0.414210, .distance = 0.975970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest583)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.707070, .xc = 1.500000, .yc = 0.500030, .distance = 1.103600};
  maneuver expected2 = {.radius = 1000.000000, .xc = 9.144400, .yc = -6.997900, .distance = 0.007107};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest584)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.566900, .xc = 1.613300, .yc = 0.613280, .distance = 0.769240};
  maneuver expected2 = {.radius = 0.566900, .xc = 2.566900, .yc = 0.000000, .distance = 0.323990};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest585)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.351800, .xc = 1.753700, .yc = 0.748760, .distance = 0.557890};
  maneuver expected2 = {.radius = 0.351800, .xc = 2.248800, .yc = 0.248760, .distance = 0.554370};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest586)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.423020, .xc = 1.700900, .yc = 0.700880, .distance = 0.473800};
  maneuver expected2 = {.radius = 0.423020, .xc = 2.500000, .yc = 0.423020, .distance = 0.806040};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest587)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.353560, .xc = 1.750000, .yc = 0.749990, .distance = 0.557140};
  maneuver expected2 = {.radius = 0.353560, .xc = 2.247500, .yc = 0.247490, .distance = 1.116000};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest588)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.404690, .xc = 1.713800, .yc = 0.713840, .distance = 0.754910};
  maneuver expected2 = {.radius = 0.404690, .xc = 2.095300, .yc = 0.000000, .distance = 1.708400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest589)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 709.480000, .yc = 707.730000, .distance = 1.058900};
  maneuver expected2 = {.radius = -0.176780, .xc = 2.623700, .yc = 0.126250, .distance = 0.553610};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest590)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 709.430000, .yc = 707.780000, .distance = 0.914210};
  maneuver expected2 = {.radius = -0.207110, .xc = 2.500000, .yc = 0.207110, .distance = 0.487980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest591)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 709.430000, .yc = 707.780000, .distance = 0.914210};
  maneuver expected2 = {.radius = -0.207110, .xc = 2.500000, .yc = 0.207110, .distance = 0.487980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest592)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 709.360000, .yc = 707.860000, .distance = 0.703550};
  maneuver expected2 = {.radius = -0.357120, .xc = 2.245000, .yc = 0.249990, .distance = 0.557400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest593)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 709.180000, .yc = 708.030000, .distance = 0.207110};
  maneuver expected2 = {.radius = -1.207100, .xc = 1.292900, .yc = -0.000000, .distance = 0.948060};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest594)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.500000, .y = 0.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.870840, .xc = 1.390400, .yc = 0.378090, .distance = 0.573450};
  maneuver expected2 = {.radius = 0.870840, .xc = 3.115800, .yc = 0.615780, .distance = 0.564740};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest595)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.048284, .xc = 2.048300, .yc = 1.048300, .distance = 0.037922};
  maneuver expected2 = {.radius = 1000.000000, .xc = 2.524100, .yc = -999.000000, .distance = 0.951720};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest596)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 709.110000, .yc = 708.100000, .distance = 0.007036};
  maneuver expected2 = {.radius = 0.700110, .xc = 2.500000, .yc = 1.490100, .distance = 1.106700};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest597)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 709.250000, .yc = 707.960000, .distance = 0.414210};
  maneuver expected2 = {.radius = 0.414210, .xc = 2.585800, .yc = 1.000000, .distance = 0.975970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest598)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.350010, .xc = 2.247500, .yc = 1.247500, .distance = -1.096100};
  maneuver expected2 = {.radius = -1000.000000, .xc = 70005.000000, .yc = 71418.000000, .distance = -0.710640};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest599)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.366370, .xc = 2.273200, .yc = 1.273200, .distance = 0.909950};
  maneuver expected2 = {.radius = -0.366370, .xc = 3.000000, .yc = 1.366400, .distance = 1.773200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest600)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.366370, .xc = 2.273200, .yc = 1.273200, .distance = 0.909950};
  maneuver expected2 = {.radius = -0.366370, .xc = 3.000000, .yc = 1.366400, .distance = 1.773200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest601)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.292290, .xc = 2.206700, .yc = 1.206700, .distance = 0.687660};
  maneuver expected2 = {.radius = -0.292290, .xc = 2.791300, .yc = 1.204600, .distance = 1.143900};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest602)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.279490, .xc = 2.197600, .yc = 1.197600, .distance = 0.557540};
  maneuver expected2 = {.radius = -0.279490, .xc = 2.720500, .yc = 1.000000, .distance = 0.777050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest603)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.355340, .xc = 2.248700, .yc = 1.253800, .distance = 0.552830};
  maneuver expected2 = {.radius = -0.355340, .xc = 2.748700, .yc = 0.748740, .distance = 0.556390};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest604)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.374190, .xc = 2.264600, .yc = 1.264600, .distance = 0.811870};
  maneuver expected2 = {.radius = -0.374190, .xc = 3.000000, .yc = 1.125800, .distance = 0.517980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest605)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.360540, .xc = 2.254900, .yc = 1.254900, .distance = 0.569940};
  maneuver expected2 = {.radius = 1000.000000, .xc = 9.897600, .yc = -5.748700, .distance = 0.696550};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest606)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.585790, .xc = 2.414200, .yc = 1.414200, .distance = 1.380200};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1003.000000, .yc = 1.456800, .distance = 0.085787};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest607)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = 0.528550, .xc = 2.373700, .yc = 1.373700, .distance = -1.655200};
  maneuver expected2 = {.radius = -1000.000000, .xc = 70005.000000, .yc = 71418.000000, .distance = -0.358860};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest608)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = 0.292890, .xc = 2.207100, .yc = 1.207100, .distance = -0.690110};
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.603200, .yc = 1001.500000, .distance = -0.792890};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest609)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = 0.292890, .xc = 2.207100, .yc = 1.207100, .distance = -0.690110};
  maneuver expected2 = {.radius = -1000.000000, .xc = 2.603900, .yc = 1001.500000, .distance = -0.792890};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest610)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.352380, .xc = 2.249200, .yc = 1.249200, .distance = 1.106400};
  maneuver expected2 = {.radius = -0.352380, .xc = 2.748400, .yc = 1.746700, .distance = 1.656400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest611)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.291240, .xc = 2.205900, .yc = 1.205900, .distance = 0.840350};
  maneuver expected2 = {.radius = -0.291240, .xc = 2.708800, .yc = 1.500000, .distance = 1.069100};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest612)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 3.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.295130, .xc = 2.206600, .yc = 1.210800, .distance = 0.732830};
  maneuver expected2 = {.radius = -0.295130, .xc = 2.791300, .yc = 1.291300, .distance = 0.735780};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest613)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = 0.139750, .xc = 2.098800, .yc = 1.098800, .distance = -0.278770};
  maneuver expected2 = {.radius = -0.139750, .xc = 2.000000, .yc = 1.360300, .distance = -0.388520};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest614)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 0.146750, .xc = 2.103800, .yc = 1.103800, .distance = -0.346290};
  maneuver expected2 = {.radius = -0.146750, .xc = 2.104800, .yc = 1.397300, .distance = -0.578280};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest615)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = 0.180240, .xc = 2.141600, .yc = 1.141600, .distance = -0.444050};
  maneuver expected2 = {.radius = -0.180240, .xc = 2.180200, .yc = 1.500000, .distance = -0.868730};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest616)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -705.230000, .yc = -705.980000, .distance = -0.351790};
  maneuver expected2 = {.radius = 0.176780, .xc = 1.876300, .yc = 1.373800, .distance = -0.553610};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest617)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -705.180000, .yc = -706.030000, .distance = -0.207110};
  maneuver expected2 = {.radius = 0.207110, .xc = 2.000000, .yc = 1.292900, .distance = -0.487980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest618)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -705.180000, .yc = -706.030000, .distance = -0.207110};
  maneuver expected2 = {.radius = 0.207110, .xc = 2.000000, .yc = 1.292900, .distance = -0.487980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest619)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = 0.353540, .xc = 2.250000, .yc = 1.250000, .distance = -0.551800};
  maneuver expected2 = {.radius = -1000.000000, .xc = -5.143100, .yc = 8.499200, .distance = -0.003554};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest620)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = 0.048284, .xc = 2.048300, .yc = 1.048300, .distance = -0.037922};
  maneuver expected2 = {.radius = -1000.000000, .xc = -998.000000, .yc = 1.273800, .distance = -0.451720};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest621)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 2.000000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = 0.175900, .xc = 2.123100, .yc = 1.125600, .distance = -0.278940};
  maneuver expected2 = {.radius = -0.175900, .xc = 1.875600, .yc = 1.375600, .distance = -0.277190};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest622)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -705.330000, .yc = -705.860000, .distance = -0.658820};
  maneuver expected2 = {.radius = -0.068284, .xc = 1.500000, .yc = 1.431700, .distance = -0.053630};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest623)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -705.340000, .yc = -705.850000, .distance = -0.687110};
  maneuver expected2 = {.radius = -0.020000, .xc = 1.514100, .yc = 1.485900, .distance = -0.031416};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest624)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -705.340000, .yc = -705.850000, .distance = -0.698820};
  maneuver expected2 = {.radius = -0.011716, .xc = 1.511700, .yc = 1.500000, .distance = -0.027604};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest625)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -705.340000, .yc = -705.840000, .distance = -0.707110};
  maneuver expected2 = {.radius = -0.010000, .xc = 1.507100, .yc = 1.507100, .distance = -0.031417};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest626)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.262340, .xc = 1.828600, .yc = 0.828640, .distance = -0.383610};
  maneuver expected2 = {.radius = 0.262340, .xc = 1.500000, .yc = 1.237700, .distance = -1.001700};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest627)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.262340, .xc = 1.828600, .yc = 0.828640, .distance = -0.383610};
  maneuver expected2 = {.radius = 0.262340, .xc = 1.500000, .yc = 1.237700, .distance = -1.001700};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest628)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.254810, .xc = 1.834000, .yc = 0.833970, .distance = -0.278240};
  maneuver expected2 = {.radius = 0.254810, .xc = 1.680200, .yc = 1.319800, .distance = -0.678490};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest629)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.386500, .xc = 1.740800, .yc = 0.740840, .distance = -0.230290};
  maneuver expected2 = {.radius = 0.386500, .xc = 1.886500, .yc = 1.500000, .distance = -0.533850};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest630)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.500000, .y = 1.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -6.071100, .xc = -2.321500, .yc = -3.235600, .distance = -0.414460};
  maneuver expected2 = {.radius = 6.071100, .xc = 5.749700, .yc = 5.835600, .distance = -0.293040};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest631)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.566900, .xc = 1.613300, .yc = 0.613280, .distance = -0.769240};
  maneuver expected2 = {.radius = 0.566900, .xc = 1.000000, .yc = 1.566900, .distance = -0.323990};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest632)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -705.110000, .yc = -706.100000, .distance = -0.007036};
  maneuver expected2 = {.radius = -0.700110, .xc = 1.500000, .yc = 0.509930, .distance = -1.106700};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest633)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -1000.000000, .xc = -705.250000, .yc = -705.960000, .distance = -0.414210};
  maneuver expected2 = {.radius = -0.414210, .xc = 1.414200, .yc = 1.000000, .distance = -0.975970};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest634)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.350010, .xc = 1.752500, .yc = 0.752510, .distance = 1.096100};
  maneuver expected2 = {.radius = 1000.000000, .xc = -70001.000000, .yc = -71416.000000, .distance = 0.710640};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest635)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.008284, .xc = 2.008300, .yc = 1.008300, .distance = 0.019519};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1.504500, .yc = -999.000000, .distance = 1.008300};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest636)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.008285, .xc = 2.008300, .yc = 1.008300, .distance = 0.019520};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1.503800, .yc = -999.000000, .distance = 1.008300};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest637)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.292290, .xc = 1.793300, .yc = 0.793320, .distance = -0.687660};
  maneuver expected2 = {.radius = 0.292290, .xc = 1.208700, .yc = 0.795400, .distance = -1.143900};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest638)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.279490, .xc = 1.802400, .yc = 0.802370, .distance = -0.557540};
  maneuver expected2 = {.radius = 0.279490, .xc = 1.279500, .yc = 1.000000, .distance = -0.777050};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest639)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 1.000000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.355340, .xc = 1.751300, .yc = 0.746240, .distance = -0.552830};
  maneuver expected2 = {.radius = 0.355340, .xc = 1.251300, .yc = 1.251300, .distance = -0.556390};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest640)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.000000 };
  maneuver expected1 = {.radius = -0.374190, .xc = 1.735400, .yc = 0.735410, .distance = -0.811870};
  maneuver expected2 = {.radius = 0.374190, .xc = 1.000000, .yc = 0.874190, .distance = -0.517980};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest641)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.360540, .xc = 1.745100, .yc = 0.745060, .distance = -0.569940};
  maneuver expected2 = {.radius = -1000.000000, .xc = -5.897600, .yc = 7.748700, .distance = -0.696550};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest642)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 1.570800 };
  maneuver expected1 = {.radius = -0.585790, .xc = 1.585800, .yc = 0.585790, .distance = -1.380200};
  maneuver expected2 = {.radius = -1000.000000, .xc = -999.000000, .yc = 0.543220, .distance = -0.085787};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest643)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 2.356200 };
  maneuver expected1 = {.radius = -0.528550, .xc = 1.626300, .yc = 0.626260, .distance = 1.655200};
  maneuver expected2 = {.radius = 1000.000000, .xc = -70001.000000, .yc = -71416.000000, .distance = 0.358860};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest644)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = 3.141600 };
  maneuver expected1 = {.radius = -0.292890, .xc = 1.792900, .yc = 0.792890, .distance = 0.690110};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1.396800, .yc = -999.500000, .distance = 0.792890};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest645)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -3.141600 };
  maneuver expected1 = {.radius = -0.292890, .xc = 1.792900, .yc = 0.792890, .distance = 0.690110};
  maneuver expected2 = {.radius = 1000.000000, .xc = 1.396100, .yc = -999.500000, .distance = 0.792890};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest646)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -2.356200 };
  maneuver expected1 = {.radius = -0.352380, .xc = 1.750800, .yc = 0.750830, .distance = -1.106400};
  maneuver expected2 = {.radius = 0.352380, .xc = 1.251600, .yc = 0.253330, .distance = -1.656400};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest647)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -1.570800 };
  maneuver expected1 = {.radius = -0.291240, .xc = 1.794100, .yc = 0.794060, .distance = -0.840350};
  maneuver expected2 = {.radius = 0.291240, .xc = 1.291200, .yc = 0.500000, .distance = -1.069100};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests2, waypoint2maneuversTest648)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = -0.785400 };
  pose finalDestination = {.x = 1.000000, .y = 0.500000, .theta = -0.785400 };
  maneuver expected1 = {.radius = -0.295130, .xc = 1.793400, .yc = 0.789240, .distance = -0.732830};
  maneuver expected2 = {.radius = 0.295130, .xc = 1.208700, .yc = 0.708690, .distance = -0.735780};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}




// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}