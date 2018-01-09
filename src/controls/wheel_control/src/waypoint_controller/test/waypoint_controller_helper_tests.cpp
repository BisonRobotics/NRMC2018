#include <gtest/gtest.h>
#include <waypoint_controller/waypoint_controller_helper.h>
#include <vector>
#include <utility>
#define _USE_MATH_DEFINES
#include <cmath>

#define APPROX(A, B, T) ((A > B - T && A < B + T) ? true : false)
#define WAYPOINT2MANEUVERTOL .009

TEST(WaypointControllerHelperTests, anglediffWorks1)
{
  float angle1, angle2, result;
  angle1 = 2.0;
  angle2 = -2.0;
  result = -2.283;
  EXPECT_NEAR(anglediff(angle1, angle2), result, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, anglediffWorks2)
{
  float angle1, angle2, result;
  angle1 = 0;
  angle2 = 4;
  result = 2.283;
  EXPECT_NEAR(anglediff(angle1, angle2), result, WAYPOINT2MANEUVERTOL);
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
  pose finalDestination = {.x = 3, .y = 1, .theta = M_PI / 4 };

  std::vector<std::pair<float, float> > points;
  std::vector<maneuver> myMans;
  myMans.push_back(myMan1);
  myMans.push_back(myMan2);

  waypointWithManeuvers wpWithMan = {.initialPose = initialPose, .terminalPose = finalDestination, .mans = myMans };

  points = waypointWithManeuvers2points(wpWithMan);

  EXPECT_TRUE(points.size() > (myMan1.distance + myMan2.distance) * 10)
      << "Expected at least " << (myMan1.distance + myMan2.distance) * 10 << " points, but got "
      << points.size();  // point at least every 10cm
}

TEST(WaypointControllerHelperTests, speedAndRadius2WheelVelsTests)
{
  std::pair<float, float> returnSpeeds;
  std::pair<float, float> expectedSpeeds;
  float AxelLen = .5f;
  float maxSpeed = .5f;
  float turnRadius = 1.0f;
  float speed = .3f;
  expectedSpeeds.first = .2250f;
  expectedSpeeds.second = .3750f;

  returnSpeeds = speedAndRadius2WheelVels(speed, turnRadius, AxelLen, maxSpeed);
  EXPECT_TRUE(APPROX(returnSpeeds.first, expectedSpeeds.first, .001) &&
              APPROX(returnSpeeds.second, expectedSpeeds.second, .001))
      << "Expected " << returnSpeeds.first << " = " << expectedSpeeds.first << " and " << returnSpeeds.second << " = "
      << expectedSpeeds.second;
}


TEST(WaypointControllerHelperTests, waypoint2maneuversTest1)
{

  pose initialPose = {.x = 0, .y = 0, .theta = 0 };
  pose finalDestination = {.x = 3, .y = 1, .theta = M_PI_4 };
  maneuver expected1 = {.radius = 1000, .xc = .58579 / 2.0f, .yc = 1000, .distance = .58579 };
  maneuver expected2 = {.radius = 3.4142, .xc = .58579, .yc = 3.4142, .distance = 2.6815 };
  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);

  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);
  
  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);
  
  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest2)
{
  pose initialPose = {.x = 0.000000, .y = 0.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 3.000000, .y = 1.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = 1000.000000, .xc = 0.292890, .yc = 1000.000000, .distance = 0.585790};
  maneuver expected2 = {.radius = 3.414200, .xc = 0.585790, .yc = 3.414200, .distance = 2.681500};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest3)
{
  pose initialPose = {.x = 2.000000, .y = 1.000000, .theta = 0.000000 };
  pose finalDestination = {.x = 1.000000, .y = 0.000000, .theta = 0.785400 };
  maneuver expected1 = {.radius = -0.000009, .xc = 2.000000, .yc = 0.999990, .distance = -0.000007};
  maneuver expected2 = {.radius = -1000.000000, .xc = -705.610000, .yc = 707.600000, .distance = -1.414200};

  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);
  EXPECT_TRUE(myMans.size() ==2);
  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);

  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);
  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);

}


/*
TEST(WaypointControllerHelperTests, waypoint2maneuversTest1)
{
  pose initialPose = {.x = 0, .y = 0, .theta = 0 };
  pose finalDestination = {.x = 3, .y = 1, .theta = M_PI / 4 };
  maneuver expected1 = {.radius = 1000, .xc = .58579 / 2, .yc = 1000, .distance = .58579 };
  maneuver expected2 = {.radius = 3.4142, .xc = .58579, .yc = 3.4142, .distance = 2.6815 };
  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);

  EXPECT_TRUE(myMans.size() >= 2) << "NOT ENGOUGH MANEUVERS!\n";

  EXPECT_TRUE(APPROX(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL))
      << "First Dist of: " << myMans.at(0).distance << " not " << expected1.distance << "\n";
  EXPECT_TRUE(myMans.at(0).radius > 10) << "Radius of first maneuver not > 10";
  EXPECT_TRUE(APPROX(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL))
      << "Second Dist of: " << myMans.at(1).distance << " not " << expected2.distance << "\n";
  ;
  EXPECT_TRUE(APPROX(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL))
      << "Second radius of: " << myMans.at(1).radius << " not " << expected2.radius << "\n";
  ;
  EXPECT_TRUE(APPROX(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL)) << "Second xc of: " << myMans.at(1).xc
                                                                           << " not " << expected2.xc << "\n";
  ;
  EXPECT_TRUE(APPROX(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL)) << "Second yc of: " << myMans.at(1).yc
                                                                           << " not " << expected2.yc << "\n";
  ;
}

TEST(WaypointControllerHelperTests, waypoint2maneuversTest2)
{
  pose initialPose = {.x = 0, .y = 0, .theta = 0 };
  pose finalDestination = {.x = 3, .y = -1, .theta = -M_PI / 4 };
  maneuver expected1 = {.radius = 1000, .xc = .58579 / 2, .yc = 1000, .distance = .58579 };
  maneuver expected2 = {.radius = -3.4142, .xc = .58579, .yc = -3.4142, .distance = 2.6815 };
  std::vector<maneuver> myMans;

  myMans = waypoint2maneuvers(initialPose, finalDestination);

  EXPECT_TRUE(myMans.size() >= 2) << "NOT ENGOUGH MANEUVERS!\n";

  EXPECT_TRUE(APPROX(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL))
      << "First Dist of: " << myMans.at(0).distance << " not " << expected1.distance << "\n";
  EXPECT_TRUE(myMans.at(0).radius > 10) << "Radius of first maneuver not > 10";
  EXPECT_TRUE(APPROX(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL))
      << "Second Dist of: " << myMans.at(1).distance << " not " << expected2.distance << "\n";
  ;
  EXPECT_TRUE(APPROX(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL))
      << "Second radius of: " << myMans.at(1).radius << " not " << expected2.radius << "\n";
  ;
  EXPECT_TRUE(APPROX(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL)) << "Second xc of: " << myMans.at(1).xc
                                                                           << " not " << expected2.xc << "\n";
  ;
  EXPECT_TRUE(APPROX(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL)) << "Second yc of: " << myMans.at(1).yc
                                                                           << " not " << expected2.yc << "\n";
  ;
}

TEST(WaypointControllerHelperTests, oneturnTestSuite1)
{
  pose initialPose = {.x = 0, .y = 0, .theta = 0 };
  pose finalDestination = {.x = 3, .y = 1, .theta = M_PI / 3 };
  maneuver expected1 = {.radius = 1000, .xc = 1.2679 / 2, .yc = 1000, .distance = 1.2679 };
  maneuver expected2 = {.radius = 2, .xc = 1.2679, .yc = 2, .distance = 2.0944 };
  std::vector<maneuver> myMans;

  myMans = oneTurnSolution(initialPose, finalDestination);

  EXPECT_TRUE(myMans.size() >= 2) << "NOT ENGOUGH MANEUVERS!\n";

  EXPECT_TRUE(APPROX(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL))
      << "First Dist of: " << myMans.at(0).distance << " not " << expected1.distance << "\n";
  EXPECT_TRUE(myMans.at(0).radius > 100) << "Radius of first maneuver not > 100";
  EXPECT_TRUE(APPROX(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL))
      << "Second Dist of: " << myMans.at(1).distance << " not " << expected2.distance << "\n";
  ;
  EXPECT_TRUE(APPROX(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL))
      << "Second radius of: " << myMans.at(1).radius << " not " << expected2.radius << "\n";
  ;
  EXPECT_TRUE(APPROX(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL)) << "Second xc of: " << myMans.at(1).xc
                                                                           << " not " << expected2.xc << "\n";
  ;
  EXPECT_TRUE(APPROX(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL)) << "Second yc of: " << myMans.at(1).yc
                                                                           << " not " << expected2.yc << "\n";
  ;
}
*/
// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
