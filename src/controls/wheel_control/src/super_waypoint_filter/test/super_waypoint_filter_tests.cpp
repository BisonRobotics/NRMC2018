#include <gtest/gtest.h>
#include <super_waypoint_filter/super_waypoint_filter.h>
#include <gmock/gmock.h>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

TEST(superWaypointFilterTests, addPointFromBehind1)
{
 SuperWaypointFilter smfw;
 std::vector<geometry_msgs::Pose2D> startPath;
 geometry_msgs::Pose2D point;
 point.x = 1.7;
 point.y = 0;
 point.theta = .3;
 startPath.clear();
 startPath.push_back(point);
 ASSERT_TRUE (smfw.interpolateAndAddPoint(&startPath, 1.2) == 1);
 ASSERT_TRUE (startPath.size() == 2);
 ASSERT_NEAR (startPath.at(0).theta, point.theta, .001); //sets theta to 0
 ASSERT_NEAR (startPath.at(0).x, 1.2, .001);
 ASSERT_NEAR (startPath.at(0).y, -.148, .01);
}

TEST(superWaypointFilterTests, interpolateTest1)
{
  ASSERT_NEAR (SuperWaypointFilter::interpolateYFromXAndTwoPoints(0,0,1,1,.5), .5, .001);
}

TEST(superWaypointFilterTests, interpolateTest2)
{
  ASSERT_NEAR (SuperWaypointFilter::interpolateYFromXAndTwoPoints(0,0,-1,1,.5), -.5, .001);
}

TEST(superWaypointFilterTests, interpolateTest3)
{
  ASSERT_NEAR (SuperWaypointFilter::interpolateYFromXAndTwoPoints(0,0,1,-1,.5), -.5, .001);
}

TEST(superWaypointFilterTests, interpolateTest4)
{
  ASSERT_NEAR (SuperWaypointFilter::interpolateYFromXAndTwoPoints(0,0,-1,-1,.5), .5, .001);
}

TEST(superWaypointFilterTests, addPointFromBehind2)
{
 SuperWaypointFilter smfw;
 std::vector<geometry_msgs::Pose2D> startPath;
 geometry_msgs::Pose2D point;
 point.x = 2.0;
 point.y = 0;
 point.theta = 1.56; //would need to go over about 10 m to go back 
                    //using this angle
 startPath.clear();
 startPath.push_back(point);
 ASSERT_TRUE (smfw.interpolateAndAddPoint(&startPath, 1.0) == 1);
 ASSERT_TRUE (startPath.size() == 2);
 ASSERT_NEAR (startPath.at(0).theta, 0, .001);
 ASSERT_NEAR (startPath.at(0).x, 1.0, .001);
 ASSERT_NEAR (startPath.at(0).y, point.y, .01); //so we take the y from the point
}

TEST(superWaypointFilterTests, contructPathFromOneWaypointInGoal)
{
  SuperWaypointFilter smfw;
  std::vector<geometry_msgs::Pose2D> startPath;
  geometry_msgs::Pose2D point;
  point.x = 6.0;
  point.y = 0;
  point.theta = 0;

  startPath.clear();
  startPath.push_back(point);

  smfw.filterWaypoints(startPath);
  ASSERT_TRUE (smfw.getForwardPath().size() > 6) <<"Actual size was: "<< smfw.getForwardPath().size();

}

TEST(superWaypointFilterTests, contructPathWaypointsInObstacleFieldAndGoal)
{
  SuperWaypointFilter smfw;
  std::vector<geometry_msgs::Pose2D> startPath;
  geometry_msgs::Pose2D point;
  point.x = 2.0;
  point.y = .8;
  point.theta = 0;
  startPath.clear();
  startPath.push_back(point);
  point.x = 6.0;
  point.y = 0;
  point.theta = 0;
  startPath.push_back(point);

  smfw.filterWaypoints(startPath);
  ASSERT_TRUE (smfw.getForwardPath().size() > 6) <<"Actual size was: "<< smfw.getForwardPath().size();
}

TEST(superWaypointFilterTests, contructPathWaypointsInObstacleFieldAndGoalAndGetBackwardPath)
{
  SuperWaypointFilter smfw;
  std::vector<geometry_msgs::Pose2D> startPath;
  geometry_msgs::Pose2D point;
  point.x = 2.0;
  point.y = .8;
  point.theta = 0;
  startPath.clear();
  startPath.push_back(point);
  point.x = 6.0;
  point.y = 0;
  point.theta = 0;
  startPath.push_back(point);

  smfw.filterWaypoints(startPath);
  ASSERT_TRUE (smfw.getForwardPath().size() > 6) <<"Actual size was: "<< smfw.getForwardPath().size();
  ASSERT_TRUE (smfw.getBackwardPath().size() > 6) <<"Actual size was: "<< smfw.getBackwardPath().size();
}

TEST(SuperWaypointFilterTests, ableToGetBackwardPath2)
{
  SuperWaypointFilter smfw;
  std::vector<geometry_msgs::Pose2D> startPath;
  geometry_msgs::Pose2D point;
  point.x = .5002;
  point.y = .66289;
  point.theta = -.2686;
  startPath.clear();
  startPath.push_back(point);
  point.x = 1.464;
  point.y = .3975;
  point.theta = .294;
  startPath.push_back(point);
  point.x = 2.269;
  point.y = .6414;
  point.theta = -.40362;
  startPath.push_back(point);
  point.x = 5.487;
  point.y = -.743132;
  point.theta = .9667;
  startPath.push_back(point);
  point.x = 6.0;
  point.y = 0;
  point.theta = 0;
  startPath.push_back(point);
  
  smfw.filterWaypoints(startPath);
  ASSERT_TRUE (smfw.getForwardPath().size() > 6) <<"Actual size was: "<< smfw.getForwardPath().size();
  ASSERT_TRUE (smfw.getBackwardPath().size() > 6) <<"Actual size was: "<< smfw.getBackwardPath().size();
}

TEST(SuperWaypointFilterTests, ableToGetBackwardPathNoGoal)
{
  SuperWaypointFilter smfw;
  std::vector<geometry_msgs::Pose2D> startPath;
  geometry_msgs::Pose2D point;
  point.x = .5002;
  point.y = .66289;
  point.theta = -.2686;
  startPath.clear();
  startPath.push_back(point);
  point.x = 1.464;
  point.y = .3975;
  point.theta = .294;
  startPath.push_back(point);
  point.x = 2.269;
  point.y = .6414;
  point.theta = -.40362;
  startPath.push_back(point);
  
  smfw.filterWaypoints(startPath);
  ASSERT_TRUE (smfw.getForwardPath().size() > 6) <<"Actual size was: "<< smfw.getForwardPath().size();
  ASSERT_TRUE (smfw.getBackwardPath().size() > 6) <<"Actual size was: "<< smfw.getBackwardPath().size();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}