#include <gtest/gtest.h>
#include <super_waypoint_filter/super_waypoint_filter.h>
#include <gmock/gmock.h>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#define MIN_DISTANCE_FOR_RUN 2.5
#define OBSTACLE_ZONE_START_X 1.5
#define OBSTACLE_ZONE_END_X 4.44
#define MIN_WAYPOINT_DISTANCE .8

#define FIELD_WIDTH_2 1.89

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

TEST(SuperWaypointFilterTests, PathsEnterAndExitObstacleZone1)
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
  
  //make sure we got the begining, end, 4 midpoints and added points
  ASSERT_TRUE (smfw.getForwardPath().size() >= 8) <<"Actual size was: "<< smfw.getForwardPath().size();
  ASSERT_TRUE (smfw.getBackwardPath().size() >= 10) <<"Actual size was: "<< smfw.getBackwardPath().size();
  
  //make sure all obstacle zone points are in obstacle zone
  for (int index =0; index < smfw.getForwardPath().size() - 3; index++) //two waypoints are added to the end
  {
      ASSERT_TRUE(smfw.getForwardPath().at(index).x >= 1.490 && smfw.getForwardPath().at(index).x <= 4.45);
      ASSERT_TRUE(smfw.getForwardPath().at(index).y >= -FIELD_WIDTH_2 - .01 && smfw.getForwardPath().at(index).y <= FIELD_WIDTH_2 + .01);
  }
  
  for (int index =0; index < smfw.getBackwardPath().size() - 5; index++) //four waypoints are added to the end
  {
      ASSERT_TRUE(smfw.getBackwardPath().at(index).x >= 1.490 && smfw.getBackwardPath().at(index).x <= 4.45);
      ASSERT_TRUE(smfw.getBackwardPath().at(index).y >= -FIELD_WIDTH_2 - .01 && smfw.getBackwardPath().at(index).y <= FIELD_WIDTH_2 + .01);
  }
  //make sure obstacle zone entrance and exit points have the right index
  ASSERT_NEAR(smfw.getForwardPath().at(0).x, 1.5, .01);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 3).x, 4.44, .01);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(0).x, 4.44, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 5).x, 1.5, .01);
  //make sure added points get added
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 2).x, 5.4, .01);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 2).y, smfw.getForwardPath().at(smfw.getForwardPath().size() - 3).y, .1);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 2).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 1).x, 6.4, .01);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 1).y, smfw.getForwardPath().at(smfw.getForwardPath().size() - 3).y, .1);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 1).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 4).x, .8, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 4).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 4).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 3).x, 1.4, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 3).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 3).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 2).x, .8, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 2).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 2).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 1).x, -.8, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 1).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 1).theta, 0, .1);
}


TEST(SuperWaypointFilterTests, PathsEnterAndExitObstacleZone2)
{
  SuperWaypointFilter smfw;
  std::vector<geometry_msgs::Pose2D> startPath;
  geometry_msgs::Pose2D point;
  point.x = 1.7002;
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
  
  //make sure we got the begining, end, 4 midpoints and added points
  ASSERT_TRUE (smfw.getForwardPath().size() >= 8) <<"Actual size was: "<< smfw.getForwardPath().size();
  ASSERT_TRUE (smfw.getBackwardPath().size() >= 10) <<"Actual size was: "<< smfw.getBackwardPath().size();
  
  //make sure all obstacle zone points are in obstacle zone
  for (int index =0; index < smfw.getForwardPath().size() - 3; index++) //two waypoints are added to the end
  {
      ASSERT_TRUE(smfw.getForwardPath().at(index).x >= 1.490 && smfw.getForwardPath().at(index).x <= 4.45);
      ASSERT_TRUE(smfw.getForwardPath().at(index).y >= -FIELD_WIDTH_2 - .01 && smfw.getForwardPath().at(index).y <= FIELD_WIDTH_2 + .01);
  }
  
  for (int index =0; index < smfw.getBackwardPath().size() - 5; index++) //four waypoints are added to the end
  {
      ASSERT_TRUE(smfw.getBackwardPath().at(index).x >= 1.490 && smfw.getBackwardPath().at(index).x <= 4.45);
      ASSERT_TRUE(smfw.getBackwardPath().at(index).y >= -FIELD_WIDTH_2 - .01 && smfw.getBackwardPath().at(index).y <= FIELD_WIDTH_2 + .01);
  }
  //make sure obstacle zone entrance and exit points have the right index
  ASSERT_NEAR(smfw.getForwardPath().at(0).x, 1.5, .01);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 3).x, 4.44, .01);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(0).x, 4.44, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 5).x, 1.5, .01);
  //make sure added points get added
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 2).x, 5.4, .01);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 2).y, smfw.getForwardPath().at(smfw.getForwardPath().size() - 3).y, .1);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 2).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 1).x, 6.4, .01);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 1).y, smfw.getForwardPath().at(smfw.getForwardPath().size() - 3).y, .1);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 1).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 4).x, .8, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 4).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 4).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 3).x, 1.4, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 3).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 3).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 2).x, .8, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 2).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 2).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 1).x, -.8, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 1).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 1).theta, 0, .1);
}

TEST(SuperWaypointFilterTests, PathsEnterAndExitObstacleZone3)
{
  SuperWaypointFilter smfw;
  std::vector<geometry_msgs::Pose2D> startPath;
  geometry_msgs::Pose2D point;
  startPath.clear();
  point.x = 1.464;
  point.y = .3975;
  point.theta = .294;
  startPath.push_back(point);
  point.x = 2.269;
  point.y = .6414;
  point.theta = -.40362;
  startPath.push_back(point);
  point.x = 4.269;
  point.y = -.6414;
  point.theta = -.20362;
  startPath.push_back(point);
  point.x =6;
  point.y = 0;
  point.theta = 0;
  startPath.push_back(point);
  
  smfw.filterWaypoints(startPath);
  
  //make sure we got the begining, end, 4 midpoints and added points
  ASSERT_TRUE (smfw.getForwardPath().size() >= 8) <<"Actual size was: "<< smfw.getForwardPath().size();
  ASSERT_TRUE (smfw.getBackwardPath().size() >= 10) <<"Actual size was: "<< smfw.getBackwardPath().size();
  
  //make sure all obstacle zone points are in obstacle zone
  for (int index =0; index < smfw.getForwardPath().size() - 3; index++) //two waypoints are added to the end
  {
      ASSERT_TRUE(smfw.getForwardPath().at(index).x >= 1.490 && smfw.getForwardPath().at(index).x <= 4.45);
      ASSERT_TRUE(smfw.getForwardPath().at(index).y >= -FIELD_WIDTH_2 - .01 && smfw.getForwardPath().at(index).y <= FIELD_WIDTH_2 + .01);
  }
  
  for (int index =0; index < smfw.getBackwardPath().size() - 5; index++) //four waypoints are added to the end
  {
      ASSERT_TRUE(smfw.getBackwardPath().at(index).x >= 1.490 && smfw.getBackwardPath().at(index).x <= 4.45);
      ASSERT_TRUE(smfw.getBackwardPath().at(index).y >= -FIELD_WIDTH_2 - .01 && smfw.getBackwardPath().at(index).y <= FIELD_WIDTH_2 + .01);
  }
  //make sure obstacle zone entrance and exit points have the right index
  ASSERT_NEAR(smfw.getForwardPath().at(0).x, 1.5, .01);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 3).x, 4.44, .01);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(0).x, 4.44, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 5).x, 1.5, .01);
  //make sure added points get added
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 2).x, 5.4, .01);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 2).y, smfw.getForwardPath().at(smfw.getForwardPath().size() - 3).y, .1);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 2).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 1).x, 6.4, .01);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 1).y, smfw.getForwardPath().at(smfw.getForwardPath().size() - 3).y, .1);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 1).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 4).x, .8, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 4).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 4).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 3).x, 1.4, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 3).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 3).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 2).x, .8, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 2).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 2).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 1).x, -.8, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 1).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 1).theta, 0, .1);
}

TEST(SuperWaypointFilterTests, PathsEnterAndExitObstacleZone4)
{
  SuperWaypointFilter smfw;
  std::vector<geometry_msgs::Pose2D> startPath;
  geometry_msgs::Pose2D point;
  startPath.clear();
  point.x = 2.64;
  point.y = .3975;
  point.theta = .294;
  startPath.push_back(point);
  point.x = 3.269;
  point.y = .6414;
  point.theta = -.40362;
  startPath.push_back(point);
  point.x = 4.269;
  point.y = -.6414;
  point.theta = -.20362;
  startPath.push_back(point);
  point.x =6;
  point.y = 0;
  point.theta = 0;
  startPath.push_back(point);
  
  smfw.filterWaypoints(startPath);
  
  //make sure we got the begining, end, 4 midpoints and added points
  ASSERT_TRUE (smfw.getForwardPath().size() >= 8) <<"Actual size was: "<< smfw.getForwardPath().size();
  ASSERT_TRUE (smfw.getBackwardPath().size() >= 10) <<"Actual size was: "<< smfw.getBackwardPath().size();
  
  //make sure all obstacle zone points are in obstacle zone
  for (int index =0; index < smfw.getForwardPath().size() - 3; index++) //two waypoints are added to the end
  {
      ASSERT_TRUE(smfw.getForwardPath().at(index).x >= 1.490 && smfw.getForwardPath().at(index).x <= 4.45);
      ASSERT_TRUE(smfw.getForwardPath().at(index).y >= -FIELD_WIDTH_2 - .01 && smfw.getForwardPath().at(index).y <= FIELD_WIDTH_2 + .01);
  }
  
  for (int index =0; index < smfw.getBackwardPath().size() - 5; index++) //four waypoints are added to the end
  {
      ASSERT_TRUE(smfw.getBackwardPath().at(index).x >= 1.490 && smfw.getBackwardPath().at(index).x <= 4.45);
      ASSERT_TRUE(smfw.getBackwardPath().at(index).y >= -FIELD_WIDTH_2 - .01 && smfw.getBackwardPath().at(index).y <= FIELD_WIDTH_2 + .01);
  }
  //make sure obstacle zone entrance and exit points have the right index
  ASSERT_NEAR(smfw.getForwardPath().at(0).x, 1.5, .01);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 3).x, 4.44, .01);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(0).x, 4.44, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 5).x, 1.5, .01);
  //make sure added points get added
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 2).x, 5.4, .01);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 2).y, smfw.getForwardPath().at(smfw.getForwardPath().size() - 3).y, .1);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 2).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 1).x, 6.4, .01);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 1).y, smfw.getForwardPath().at(smfw.getForwardPath().size() - 3).y, .1);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 1).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 4).x, .8, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 4).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 4).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 3).x, 1.4, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 3).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 3).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 2).x, .8, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 2).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 2).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 1).x, -.8, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 1).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 1).theta, 0, .1);
}

TEST(SuperWaypointFilterTests, PathsEnterAndExitObstacleZone5)
{
  SuperWaypointFilter smfw;
  std::vector<geometry_msgs::Pose2D> startPath;
  geometry_msgs::Pose2D point;
  startPath.clear();
  point.x = 1.464;
  point.y = .3975;
  point.theta = .294;
  startPath.push_back(point);
  
  smfw.filterWaypoints(startPath);
  
  //make sure we got the begining, end, 4 midpoints and added points
  ASSERT_TRUE (smfw.getForwardPath().size() >= 8) <<"Actual size was: "<< smfw.getForwardPath().size();
  ASSERT_TRUE (smfw.getBackwardPath().size() >= 10) <<"Actual size was: "<< smfw.getBackwardPath().size();
  
  //make sure all obstacle zone points are in obstacle zone
  for (int index =0; index < smfw.getForwardPath().size() - 3; index++) //two waypoints are added to the end
  {
      ASSERT_TRUE(smfw.getForwardPath().at(index).x >= 1.490 && smfw.getForwardPath().at(index).x <= 4.45);
      ASSERT_TRUE(smfw.getForwardPath().at(index).y >= -FIELD_WIDTH_2 - .01 && smfw.getForwardPath().at(index).y <= FIELD_WIDTH_2 + .01);
  }
  
  for (int index =0; index < smfw.getBackwardPath().size() - 5; index++) //four waypoints are added to the end
  {
      ASSERT_TRUE(smfw.getBackwardPath().at(index).x >= 1.490 && smfw.getBackwardPath().at(index).x <= 4.45);
      ASSERT_TRUE(smfw.getBackwardPath().at(index).y >= -FIELD_WIDTH_2 - .01 && smfw.getBackwardPath().at(index).y <= FIELD_WIDTH_2 + .01);
  }
  //make sure obstacle zone entrance and exit points have the right index
  ASSERT_NEAR(smfw.getForwardPath().at(0).x, 1.5, .01);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 3).x, 4.44, .01);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(0).x, 4.44, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 5).x, 1.5, .01);
  //make sure added points get added
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 2).x, 5.4, .01);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 2).y, smfw.getForwardPath().at(smfw.getForwardPath().size() - 3).y, .1);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 2).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 1).x, 6.4, .01);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 1).y, smfw.getForwardPath().at(smfw.getForwardPath().size() - 3).y, .1);
  ASSERT_NEAR(smfw.getForwardPath().at(smfw.getForwardPath().size() - 1).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 4).x, .8, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 4).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 4).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 3).x, 1.4, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 3).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 3).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 2).x, .8, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 2).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 2).theta, 0, .1);
  
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 1).x, -.8, .01);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 1).y, 0, .1);
  ASSERT_NEAR(smfw.getBackwardPath().at(smfw.getBackwardPath().size() - 1).theta, 0, .1);
}
// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}