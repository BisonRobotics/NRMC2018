#include <gtest/gtest.h>
#include <pose_estimate_filter/pose_estimate_filter.h>

using namespace apriltag_tracker;

TEST(PoseEstimateFilterTests, TrivialTest)
{
  ASSERT_EQ(true, true);
}

TEST(PoseEstimateFilterTests, getRPY)
{
  tf2::Quaternion q;
  double roll, pitch, yaw;

  q.setRPY(0.0, 0.0, 0.0);
  PoseEstimateFilter::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,   0.0, 1e-10);
  ASSERT_NEAR(pitch,  0.0, 1e-10);
  ASSERT_NEAR(yaw,    0.0, 1e-10);

  q.setRPY(0.0, 0.0, 1.0);
  PoseEstimateFilter::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,   0.0, 1e-10);
  ASSERT_NEAR(pitch,  0.0, 1e-10);
  ASSERT_NEAR(yaw,    1.0, 1e-10);

  q.setRPY(0.0, 0.0, -1.0);
  PoseEstimateFilter::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,   0.0, 1e-10);
  ASSERT_NEAR(pitch,  0.0, 1e-10);
  ASSERT_NEAR(yaw,   -1.0, 1e-10);

  q.setRPY(0.0, 0.0, 3*M_PI);
  PoseEstimateFilter::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,   0.0, 1e-10);
  ASSERT_NEAR(pitch,  0.0, 1e-10);
  ASSERT_NEAR(yaw,   M_PI, 1e-10);

  q.setRPY(0.0, 0.0, -3*M_PI);
  PoseEstimateFilter::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,   0.0, 1e-10);
  ASSERT_NEAR(pitch,  0.0, 1e-10);
  ASSERT_NEAR(yaw,   -M_PI, 1e-10);
}

TEST(PoseEstimateFilterTests, getTheta)
{
  tf2::Quaternion q;
  geometry_msgs::Quaternion orientation;

  q.setRPY(0.0, 0.0, M_PI);
  orientation = tf2::toMsg(q);
  ASSERT_NEAR(PoseEstimateFilter::getTheta(orientation), M_PI, 1e-10);

  q.setRPY(0.0, 0.0, -M_PI);
  orientation = tf2::toMsg(q);
  ASSERT_NEAR(PoseEstimateFilter::getTheta(orientation), -M_PI, 1e-10);

  q.setRPY(0.0, 0.0, 0.0);
  orientation = tf2::toMsg(q);
  ASSERT_NEAR(PoseEstimateFilter::getTheta(orientation), 0.0, 1e-10);

  q.setRPY(0.0, 0.0, 2*M_PI);
  orientation = tf2::toMsg(q);
  ASSERT_NEAR(PoseEstimateFilter::getTheta(orientation), 0.0, 1e-10);
}

TEST(PoseEstimateFilterTests, getAverageOrientation)
{
  using geometry_msgs::PoseStamped;
  std::list<PoseStamped> poses;
  tf2::Quaternion q;
  geometry_msgs::Quaternion orientation;
  PoseStamped pose;

  q.setRPY(0.0, 0.0, -1.0); pose.pose.orientation = tf2::toMsg(q); poses.push_back(pose);
  q.setRPY(0.0, 0.0, -2.0); pose.pose.orientation = tf2::toMsg(q); poses.push_back(pose);
  q.setRPY(0.0, 0.0,  2.0); pose.pose.orientation = tf2::toMsg(q); poses.push_back(pose);
  q.setRPY(0.0, 0.0,  1.0); pose.pose.orientation = tf2::toMsg(q); poses.push_back(pose);
  orientation = PoseEstimateFilter::getAverageOrientation(poses);
  ASSERT_NEAR(PoseEstimateFilter::getTheta(orientation), 0.0, 1e-10);

  poses.clear();
  q.setRPY(0.0, 0.0, M_PI_2); pose.pose.orientation = tf2::toMsg(q); poses.push_back(pose);
  q.setRPY(0.0, 0.0,    0.0); pose.pose.orientation = tf2::toMsg(q); poses.push_back(pose);
  orientation = PoseEstimateFilter::getAverageOrientation(poses);
  ASSERT_NEAR(PoseEstimateFilter::getTheta(orientation), M_PI_4, 1e-10);

  poses.clear();
  q.setRPY(0.0, 0.0,   M_PI); pose.pose.orientation = tf2::toMsg(q); poses.push_back(pose);
  q.setRPY(0.0, 0.0, M_PI_2); pose.pose.orientation = tf2::toMsg(q); poses.push_back(pose);
  orientation = PoseEstimateFilter::getAverageOrientation(poses);
  ASSERT_NEAR(PoseEstimateFilter::getTheta(orientation), 3*M_PI_4, 1e-10);

  poses.clear();
  q.setRPY(0.0, 0.0,    M_PI); pose.pose.orientation = tf2::toMsg(q); poses.push_back(pose);
  q.setRPY(0.0, 0.0, -M_PI_2); pose.pose.orientation = tf2::toMsg(q); poses.push_back(pose);
  orientation = PoseEstimateFilter::getAverageOrientation(poses);
  ASSERT_NEAR(PoseEstimateFilter::getTheta(orientation), -3*M_PI_4, 1e-10);

  poses.clear();
  q.setRPY(0.0, 0.0, -M_PI_2); pose.pose.orientation = tf2::toMsg(q); poses.push_back(pose);
  q.setRPY(0.0, 0.0,     0.0); pose.pose.orientation = tf2::toMsg(q); poses.push_back(pose);
  orientation = PoseEstimateFilter::getAverageOrientation(poses);
  ASSERT_NEAR(PoseEstimateFilter::getTheta(orientation), -M_PI_4, 1e-10);
}

TEST(PoseEstimateFilterTests, getAveragePosition)
{
  using geometry_msgs::PoseStamped;
  std::list<PoseStamped> poses;
  tf2::Quaternion q;
  geometry_msgs::Point position;
  PoseStamped pose;
  position.z = 0.0;

  position.x =  1.0; position.y = -2.0; pose.pose.position = position; poses.push_back(pose);
  position.x = -1.0; position.y = -1.0; pose.pose.position = position; poses.push_back(pose);
  position.x =  2.0; position.y =  1.0; pose.pose.position = position; poses.push_back(pose);
  position.x = -2.0; position.y =  2.0; pose.pose.position = position; poses.push_back(pose);
  ASSERT_NEAR(PoseEstimateFilter::getAveragePosition(poses).x, 0.0, 1e-10);
  ASSERT_NEAR(PoseEstimateFilter::getAveragePosition(poses).y, 0.0, 1e-10);
  ASSERT_NEAR(PoseEstimateFilter::getAveragePosition(poses).z, 0.0, 1e-10);

  poses.clear();
  position.x =  1.0; position.y =  1.0; pose.pose.position = position; poses.push_back(pose);
  position.x =  2.0; position.y =  2.0; pose.pose.position = position; poses.push_back(pose);
  ASSERT_NEAR(PoseEstimateFilter::getAveragePosition(poses).x, 1.5, 1e-10);
  ASSERT_NEAR(PoseEstimateFilter::getAveragePosition(poses).y, 1.5, 1e-10);

  poses.clear();
  position.x =   1.0; position.y =  1.0; pose.pose.position = position; poses.push_back(pose);
  position.x =  -2.0; position.y =  2.0; pose.pose.position = position; poses.push_back(pose);
  ASSERT_NEAR(PoseEstimateFilter::getAveragePosition(poses).x, -0.5, 1e-10);
  ASSERT_NEAR(PoseEstimateFilter::getAveragePosition(poses).y,  1.5, 1e-10);
}

TEST(PoseEstimateFilterTests, flushOldPoses)
{
  using geometry_msgs::PoseStamped;
  ros::Time current_time = ros::Time(100);
  PoseStamped pose;
  std::list<PoseStamped> poses;

  pose.header.seq = 1; pose.header.stamp = ros::Time(99); poses.emplace_back(pose);
  pose.header.seq = 2; pose.header.stamp = ros::Time(90); poses.emplace_back(pose);
  PoseEstimateFilter::flushOldPoses(&poses, current_time, ros::Duration(10));
  ASSERT_EQ(2, poses.size());

  poses.clear();
  pose.header.seq = 1; pose.header.stamp = ros::Time(99); poses.emplace_back(pose);
  pose.header.seq = 2; pose.header.stamp = ros::Time(89); poses.emplace_back(pose);
  PoseEstimateFilter::flushOldPoses(&poses, current_time, ros::Duration(10));
  ASSERT_EQ(1, poses.size());
}

TEST(PoseEstimateFilterTests, addPoseEstimate)
{
  using geometry_msgs::PoseStamped;
  ros::Time current_time = ros::Time(100);
  PoseStamped pose;
  std::list<PoseStamped> poses;
  PoseEstimateFilter test(2, 10);

  test.addPoseEstimate(pose, current_time);
  test.addPoseEstimate(pose, current_time);
  test.addPoseEstimate(pose, current_time);
  poses = test.getPoses();
  ASSERT_EQ(2, poses.size());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}