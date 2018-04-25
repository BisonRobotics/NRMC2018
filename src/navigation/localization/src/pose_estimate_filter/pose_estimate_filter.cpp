#include <pose_estimate_filter/pose_estimate_filter.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>


using geometry_msgs::PoseStamped;
using geometry_msgs::Point;
using geometry_msgs::Quaternion;
using namespace apriltag_tracker;


PoseEstimateFilter::PoseEstimateFilter(int list_size, double max_dt)
{
  this->seq = 0;
  this->list_size = list_size;
  this->max_dt = ros::Duration(max_dt);
  this->mutex = new boost::mutex;
}

void PoseEstimateFilter::addPoseEstimate(PoseStamped pose)
{
  addPoseEstimate(pose, ros::Time::now());
}

void PoseEstimateFilter::addPoseEstimate(PoseStamped pose, ros::Time current_time)
{
  mutex->lock();
  pose.header.stamp = current_time; // Not great, but we care more about arrival time anyways
  poses.emplace_front(pose);
  while (poses.size() > list_size)
  {
    poses.pop_back();
  }
  mutex->unlock();
}

void PoseEstimateFilter::flushOldPoses(std::list<PoseStamped> *poses, ros::Time current_time)
{
  flushOldPoses(poses, current_time, this->max_dt);
}

void PoseEstimateFilter::flushOldPoses(std::list<PoseStamped> *poses, ros::Time current_time,
                                       ros::Duration max_dt)
{
  auto it = poses->begin();
  int good_poses = 0;
  bool clean = false;
  for (int i = 0; i < poses->size(); i++, it++)
  {
    ros::Duration time_diff(current_time - it->header.stamp);
    if (time_diff > max_dt)
    {
      clean = true;
      good_poses = i;
      break;
    }
  }
  if (clean)
  {
    for (int i = 0; (i + good_poses) < poses->size(); i++)
    {
      poses->pop_back();
    }
  }
}

void PoseEstimateFilter::getRPY(tf2::Quaternion q, double &roll, double &pitch, double &yaw)
{
  tf2::Matrix3x3 matrix;
  matrix.setRotation(q);
  matrix.getRPY(roll, pitch, yaw);
}

double PoseEstimateFilter::getTheta(geometry_msgs::Quaternion orientation)
{
  double roll, pitch, yaw;
  tf2::Quaternion q;
  tf2::fromMsg(orientation, q);
  getRPY(q, roll, pitch, yaw);
  return yaw;
}

// Assumes that the thetas are close enough not to negate each other
Quaternion PoseEstimateFilter::getAverageOrientation(std::list<PoseStamped> &poses)
{
  double x = 0.0, y = 0.0;
  for (auto it = poses.begin(); it != poses.end(); it++)
  {
    Quaternion orientation = it->pose.orientation;
    x += cos(getTheta(orientation));
    y += sin(getTheta(orientation));
  }
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, atan2(y,x));
  return tf2::toMsg(q);
}

Point PoseEstimateFilter::getAveragePosition(std::list<PoseStamped> &poses)
{
  double x = 0.0, y = 0.0, z = 0.0;
  for (auto it = poses.begin(); it != poses.end(); it++)
  {
    Point position = it->pose.position;
    x += position.x;
    y += position.y;
    z += position.z;
  }
  double size = poses.size();
  Point position;
  position.x = x / size;
  position.y = y / size;
  position.z = z / size;
  return position;
}

PoseStamped PoseEstimateFilter::getMovingAverageTransform()
{
  return getMovingAverageTransform(ros::Time::now());
}

PoseStamped PoseEstimateFilter::getMovingAverageTransform(ros::Time current_time)
{
  mutex->lock();
  flushOldPoses(&poses, current_time);
  if (poses.size() < 1)
  {
    throw empty_list_error("Not enough poses to perform transform");
  }
  PoseStamped pose;
  pose.header.stamp = current_time;
  pose.header.frame_id = "map";
  pose.header.seq = this->seq++;
  pose.pose.orientation = getAverageOrientation(poses);
  pose.pose.position = getAveragePosition(poses);
  mutex->unlock();
  return pose;
}

std::list<geometry_msgs::PoseStamped> PoseEstimateFilter::getPoses()
{
  return poses;
}


