#include <apriltag_tracker_interface/apriltag_tracker_interface.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2/convert.h"
#include "std_msgs/Bool.h"

AprilTagTrackerInterface::AprilTagTrackerInterface(std::string topic, double timeout) : nh_ ()
{
  this->sub = this->nh_.subscribe(topic, 100, &AprilTagTrackerInterface::callback, this);
  this->pub = this->nh_.advertise<std_msgs::Bool>("is_floating", 100);
  x = 0.0;
  y = 0.0;
  theta = 0.0;
  is_floating = false;
  this->timeout = ros::Duration (timeout);
}

AprilTagTrackerInterface::~AprilTagTrackerInterface()
{
}

double AprilTagTrackerInterface::getX()
{
  return x;
}

double AprilTagTrackerInterface::getY()
{
  return y;
}

double AprilTagTrackerInterface::getTheta()
{
  return theta;
}

void AprilTagTrackerInterface::updateIsFloating (){
  is_floating = (ros::Time::now () - last_time) > timeout;
  std_msgs::Bool msg;
  msg.data = (unsigned char) is_floating;
  pub.publish(msg);

}


ReadableSensors::ReadStatus AprilTagTrackerInterface::receiveData()
{

  updateIsFloating();

  if (is_floating){
    return ReadableSensors::ReadStatus::READ_FAILED;
  } else {
    return ReadableSensors::ReadStatus::READ_FAILED;
  }
}



// TODO: implement this, should return true if the tag is currently not in sight. Maybe through a ROS message?

void AprilTagTrackerInterface::callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  this->x = msg->pose.position.x;
  this->y = msg->pose.position.y;
  this->theta = qtToTheta(msg->pose.orientation);
  last_time = ros::Time::now ();
}

bool AprilTagTrackerInterface::isFloating()
{
  return is_floating;
}


double AprilTagTrackerInterface::qtToTheta(geometry_msgs::Quaternion  qt)
{
  tf2::Matrix3x3 m;
  m.setRotation(tf2::Quaternion(qt.x,qt.y, qt.z, qt.w));
  double tmp1, tmp2, theta;
  m.getRPY (tmp1, tmp2, theta);
  return theta;
}