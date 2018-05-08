#include "lp_research/lpresearchimu.h"

LpResearchImu::LpResearchImu(std::string topic) : nh_(), quaternion(0,0,0,1)
{
  this->sub = this->nh_.subscribe(topic, 100, &LpResearchImu::imu_callback, this);
  x_acc = 0.0f;
  y_acc = 0.0f;
  omega = 0.0f;
  is_data_valid = false;
}

double LpResearchImu::getX(void)
{
  return x_acc;
}

double LpResearchImu::getY(void)
{
  return y_acc;
}

double LpResearchImu::getOmega(void)
{
  return omega;
}

ReadableSensors::ReadStatus LpResearchImu::receiveData()
{
  if (is_data_valid)
  {
    return ReadableSensors::ReadStatus::READ_SUCCESS;
  }
  else
  {
    return ReadableSensors::ReadStatus::READ_FAILED;
  }
}

void LpResearchImu::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  x_acc = msg->linear_acceleration.x;
  y_acc = msg->linear_acceleration.y;
  omega = msg->angular_velocity.z;
  quaternion.setW(msg->orientation.w);
  quaternion.setX(msg->orientation.x);
  quaternion.setY(msg->orientation.y);
  quaternion.setZ(msg->orientation.z);
  is_data_valid = true;
}

tf2::Quaternion LpResearchImu::getOrientation()
{
  return quaternion;
}
