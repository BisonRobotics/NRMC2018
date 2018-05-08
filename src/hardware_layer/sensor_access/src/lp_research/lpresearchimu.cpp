#include "lp_research/lpresearchimu.h"

LpResearchImu::LpResearchImu(std::string topic) : nh_(), orientation(0,0,0,1)
{
  this->sub1 = this->nh_.subscribe(topic, 100, &LpResearchImu::imu_callback, this);
  this->sub2 = this->nh_.subscribe("/imu", 100, &LpResearchImu::imu_raw_callback, this);
  x_acc = 0.0f;
  y_acc = 0.0f;
  omega = 0.0f;
  is_data_valid = false;
  received_static_orientation = false;
  tf_listener = new tf2_ros::TransformListener(tf_buffer);
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
  if (!received_static_orientation)
  {
    try
    {
      geometry_msgs::TransformStamped transformStamped;
      transformStamped = tf_buffer.lookupTransform("base_link", "imu", ros::Time(0));
      geometry_msgs::Quaternion qt = transformStamped.transform.rotation;
      tf2::fromMsg(transformStamped.transform.rotation, static_orientation);
      received_static_orientation = true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("[lpresearchimu.cpp | Transform not found] %s",ex.what());
    }
  }

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
  if (received_static_orientation)
  {

  }
}

void LpResearchImu::imu_raw_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  if (received_static_orientation)
  {
    x_acc = -msg->linear_acceleration.z;
    y_acc = msg->linear_acceleration.x;
    omega = -msg->angular_velocity.y;

    orientation.setW(msg->orientation.w);
    orientation.setX(msg->orientation.x);
    orientation.setY(msg->orientation.y);
    orientation.setZ(msg->orientation.z);

    // Transform the orientation to base_link and remove yaw offset
    double roll,pitch,yaw;
    orientation = orientation*static_orientation.inverse();
    tf2::Matrix3x3(orientation).getRPY(roll,pitch,yaw);
    orientation.setRPY(roll, pitch, 0.0);

    is_data_valid = true;
  }
}

tf2::Quaternion LpResearchImu::getOrientation()
{
  return orientation;
}
