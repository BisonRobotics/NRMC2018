#ifndef __LP_RESEARCH_IMU__
#define __LP_RESEARCH_IMU__

#include "sensor_access/imu_sensor_interface.h"
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"

#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class LpResearchImu : public ImuSensorInterface
{
public:
  LpResearchImu(std::string topic_name);
  double getX();
  double getY();
  double getOmega();
  ReadableSensors::ReadStatus receiveData();
  tf2::Quaternion getOrientation ();
private:
  void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
  void imu_raw_callback(const sensor_msgs::Imu::ConstPtr &msg);
  double x_acc;
  double y_acc;
  double omega;
  bool is_data_valid;
  bool received_static_orientation;
  ros::Subscriber sub1, sub2;
  ros::NodeHandle nh_;
  tf2::Quaternion orientation;
  tf2::Quaternion static_orientation;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener *tf_listener;
};

#endif
