#ifndef __LP_RESEARCH_IMU__
#define __LP_RESEARCH_IMU__


#include "sensor_access/imu_sensor_interface.h"
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"

class LpResearchImu : public ImuSensorInterface{
  public:
  LpResearchImu (std::string topic_name);
  float getX();
  float getY();
  float getAlpha();
  ReadableSensors::ReadStatus receiveData();
  private:
  void imu_callback (const sensor_msgs::Imu::ConstPtr &msg);
  double x_acc;
  double y_acc;
  double alpha;
  bool is_data_valid;
  ros::Subscriber sub;
  ros::NodeHandle nh_;

};




#endif
