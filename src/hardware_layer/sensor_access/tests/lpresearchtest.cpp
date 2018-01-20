#include "lp_research/lpresearchimu.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_listener");
  LpResearchImu *lp = new LpResearchImu("imu");
  ros::NodeHandle n;
  while (ros::ok())
  {
    if (lp->receiveData() == ReadableSensors::ReadStatus::READ_SUCCESS)
    {
      ROS_INFO("%f %f %f ", lp->getX(), lp->getY(), lp->getOmega());
    }
    else
    {
      ROS_INFO("No Data received");
    }
    ros::spinOnce();
  }
}
