#include "lp_research/lpresearchimu.h"
#include "ros/ros.h"

int main (int argc, char **argv){
 ros::init (argc, argv, "imu_listener");
   LpResearchImu *lp = new LpResearchImu ("imu");
  ros::NodeHandle n;
  while (ros::ok ()){
    ROS_INFO ("%f %f %f ", lp->getX(), lp->getY(), lp->getAlpha() );
    ros::spinOnce ();
  }
}
