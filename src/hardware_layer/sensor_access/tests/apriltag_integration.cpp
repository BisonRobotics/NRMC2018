#include "apriltag_tracker_interface/apriltag_tracker_interface.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_listener");
  AprilTagTrackerInterface ap ("pose_estimate", .01);
  ros::Rate loop_rate (50);
  ros::NodeHandle n;
  while (ros::ok())
  {
    if (ap.receiveData() == ReadableSensors::ReadStatus::READ_SUCCESS)
    {
      ROS_INFO("%f %f %f ", ap.getX(), ap.getY(), ap.getTheta());
    }
    else
    {
      ROS_INFO("No Data received");
    }
    loop_rate.sleep ();
    ros::spinOnce();
  }
}
