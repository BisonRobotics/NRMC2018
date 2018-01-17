#include "apriltag_tracker_interface/apriltag_tracker_interface.h"
#include "ros/ros.h"

int main (int argc, char **argv){
  ros::init (argc, argv, "imu_listener");
  AprilTagTrackerInterface ap;
  ros::NodeHandle n;
  while (ros::ok ()){
      if (ap.receiveData() == ReadableSensors::ReadStatus::READ_SUCCESS) {
          ROS_INFO ("%f %f %f ", ap.getX(), ap.getY(), ap.getTheta());
      } else {
          ROS_INFO ("No Data received");
      }
    ros::spinOnce ();
  }
}
