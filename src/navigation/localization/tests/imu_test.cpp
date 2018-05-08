#include <apriltag_tracker_interface/apriltag_tracker_interface.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_interface/teleop_interface.h"
#include "super_localizer/super_localizer.h"
#include "wheel_params/wheel_params.h"
#include "lp_research/lpresearchimu.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

geometry_msgs::TransformStamped create_tf(double x, double y, tf2::Quaternion imu_orientation)
{
  geometry_msgs::TransformStamped tfStamp;
  tfStamp.header.stamp = ros::Time::now();
  tfStamp.header.frame_id = "map";
  tfStamp.child_frame_id = "base_link";
  tfStamp.transform.translation.x = x;
  tfStamp.transform.translation.y = y;
  tfStamp.transform.translation.z = 0.0;
  tfStamp.transform.rotation = tf2::toMsg(imu_orientation);

  return tfStamp;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "localization_tester");
  ros::NodeHandle n;
  ros::Rate r(10);
  tf2_ros::TransformBroadcaster br;

  // Needs to be after the static transform is found
  LpResearchImu *lpResearchImu = new LpResearchImu("imu_base_link");

  while (ros::ok())
  {
    if (lpResearchImu->receiveData() == ReadableSensors::ReadStatus::READ_SUCCESS)
    {
      br.sendTransform(create_tf(2, 0,lpResearchImu->getOrientation()));
    }
    else
    {
      ROS_WARN("Getting invalid data");
    }
    r.sleep();
    ros::spinOnce();
  }
}
