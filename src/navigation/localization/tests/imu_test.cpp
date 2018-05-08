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
#include "tf2/LinearMath/Quaternion.h"


geometry_msgs::TransformStamped create_tf(double x, double y, tf2::Quaternion my_quat)
{
  geometry_msgs::TransformStamped tfStamp;
  tfStamp.header.stamp = ros::Time::now();
  tfStamp.header.frame_id = "map";
  tfStamp.child_frame_id = "base_link";
  tfStamp.transform.translation.x = x;
  tfStamp.transform.translation.y = y;
  tfStamp.transform.translation.z = 0.0;
  tf2::Quaternion q;
  double raw,paw,yaw;
  tf2::Matrix3x3(tf2::Quaternion(my_quat.getX(), my_quat.getY(), my_quat.getZ(), my_quat.getW())).getRPY(raw,paw,yaw);
  q.setRPY(raw, paw, yaw);
  ROS_INFO ("R %.4f, P %.4f Y %.4f", raw, paw, yaw);
  tfStamp.transform.rotation.x = q.x();
  tfStamp.transform.rotation.y = q.y();
  tfStamp.transform.rotation.z = q.z();
  tfStamp.transform.rotation.w = q.w();
  return tfStamp;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "localization_tester");
  LpResearchImu *lpResearchImu = new LpResearchImu("imu_base_link");
  ros::NodeHandle n;
  ros::Rate r(10);
  tf2_ros::TransformBroadcaster br;

  while (ros::ok())
  {
    br.sendTransform(create_tf(2, 0,lpResearchImu->getOrientation()));
    r.sleep();
    ros::spinOnce();
  }
}
