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
float velocity_left = 0.0f;
float velocity_right = 0.0f;

void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
  if (joy->buttons[4])
  {
    velocity_left = joy->axes[1];
    velocity_right = joy->axes[4];
  }
  else
  {
    velocity_left = 0.0f;
    velocity_right = 0.0f;
  }
}

geometry_msgs::TransformStamped create_tf(double x, double y, double theta, tf2::Quaternion my_quat, double z)
{
  geometry_msgs::TransformStamped tfStamp;
  tfStamp.header.stamp = ros::Time::now();
  tfStamp.header.frame_id = "map";
  tfStamp.child_frame_id = "base_link";
  tfStamp.transform.translation.x = x;
  tfStamp.transform.translation.y = y;
  tfStamp.transform.translation.z = z;
  tf2::Quaternion q;
  double raw,paw,yaw;
  tf2::Matrix3x3(tf2::Quaternion(my_quat.getX(), my_quat.getY(), my_quat.getZ(), my_quat.getW())).getRPY(raw,paw,yaw);
  q.setRPY(raw, paw, theta);
  tfStamp.transform.rotation.x = q.x();
  tfStamp.transform.rotation.y = q.y();
  tfStamp.transform.rotation.z = q.z();
  tfStamp.transform.rotation.w = q.w();
  return tfStamp;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localization_tester");
  AprilTagTrackerInterface *aprilTags = new AprilTagTrackerInterface("/pose_estimate_filter/pose_estimate", .1);
  LpResearchImu *lpResearchImu = new LpResearchImu("imu_base_link");
  ros::NodeHandle n;
  ros::Rate r(50);
  tf2_ros::TransformBroadcaster br;
  ros::Subscriber sub = n.subscribe("/joy", 30, callback);
  TeleopInterface teleopInterface(.1f);
  SuperLocalizer superLocalizer(ROBOT_AXLE_LENGTH, 0.0, 0.0, 0.0, teleopInterface.fl, teleopInterface.fr,
                                teleopInterface.br, teleopInterface.bl, lpResearchImu, aprilTags,
                                SuperLocalizer_default_gains);
  ros::Time last_time = ros::Time::now();
  LocalizerInterface::stateVector stateVector;

  while (ros::ok())
  {
    teleopInterface.update(velocity_left, velocity_right);
    superLocalizer.updateStateVector((ros::Time::now() - last_time).toSec());
    last_time = ros::Time::now();
    stateVector = superLocalizer.getStateVector();
    ROS_INFO("Position: %f %f %f Velocity %f %f %f Acceleration %f %f %f", stateVector.x_pos, stateVector.y_pos,
             stateVector.theta, stateVector.x_vel, stateVector.y_vel, stateVector.omega, stateVector.x_accel,
             stateVector.y_accel, stateVector.alpha);
    if (lpResearchImu->receiveData() == ReadableSensors::ReadStatus::READ_SUCCESS)
    {
      ROS_INFO("x: %f y: %f omega %f", lpResearchImu->getX(), lpResearchImu->getY(), lpResearchImu->getOmega());
    }
    else
    {
      ROS_INFO("IMU DATA BAD");
    }
    ROS_INFO("fl %f fr %f bl %f br %f", teleopInterface.fl->getLinearVelocity(),
             teleopInterface.fr->getLinearVelocity(), teleopInterface.bl->getLinearVelocity(),
             teleopInterface.br->getLinearVelocity());
    ROS_INFO("x: %f y %f theta %f", aprilTags->getX(), aprilTags->getY(), aprilTags->getTheta());
    br.sendTransform(create_tf(stateVector.x_pos, stateVector.y_pos, stateVector.theta, lpResearchImu->getOrientation(), aprilTags->getZ()));


    r.sleep();
    ros::spinOnce();
  }
}
