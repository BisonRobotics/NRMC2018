#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static tf2_ros::TransformBroadcaster* br;

void transformsCallback(const geometry_msgs::TransformStamped::ConstPtr& transform)
{
  geometry_msgs::TransformStamped synced_transform = *transform;
  synced_transform.header.stamp = ros::Time::now();  // Don't use this technique for anything other than visuals
  br->sendTransform(synced_transform);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transforms_publisher");
  ros::NodeHandle nh;
  br = new tf2_ros::TransformBroadcaster();
  ros::Subscriber sub = nh.subscribe("/position_sensor/transforms", 1000, transformsCallback);

  tf2::Quaternion q;

  q.setRPY(-M_PI_2, 0, -M_PI_2);
  geometry_msgs::TransformStamped servo_joint_to_optical_link_tf;
  servo_joint_to_optical_link_tf.header.seq = 0;
  servo_joint_to_optical_link_tf.header.frame_id = "servo_joint";
  servo_joint_to_optical_link_tf.child_frame_id = "camera_optical";
  servo_joint_to_optical_link_tf.transform.translation.x = 9.0e-3;
  servo_joint_to_optical_link_tf.transform.translation.y = 0.0;
  servo_joint_to_optical_link_tf.transform.translation.z = 25.0e-3;
  servo_joint_to_optical_link_tf.transform.rotation.x = q.getX();
  servo_joint_to_optical_link_tf.transform.rotation.y = q.getY();
  servo_joint_to_optical_link_tf.transform.rotation.z = q.getZ();
  servo_joint_to_optical_link_tf.transform.rotation.w = q.getW();

  q.setRPY(0, 0, 0);
  geometry_msgs::TransformStamped camera_to_base_link;
  camera_to_base_link.header.seq = 0;
  camera_to_base_link.header.frame_id = "base_link";
  camera_to_base_link.child_frame_id = "camera_link";
  camera_to_base_link.transform.translation.x = .4191;
  camera_to_base_link.transform.translation.y = 0.0;
  camera_to_base_link.transform.translation.z = 25.0e-3;
  camera_to_base_link.transform.rotation.x = q.getX();
  camera_to_base_link.transform.rotation.y = q.getY();
  camera_to_base_link.transform.rotation.z = q.getZ();
  camera_to_base_link.transform.rotation.w = q.getW();

  q.setRPY(M_PI_2, 0.0, M_PI_2);
  geometry_msgs::TransformStamped map_to_tag04_tf;
  map_to_tag04_tf.header.seq = 0;
  map_to_tag04_tf.header.frame_id = "map";
  map_to_tag04_tf.child_frame_id = "tag4";
  map_to_tag04_tf.transform.translation.x = 0.0;
  map_to_tag04_tf.transform.translation.y = 0.0;
  map_to_tag04_tf.transform.translation.z = 0.25;
  map_to_tag04_tf.transform.rotation.x = q.getX();
  map_to_tag04_tf.transform.rotation.y = q.getY();
  map_to_tag04_tf.transform.rotation.z = q.getZ();
  map_to_tag04_tf.transform.rotation.w = q.getW();

  q.setRPY(M_PI_2, 0.0, M_PI_2);
  geometry_msgs::TransformStamped map_to_tag03_tf;
  map_to_tag03_tf.header.seq = 0;
  map_to_tag03_tf.header.frame_id = "map";
  map_to_tag03_tf.child_frame_id = "tag3";
  map_to_tag03_tf.transform.translation.x = 0.0;
  map_to_tag03_tf.transform.translation.y = 0.5;
  map_to_tag03_tf.transform.translation.z = 0.25;
  map_to_tag03_tf.transform.rotation.x = q.getX();
  map_to_tag03_tf.transform.rotation.y = q.getY();
  map_to_tag03_tf.transform.rotation.z = q.getZ();
  map_to_tag03_tf.transform.rotation.w = q.getW();

  q.setRPY(M_PI_2, 0.0, M_PI_2);
  geometry_msgs::TransformStamped map_to_tag01_tf;
  map_to_tag01_tf.header.seq = 0;
  map_to_tag01_tf.header.frame_id = "map";
  map_to_tag01_tf.child_frame_id = "tag1";
  map_to_tag01_tf.transform.translation.x = 0.0;
  map_to_tag01_tf.transform.translation.y = -0.5;
  map_to_tag01_tf.transform.translation.z = 0.25;
  map_to_tag01_tf.transform.rotation.x = q.getX();
  map_to_tag01_tf.transform.rotation.y = q.getY();
  map_to_tag01_tf.transform.rotation.z = q.getZ();
  map_to_tag01_tf.transform.rotation.w = q.getW();

  q.setRPY(0.0, 0.0, M_PI);
  geometry_msgs::TransformStamped base_link_to_servo_base_link_tf;
  base_link_to_servo_base_link_tf.header.seq = 0;
  base_link_to_servo_base_link_tf.header.frame_id = "base_link";
  base_link_to_servo_base_link_tf.child_frame_id = "servo_base_link";
  base_link_to_servo_base_link_tf.transform.translation.x = -0.4191;
  base_link_to_servo_base_link_tf.transform.translation.y = 0.0;
  base_link_to_servo_base_link_tf.transform.translation.z = 0.0;
  base_link_to_servo_base_link_tf.transform.rotation.x = q.getX();
  base_link_to_servo_base_link_tf.transform.rotation.y = q.getY();
  base_link_to_servo_base_link_tf.transform.rotation.z = q.getZ();
  base_link_to_servo_base_link_tf.transform.rotation.w = q.getW();

  ros::Rate rate(2);
  while (ros::ok())
  {
    servo_joint_to_optical_link_tf.header.stamp = ros::Time::now();
    // servo_base_link_to_servo_joint_tf.header.stamp = ros::Time::now();
    base_link_to_servo_base_link_tf.header.stamp = ros::Time::now();
    map_to_tag04_tf.header.stamp = ros::Time::now();
    map_to_tag03_tf.header.stamp = ros::Time::now();
    map_to_tag01_tf.header.stamp = ros::Time::now();
    camera_to_base_link.header.stamp = ros::Time::now();

    servo_joint_to_optical_link_tf.header.seq++;
    // servo_base_link_to_servo_joint_tf.header.seq++;
    base_link_to_servo_base_link_tf.header.seq++;
    map_to_tag04_tf.header.seq++;
    map_to_tag03_tf.header.seq++;
    map_to_tag01_tf.header.seq++;
    camera_to_base_link.header.seq++;

    br->sendTransform(servo_joint_to_optical_link_tf);
    // br.sendTransform(servo_base_link_to_servo_joint_tf);
    br->sendTransform(map_to_tag04_tf);
    br->sendTransform(map_to_tag03_tf);
    br->sendTransform(map_to_tag01_tf);
    br->sendTransform(base_link_to_servo_base_link_tf);
    br->sendTransform(camera_to_base_link);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};
