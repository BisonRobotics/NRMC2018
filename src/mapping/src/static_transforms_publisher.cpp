#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "static_transforms_publisher");
  ros::NodeHandle nh;
  static tf2_ros::TransformBroadcaster br;

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

  /*q.setRPY(0.0, 0.0, 0.0);
  geometry_msgs::TransformStamped servo_base_link_to_servo_joint_tf;
  servo_base_link_to_servo_joint_tf.header.seq = 0;
  servo_base_link_to_servo_joint_tf.header.frame_id = "servo_base_link";
  servo_base_link_to_servo_joint_tf.child_frame_id = "servo_joint";
  servo_base_link_to_servo_joint_tf.transform.translation.x = 24.15e-3;
  servo_base_link_to_servo_joint_tf.transform.translation.y = 0.0;
  servo_base_link_to_servo_joint_tf.transform.translation.z = 32.5e-3;
  servo_base_link_to_servo_joint_tf.transform.rotation.x = q.getX();
  servo_base_link_to_servo_joint_tf.transform.rotation.y = q.getY();
  servo_base_link_to_servo_joint_tf.transform.rotation.z = q.getZ();
  servo_base_link_to_servo_joint_tf.transform.rotation.w = q.getW();*/
  q.setRPY(-M_PI_2, 0, -M_PI_2);
  geometry_msgs::TransformStamped camera_to_map;
  camera_to_map.header.seq = 0;
  camera_to_map.header.frame_id = "map";
  camera_to_map.child_frame_id = "camera_depth_optical_frame";
  camera_to_map.transform.translation.x = 9.0e-3;
  camera_to_map.transform.translation.y = 0.0;
  camera_to_map.transform.translation.z = 25.0e-3;
  camera_to_map.transform.rotation.x = q.getX();
  camera_to_map.transform.rotation.y = q.getY();
  camera_to_map.transform.rotation.z = q.getZ();
  camera_to_map.transform.rotation.w = q.getW();


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
  map_to_tag03_tf.transform.translation.y = 0.0;
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
  map_to_tag01_tf.transform.translation.y = 0.0;
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

  ros::Rate rate(60);
  while(ros::ok())
  {
    servo_joint_to_optical_link_tf.header.stamp = ros::Time::now();
    //servo_base_link_to_servo_joint_tf.header.stamp = ros::Time::now();
    base_link_to_servo_base_link_tf.header.stamp = ros::Time::now();
    map_to_tag04_tf.header.stamp = ros::Time::now();
    map_to_tag03_tf.header.stamp = ros::Time::now();
    map_to_tag01_tf.header.stamp = ros::Time::now();
    camera_to_map.header.stamp = ros::Time::now ();

    servo_joint_to_optical_link_tf.header.seq++;
    //servo_base_link_to_servo_joint_tf.header.seq++;
    base_link_to_servo_base_link_tf.header.seq++;
    map_to_tag04_tf.header.seq++;
    map_to_tag03_tf.header.seq++;
    map_to_tag01_tf.header.seq++;
    camera_to_map.header.seq++;

    br.sendTransform(servo_joint_to_optical_link_tf);
    //br.sendTransform(servo_base_link_to_servo_joint_tf);
    br.sendTransform(map_to_tag04_tf);
    br.sendTransform(map_to_tag03_tf);
    br.sendTransform(map_to_tag01_tf);
    br.sendTransform(base_link_to_servo_base_link_tf);
    br.sendTransform(camera_to_map);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};
