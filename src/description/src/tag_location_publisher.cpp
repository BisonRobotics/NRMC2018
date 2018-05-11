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
  ros::Subscriber sub0 = nh.subscribe("/node0/transforms", 1000, transformsCallback);
  ros::Subscriber sub1 = nh.subscribe("/node1/transforms", 1000, transformsCallback);

  tf2::Quaternion q1, q2, q3;

  q1.setRPY(M_PI_2, 0.0, M_PI_2);
  geometry_msgs::TransformStamped map_to_tag00_tf;
  map_to_tag00_tf.header.seq = 0;
  map_to_tag00_tf.header.frame_id = "map";
  map_to_tag00_tf.child_frame_id = "tag0";
  map_to_tag00_tf.transform.translation.x = 0.0;
  map_to_tag00_tf.transform.translation.y = -0.5025;
  map_to_tag00_tf.transform.translation.z = 0.2563;
  map_to_tag00_tf.transform.rotation.x = q1.getX();
  map_to_tag00_tf.transform.rotation.y = q1.getY();
  map_to_tag00_tf.transform.rotation.z = q1.getZ();
  map_to_tag00_tf.transform.rotation.w = q1.getW();

  q1.setRPY(M_PI_2, 0.0, M_PI_2);
  geometry_msgs::TransformStamped map_to_tag01_tf;
  map_to_tag01_tf.header.seq = 0;
  map_to_tag01_tf.header.frame_id = "map";
  map_to_tag01_tf.child_frame_id = "tag1";
  map_to_tag01_tf.transform.translation.x = 0.0;
  map_to_tag01_tf.transform.translation.y = 0.5025;
  map_to_tag01_tf.transform.translation.z = 0.2563;
  map_to_tag01_tf.transform.rotation.x = q1.getX();
  map_to_tag01_tf.transform.rotation.y = q1.getY();
  map_to_tag01_tf.transform.rotation.z = q1.getZ();
  map_to_tag01_tf.transform.rotation.w = q1.getW();

  q1.setRPY(M_PI_2, 0.0, M_PI_2);
  q2.setRPY(0.0, -0.321751, 0.0);
  q3 = q1 * q2;
  geometry_msgs::TransformStamped map_to_tag03_tf;
  map_to_tag03_tf.header.seq = 0;
  map_to_tag03_tf.header.frame_id = "map";
  map_to_tag03_tf.child_frame_id = "tag3";
  map_to_tag03_tf.transform.translation.x = 0.02;
  map_to_tag03_tf.transform.translation.y = -0.76;
  map_to_tag03_tf.transform.translation.z = 0.5731;
  map_to_tag03_tf.transform.rotation.x = q3.getX();
  map_to_tag03_tf.transform.rotation.y = q3.getY();
  map_to_tag03_tf.transform.rotation.z = q3.getZ();
  map_to_tag03_tf.transform.rotation.w = q3.getW();

  q1.setRPY(M_PI_2, 0.0, M_PI_2);
  q2.setRPY(0.0, 0.321751, 0.0);
  q3 = q1 * q2;
  geometry_msgs::TransformStamped map_to_tag04_tf;
  map_to_tag04_tf.header.seq = 0;
  map_to_tag04_tf.header.frame_id = "map";
  map_to_tag04_tf.child_frame_id = "tag4";
  map_to_tag04_tf.transform.translation.x = 0.02;
  map_to_tag04_tf.transform.translation.y = -0.64;
  map_to_tag04_tf.transform.translation.z = 0.5731;
  map_to_tag04_tf.transform.rotation.x = q3.getX();
  map_to_tag04_tf.transform.rotation.y = q3.getY();
  map_to_tag04_tf.transform.rotation.z = q3.getZ();
  map_to_tag04_tf.transform.rotation.w = q3.getW();

  q1.setRPY(M_PI_2, 0.0, M_PI_2);
  q2.setRPY(0.0, -0.321751, 0.0);
  q3 = q1 * q2;
  geometry_msgs::TransformStamped map_to_tag05_tf;
  map_to_tag05_tf.header.seq = 0;
  map_to_tag05_tf.header.frame_id = "map";
  map_to_tag05_tf.child_frame_id = "tag5";
  map_to_tag05_tf.transform.translation.x = 0.02;
  map_to_tag05_tf.transform.translation.y = 0.64;
  map_to_tag05_tf.transform.translation.z = 0.5731;
  map_to_tag05_tf.transform.rotation.x = q3.getX();
  map_to_tag05_tf.transform.rotation.y = q3.getY();
  map_to_tag05_tf.transform.rotation.z = q3.getZ();
  map_to_tag05_tf.transform.rotation.w = q3.getW();

  q1.setRPY(M_PI_2, 0.0, M_PI_2);
  q2.setRPY(0.0, 0.321751, 0.0);
  q3 = q1 * q2;
  geometry_msgs::TransformStamped map_to_tag06_tf;
  map_to_tag06_tf.header.seq = 0;
  map_to_tag06_tf.header.frame_id = "map";
  map_to_tag06_tf.child_frame_id = "tag6";
  map_to_tag06_tf.transform.translation.x = 0.02;
  map_to_tag06_tf.transform.translation.y = 0.76;
  map_to_tag06_tf.transform.translation.z = 0.5731;
  map_to_tag06_tf.transform.rotation.x = q3.getX();
  map_to_tag06_tf.transform.rotation.y = q3.getY();
  map_to_tag06_tf.transform.rotation.z = q3.getZ();
  map_to_tag06_tf.transform.rotation.w = q3.getW();

  ros::Rate rate(2);
  while (ros::ok())
  {
    map_to_tag00_tf.header.stamp = ros::Time::now();
    map_to_tag01_tf.header.stamp = ros::Time::now();
    map_to_tag03_tf.header.stamp = ros::Time::now();
    map_to_tag04_tf.header.stamp = ros::Time::now();
    map_to_tag05_tf.header.stamp = ros::Time::now();
    map_to_tag06_tf.header.stamp = ros::Time::now();

    map_to_tag00_tf.header.seq++;
    map_to_tag01_tf.header.seq++;
    map_to_tag03_tf.header.seq++;
    map_to_tag04_tf.header.seq++;
    map_to_tag05_tf.header.seq++;
    map_to_tag06_tf.header.seq++;

    br->sendTransform(map_to_tag00_tf);
    br->sendTransform(map_to_tag01_tf);
    br->sendTransform(map_to_tag03_tf);
    br->sendTransform(map_to_tag04_tf);
    br->sendTransform(map_to_tag05_tf);
    br->sendTransform(map_to_tag06_tf);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};
