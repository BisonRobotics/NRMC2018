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

  q.setRPY(M_PI_2, 0.0, M_PI_2);
  geometry_msgs::TransformStamped map_to_tag00_tf;
  map_to_tag00_tf.header.seq = 0;
  map_to_tag00_tf.header.frame_id = "map";
  map_to_tag00_tf.child_frame_id = "tag0";
  map_to_tag00_tf.transform.translation.x = 0.0;
  map_to_tag00_tf.transform.translation.y = 0.525;
  map_to_tag00_tf.transform.translation.z = 0.25;
  map_to_tag00_tf.transform.rotation.x = q.getX();
  map_to_tag00_tf.transform.rotation.y = q.getY();
  map_to_tag00_tf.transform.rotation.z = q.getZ();
  map_to_tag00_tf.transform.rotation.w = q.getW();

  q.setRPY(M_PI_2, 0.0, M_PI_2);
  geometry_msgs::TransformStamped map_to_tag01_tf;
  map_to_tag01_tf.header.seq = 0;
  map_to_tag01_tf.header.frame_id = "map";
  map_to_tag01_tf.child_frame_id = "tag1";
  map_to_tag01_tf.transform.translation.x = 0.0;
  map_to_tag01_tf.transform.translation.y = -0.525;
  map_to_tag01_tf.transform.translation.z = 0.25;
  map_to_tag01_tf.transform.rotation.x = q.getX();
  map_to_tag01_tf.transform.rotation.y = q.getY();
  map_to_tag01_tf.transform.rotation.z = q.getZ();
  map_to_tag01_tf.transform.rotation.w = q.getW();

  ros::Rate rate(2);
  while (ros::ok())
  {
    map_to_tag00_tf.header.stamp = ros::Time::now();
    map_to_tag01_tf.header.stamp = ros::Time::now();

    map_to_tag00_tf.header.seq++;
    map_to_tag01_tf.header.seq++;

    br->sendTransform(map_to_tag00_tf);
    br->sendTransform(map_to_tag01_tf);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};
