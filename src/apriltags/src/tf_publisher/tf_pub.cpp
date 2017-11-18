#include <ros/ros.h>
#include <apriltags/AprilTagDetections.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>



void detectionCallback(const apriltags::AprilTagDetectionsConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  if (msg->detections.size()){
  tf::Quaternion q (msg->detections[0].pose.orientation.x, msg->detections[0].pose.orientation.y, msg->detections[0].pose.orientation.z, msg->detections[0].pose.orientation.w);
  transform.setOrigin(tf::Vector3(msg->detections[0].pose.position.x, msg->detections[0].pose.position.y, msg->detections[0].pose.position.z));
  transform.setRotation(q);
  br.sendTransform (tf::StampedTransform(transform, ros::Time::now(), "/world", "/base_link"));
  }
}



int main (int argc, char ** argv){
  ros::init (argc, argv, "my_tf_broadcaster");
  ROS_INFO ("INITIALIZED");
  ros::NodeHandle node;
  ROS_INFO ("NODE HANDLE");
  ros::Subscriber sub = node.subscribe ("/apriltags/detections", 100, &detectionCallback);

  ros::spin ();
  return 0;



}

