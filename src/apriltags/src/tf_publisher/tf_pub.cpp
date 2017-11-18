#include <ros/ros.h>
#include <apriltags/AprilTagDetections.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>



void detectionCallback(const apriltags::AprilTagDetectionsConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  for (int ctr = 0; ctr < msg->detections.size(); ctr++){
    if (msg->detections[ctr].id == 3){ 
      tf::Quaternion q (msg->detections[ctr].pose.orientation.x, msg->detections[ctr].pose.orientation.y, msg->detections[ctr].pose.orientation.z, msg->detections[ctr].pose.orientation.w);
      transform.setOrigin(tf::Vector3(msg->detections[ctr].pose.position.x, msg->detections[ctr].pose.position.y, msg->detections[ctr].pose.position.z));
      transform.setRotation(q);
      br.sendTransform (tf::StampedTransform(transform, ros::Time::now(), "/camera", "/base_link"));
    } else if (msg->detections[ctr].id == 1){
      tf::Quaternion q (msg->detections[ctr].pose.orientation.x, msg->detections[ctr].pose.orientation.y, msg->detections[ctr].pose.orientation.z, msg->detections[ctr].pose.orientation.w);
      transform.setOrigin(tf::Vector3(msg->detections[ctr].pose.position.x, msg->detections[ctr].pose.position.y, msg->detections[ctr].pose.position.z));
      transform.setRotation(q);
      br.sendTransform (tf::StampedTransform(transform, ros::Time::now(), "/camera", "/map"));
    }
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

