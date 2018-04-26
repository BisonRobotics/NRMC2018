#include <pose_estimate_filter/pose_estimate_filter.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace apriltag_tracker;

bool publish_tf;

ros::Publisher *pose_pub;
PoseEstimateFilter *filter;
static tf2_ros::TransformBroadcaster* br;

void callback(const geometry_msgs::PoseStampedConstPtr &pose)
{
  filter->addPoseEstimate(*pose.get());
  try
  {
    geometry_msgs::PoseStamped pose = filter->getMovingAverageTransform();

    pose_pub->publish(pose);

    // Only use for testing
    if (publish_tf)
    {
      tf2::Stamped<tf2::Transform> transform;
      tf2::fromMsg(pose, transform);
      transform.stamp_ = ros::Time::now();
      geometry_msgs::TransformStamped message = tf2::toMsg(transform);
      message.header.frame_id = "map";
      message.child_frame_id = "base_link";
      br->sendTransform(message);
    }
  }
  catch (empty_list_error &e)
  {
    ROS_WARN("%s", e.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_estimate_filter");
  ros::NodeHandle nh("~");
  pose_pub = new ros::Publisher;
  *pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_estimate", 100);
  br = new tf2_ros::TransformBroadcaster();

  filter = new PoseEstimateFilter(15, 1.0);

  nh.param<bool>("publish_tf", publish_tf, false);
  if (publish_tf)
  {
    ROS_WARN("Transforms will be published directly to tf from incoming pose estimates");
  }
  else
  {
    ROS_INFO("Transform publishing disabled");
  }

  ros::Subscriber node0 = nh.subscribe("/node0/pose_estimate", 100, callback);
  ros::Subscriber node1 = nh.subscribe("/node1/pose_estimate", 100, callback);

  ros::spin();
}