#include <ros/ros.h>
#include <imperio/GlobalWaypoints.h>
#include <geometry_msgs/Pose2D.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_filter");
  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<imperio::GlobalWaypoints> ("global_planner_goal",200);

  std::vector<geometry_msgs::Pose2D> waypoints;
  imperio::GlobalWaypoints wpmsg;

  ros::Rate rate(.01);

  geometry_msgs::Pose2D wp1; wp1.x = 1; wp1.y = 0; wp1.theta = 0;
  geometry_msgs::Pose2D wp2; wp2.x = 2; wp2.y = .25; wp2.theta = 0;

  waypoints.push_back(wp1);
  waypoints.push_back(wp2);

  wpmsg.pose_array = waypoints;

  while (ros::ok())
  {
     pub.publish(wpmsg);  
     ros::spinOnce();
  }
}