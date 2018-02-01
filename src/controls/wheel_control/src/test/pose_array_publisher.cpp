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

  ros::Rate rate(.05);

  geometry_msgs::Pose2D wp1; wp1.x = 2; wp1.y = 0; wp1.theta = 0;
  geometry_msgs::Pose2D wp2; wp2.x = 4; wp2.y = .25; wp2.theta = 0;

  waypoints.push_back(wp1);
  waypoints.push_back(wp2);

  wpmsg.pose_array = waypoints; 

  double xcounter =5;

  while (ros::ok())
  {
     wpmsg.pose_array = waypoints; 
     ROS_INFO("Publishing waypoints:\nX: %f\nY: %f\n, Th: %f\n\nX: %f\nY: %f\n, Th: %f",
               wp1.x, wp1.y, wp1.theta, wp2.x, wp2.y, wp2.theta);
     pub.publish(wpmsg);

     wp1.x = xcounter;
     wp1.y = -1;
     wp1.theta = 0;
     xcounter += 1;
     wp2.x = xcounter;
     wp2.y = 1;
     wp2.theta = 0;
     xcounter += 1;

     waypoints.clear();
	 waypoints.push_back(wp1);
	 waypoints.push_back(wp2);

     ros::spinOnce();
     rate.sleep(); 
  }
}