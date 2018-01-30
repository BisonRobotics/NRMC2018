#include <ros/ros.h>
#include <imperio/GlobalWaypoints.h>
#include <geometry_msgs/Pose2D.h>

#include <waypoint_controller/waypoint_controller_helper.h>

std::vector<geometry_msgs::Pose2D> waypoints;

void newGoalArrayCallback(const imperio::GlobalWaypoints::ConstPtr &msg)
{
  //msg->pose_array is a vector
  waypoints = msg->pose_array;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_filter");
  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("global_planner_goal", 200, newGoalArrayCallback);
  ros::Publisher pub = node.advertise<geometry_msgs::Pose2D> ("additional_waypoint",200);

  geometry_msgs::Pose2D wpmsg;

  
  ros::Rate rate(5.0);

  while (ros::ok())
  {

     if (waypoints.size() >0)
     {
          for (auto const& wp: waypoints)
          {
               wpmsg.x = wp.x;
               wpmsg.y = wp.y;
               wpmsg.theta = wp.theta;

               pub.publish(wpmsg);
          }
          waypoints.clear();
     }
     
     ros::spinOnce();
     rate.sleep();
  }
}