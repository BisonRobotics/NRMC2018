#include <ros/ros.h>
#include <imperio/GlobalWaypoints.h>
#include <geometry_msgs/Pose2D.h>

#include <random>
#define _USE_MATH_DEFINES
#include <cmath>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_filter");
  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<imperio::GlobalWaypoints>("global_planner_goal", 200);

  std::vector<geometry_msgs::Pose2D> waypoints;
  imperio::GlobalWaypoints wpmsg;

  ros::Rate rate(.05);

  geometry_msgs::Pose2D wp1;
  wp1.x = 1.5;
  wp1.y = 0;
  wp1.theta = 0;

  waypoints.push_back(wp1);

  std::random_device rd;
  std::mt19937 mt(rd());

  //std::uniform_real_distribution<double> dist(-M_PI, M_PI);
  std::normal_distribution<double> dist{M_PI_4, .2};

  double randomTheta;
  double waypointDist = .5;

  for (int num = 0; num < 5; num++)
  {
    randomTheta = dist(mt);
    wp1.theta += randomTheta;

    if (wp1.theta > M_PI) wp1.theta -=2.0 * M_PI;
    else if (wp1.theta < -M_PI) wp1.theta += 2.0 * M_PI;

    wp1.x += waypointDist * cos(wp1.theta);
    wp1.y += waypointDist * sin(wp1.theta);
    waypoints.push_back(wp1);
  }

  ros::spinOnce();
  while (ros::ok())
  {
    wpmsg.pose_array = waypoints;
    ros::spinOnce();
    pub.publish(wpmsg);

    waypoints.clear();

    for (int num = 0; num < 5; num++)
    {
      randomTheta = dist(mt);
      wp1.theta += randomTheta;

      if (wp1.theta > M_PI) wp1.theta -= 2.0 * M_PI;
      else if (wp1.theta < -M_PI) wp1.theta += 2.0 * M_PI;

      wp1.x += waypointDist * cos(wp1.theta);
      wp1.y += waypointDist * sin(wp1.theta);
      waypoints.push_back(wp1);
    }

    ros::spinOnce();
    rate.sleep();
  }
}
