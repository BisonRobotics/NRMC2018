//
// Created by marcintrosh on 2/13/18.
//

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_node");
  tf::TransformListener tf(ros::Duration(10));
  costmap_2d::Costmap2DROS lcr("costmap", tf);
  lcr.start ();
  ros::spin();

  return (0);
}