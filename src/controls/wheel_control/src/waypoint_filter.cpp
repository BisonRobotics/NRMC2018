#include <ros/ros.h>
#include <imperio/GlobalWaypoints.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>


#include <waypoint_controller/waypoint_controller_helper.h>

std::vector<geometry_msgs::Pose2D> waypoints;

void newGoalArrayCallback(const imperio::GlobalWaypoints::ConstPtr& msg)
{
  // msg->pose_array is a vector
  waypoints = msg->pose_array;
}

double interpolateYFromXAndTwoPoints(double x0, double y0, double x1, double y1, double x)
{
    return ((y1 - y0)/(x1 - x0)) * (x - x0) + y0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_filter");
  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("position_controller/global_planner_goal", 200, newGoalArrayCallback);
  ros::Publisher pub = node.advertise<geometry_msgs::Pose2D>("position_controller/additional_waypoint", 200);

  geometry_msgs::Pose2D wpmsg;
  
  ros::Publisher goals_from_autonomy = node.advertise<visualization_msgs::Marker>("autonomy_goals", 10000);
  
  visualization_msgs::Marker goal_markers;
  goal_markers.action = visualization_msgs::Marker::ADD;
  goal_markers.pose.orientation.w = 1;
  goal_markers.type = visualization_msgs::Marker::LINE_STRIP;
  goal_markers.scale.x = .1;
  goal_markers.scale.y = .1;
  goal_markers.scale.z = .1;
  goal_markers.color.b = 1;
  goal_markers.color.g = 1;
  goal_markers.color.a = 1;
  goal_markers.header.frame_id = "/map";
  
  geometry_msgs::Point vis_point;


  ros::Rate rate(10.0);

  while (ros::ok())
  {
    if (waypoints.size() > 0)
    {
      double direction_metric = 0;
      double prev_x;
      bool overwrite_dump = false;
      bool overwrite_dig = false;
            
      bool grabbing_entrance = false;
      bool grabbing_exit = false;
      bool post_once = false;
      
      goal_markers.points.clear();
      geometry_msgs::Pose2D last_point_in_start_zone;
      geometry_msgs::Pose2D last_point_in_obstacle_zone;
      geometry_msgs::Pose2D last_point_in_dig_zone;
      geometry_msgs::Pose2D obstacle_zone_exit_point;
      
      //computer direction metric
      for (int index =0; index < waypoints.size(); index++)
      {
          if (index == 0)
          {
              prev_x = waypoints.at(index).x;
          }
          else
          {
              direction_metric += waypoints.at(index).x - prev_x;
          }
      }

      for (auto const& wp : waypoints)
      {
        vis_point.x = wp.x;
        vis_point.y = wp.y;
        vis_point.z = .1;
        goal_markers.points.push_back(vis_point);
        
        if (direction_metric > 2.5) //make sure there is a waypoint in the start zone (by placing one)
        {
            // also make sure there is a waypoint in the dig zone and maybe through an exception if there is not.
        }
        
        if (direction_metric < -2.5) //make sure there is a waypoint in the starting area (which is the dig zone, by placing one)
        {
            // also make sure there is a waypoint in the start/dump zone and maybe through an exception if there is not.
        }
        
        if (direction_metric > 2.5) // need to grab waypoints on entering and exiting waypoints from obstacle zone.
        {
            if (wp.x < 1.5) //this waypoint is in the start zone, as we iterate it will be the closest one to the obstacle zone
            {
                //copy waypoint for posting
                wpmsg.x = wp.x;
                wpmsg.y = wp.y;
                wpmsg.theta = wp.theta;
                //record last point
                last_point_in_start_zone.x = wp.x;
                last_point_in_start_zone.y = wp.y;
                last_point_in_start_zone.theta = wp.theta;
                grabbing_entrance = true;
            }
            else if ((grabbing_entrance || grabbing_exit) && wp.x <4.44 ) // there is a(n) waypoint(s) in the obstacle field
            {
                grabbing_entrance = false;
                grabbing_exit = true;
                //record last point in obstacle field
                last_point_in_obstacle_zone.x = wp.x;
                last_point_in_obstacle_zone.y = wp.y;
                last_point_in_obstacle_zone.theta = wp.theta;
                //keep recording points to remember last one
                //copy waypoint for posting
                wpmsg.x = wp.x;
                wpmsg.y = wp.y;
                wpmsg.theta = wp.theta;
            }
            else if (grabbing_entrance) // there is not a waypoint in the obstacle field
            {
                grabbing_entrance = false;
                //calculate exit from here, now, between last_point_in_start_zone and this first point                
                //it will be used in overwrite_dig logic
                obstacle_zone_exit_point.y = interpolateYFromXAndTwoPoints(last_point_in_start_zone.x, last_point_in_start_zone.y, wp.x, wp.y, 4.44);
                obstacle_zone_exit_point.x = 4.44;
                obstacle_zone_exit_point.theta = wp.theta;
                overwrite_dig = true;
            }
            else if (grabbing_exit) // waypoint in dig_zone after waypoints in obstacle zone
            {
                grabbing_exit = false;
                //calculate exit from here, now, between last_point_in_dig_zone and this first point                
                //it will be used in overwrite_dig logic
                obstacle_zone_exit_point.y = interpolateYFromXAndTwoPoints(last_point_in_obstacle_zone.x, last_point_in_obstacle_zone.y, wp.x, wp.y, 4.44);
                obstacle_zone_exit_point.x = 4.44;
                obstacle_zone_exit_point.theta = wp.theta;
                overwrite_dig = true;
            }
            // if there is no waypoint beyond 4.44, then the dig will not be overwritten because there is no dig
        }
        else if (direction_metric < -2.5) //need to overwrite dump
        {
            if (wp.x > 4.44) //this waypoint is in the starting zone (the dig zone now), as we iterate it will be the closest one to the obstacle zone
            {
                //copy waypoint for posting
                wpmsg.x = wp.x;
                wpmsg.y = wp.y;
                wpmsg.theta = wp.theta;
                //record last point
                last_point_in_dig_zone.x = wp.x;
                last_point_in_dig_zone.y = wp.y;
                last_point_in_dig_zone.theta = wp.theta;
                grabbing_entrance = true;
            }
            else if ((grabbing_entrance || grabbing_exit) && wp.x >1.5 ) // there is a waypoint in the obstacle field
            {
                grabbing_entrance = false;
                grabbing_exit = true;
                //record last point in obstacle field
                last_point_in_obstacle_zone.x = wp.x;
                last_point_in_obstacle_zone.y = wp.y;
                last_point_in_obstacle_zone.theta = wp.theta;
                //copy waypoint for posting
                wpmsg.x = wp.x;
                wpmsg.y = wp.y;
                wpmsg.theta = wp.theta;
            }
            else if (grabbing_entrance) // there is not a waypoint in the obstacle field
            {
                grabbing_entrance = false;
                //calculate exit from here, now, between last_point_in_dig_zone (the starting zone) and this first point                
                //it will be used in overwrite_dig logic
                obstacle_zone_exit_point.y = interpolateYFromXAndTwoPoints(last_point_in_dig_zone.x, last_point_in_dig_zone.y, wp.x, wp.y, 1.5);
                obstacle_zone_exit_point.x = 1.5;
                obstacle_zone_exit_point.theta = wp.theta;
                overwrite_dump = true;
            }
            else if (grabbing_exit) // waypoint in dig_zone after waypoints in obstacle zone
            {
                grabbing_exit = false;
                //calculate exit from here, now, between last_point_in_obstacle_zone and this first point                
                //it will be used in overwrite_dig logic
                obstacle_zone_exit_point.x = interpolateYFromXAndTwoPoints(last_point_in_obstacle_zone.x, last_point_in_obstacle_zone.y, wp.x, wp.y, 1.5);
                obstacle_zone_exit_point.x = 1.5;
                obstacle_zone_exit_point.theta = wp.theta;
                overwrite_dump = true;
            }
        }
        else // just need to copy waypoints
        {
          wpmsg.x = wp.x;
          wpmsg.y = wp.y;
          wpmsg.theta = wp.theta;
        }
      
      if (!overwrite_dump && !overwrite_dig)
      {
        pub.publish(wpmsg);
      }
        
      if (overwrite_dump && !post_once)
      {
          post_once = true;
          pub.publish(obstacle_zone_exit_point);
          ros::spinOnce();
          rate.sleep();
          
          wpmsg.x = 1.2;
          wpmsg.y = -0.025;
          wpmsg.theta =0;
          pub.publish(wpmsg);
          ros::spinOnce();
          rate.sleep();
          
          wpmsg.x = .6;
          wpmsg.y = 0.025;
          wpmsg.theta =0;
          pub.publish(wpmsg);
          ros::spinOnce();
          rate.sleep();
      }
      
      if (overwrite_dig && !post_once)
      {
          post_once = true;
          pub.publish(obstacle_zone_exit_point);
          ros::spinOnce();
          rate.sleep();
          wpmsg.x = 5.25;
          wpmsg.y = -0.025;
          wpmsg.theta =0;
          pub.publish(wpmsg);
          ros::spinOnce();
          rate.sleep();
          
          wpmsg.x = 6.25;
          wpmsg.y = 0.025;
          wpmsg.theta =0;
          pub.publish(wpmsg);
          ros::spinOnce();
          rate.sleep();
      }
      
      goals_from_autonomy.publish(goal_markers);
      ros::spinOnce();
      rate.sleep();
      
      waypoints.clear();
    }
    }
    else
    {
      ros::spinOnce();
      rate.sleep();
    }
  }
}
