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
      
      
      //TODO: interpolate y value at exit of obstacle zone from waypoint in dig zone closest to obstacle zone
      //      and do something similar for on the way back after a dig to make sure entrance and exit to obstacle zone
      //      is done how the RRT thought it should.
    
      goal_markers.points.clear();
      geometry_msgs::Pose2D entrance_point;
      geometry_msgs::Pose2D arrival_point;
      
      bool grabbing_entrance = false;

      for (auto const& wp : waypoints)
      {
        vis_point.x = wp.x;
        vis_point.y = wp.y;
        vis_point.z = .1;
        goal_markers.points.push_back(vis_point);
        
        if (direction_metric > 0) // need to grab waypoints on entering and exiting waypoints from obstacle zone.
        {
            if (wp.x < 1.5) //this waypoint is in the start zone, as we iterate it will be the closest one to the obstacle zone
            {
                //copy waypoint for posting
                grabbing_entrance = true;
            }
            else if (grabbing_entrance && wp.x <4.44) // there is a waypoint in the obstacle field
            {
                //post extra 
            }
            else if (grabbing_entrance) // there is not a waypoint in the obstacle field
            {
                
            }
            if (wp.x > 4.44) //need to ignore these and overwrite_dig
            {
                
            }
        }
        else if (direction_metric < 0) //need to overwrite dump
        {
            
        }
        else // just need to forward
        {
            
        }
          
        /*
        if (wp.x > 4.44 + 1.0) // entering/exiting the dig zone + some buffer
        {
          if (direction_metric > 0) //moving forward to do a dig
          {
            overwrite_dig = true;
          }
          else //moving backward or single waypoint or moving sideways
          {
            wpmsg.x = wp.x;
            wpmsg.y = wp.y;
            wpmsg.theta = wp.theta;
          }
        }
        else if (wp.x > 4.44) // inside the buffer zone before going to the dig area
        {
            wpmsg.x = 4.75;
            wpmsg.y = wp.y;
            if (wp.y < -.4)
            {
                wpmsg.theta = .7;
            }
            else if (wp.y > .4)
            {
                wpmsg.theta = -.7;
            }
            else 
            {
                wpmsg.theta = 0;
            }
        }
        else if (wp.x > 1.5) // less than 4.44 and greater than 1.5, obstacle zone
        {
          wpmsg.x = wp.x;
          wpmsg.y = wp.y;
          wpmsg.theta = wp.theta;
        }
        else // entering/exiting start/dumping area
        {
            if (direction_metric >= 0) //moving forward, or single waypoint or sideways for some reason
            {
               wpmsg.x = wp.x;
               wpmsg.y = wp.y;
               wpmsg.theta = wp.theta;
            }
            else
            {
                overwrite_dump = true;
            }
        }

        if (!overwrite_dump && !overwrite_dig)
        {
            pub.publish(wpmsg);
        }

        ros::spinOnce();
        rate.sleep();
      }
      */
      if (overwrite_dump)
      {
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
      
      if (overwrite_dig)
      {
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
      
      waypoints.clear();
    }
    else
    {
      ros::spinOnce();
      rate.sleep();
    }
  }
}
