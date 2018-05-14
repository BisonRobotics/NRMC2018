#include <ros/ros.h>
#include <imperio/GlobalWaypoints.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>


#define MIN_DISTANCE_FOR_RUN 2.5
#define OBSTACLE_ZONE_START_X 1.5
#define OBSTACLE_ZONE_END_X 4.44
#define MIN_WAYPOINT_DISTANCE .8

#include <super_waypoint_filter/super_waypoint_filter.h>

std::vector<geometry_msgs::Pose2D> waypoints;
bool one_true_path_recieved = false;
bool go = false;


void newGoalArrayCallback(const imperio::GlobalWaypoints::ConstPtr& msg)
{
  if (!one_true_path_recieved)
  {
    waypoints = msg->pose_array;
    one_true_path_recieved = true;
  }
  go = true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "super_waypoint_filter");
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
    bool direction = false; //true is forward
    bool path_been_filtered = false;
    SuperWaypointFilter smfw;
    
    std::vector<geometry_msgs::Pose2D> waypoints_to_publish;
    std::vector<geometry_msgs::Pose2D> waypoints_from_autonomy;

    while(ros::ok())
    {
      if (go)
      {
          go = false;
          direction = !direction;
          if (!path_been_filtered)
          {
              ROS_WARN("GOING TO FILTER PATH");
              if (smfw.filterWaypoints(waypoints))
              {
                path_been_filtered = true;
                ROS_WARN("WAYPOINTS FILTERED");
              }
              else
              {
                  ROS_ERROR("PATH COULD NOT BE FILTERED");
              }
          }
          if (direction)
          {
              waypoints_to_publish = smfw.getForwardPath();
              ROS_WARN("GETTING FORWARD PATH, SIZE: %d", (int)waypoints_to_publish.size());
          }
          else
          {
              waypoints_to_publish = smfw.getBackwardPath();
              ROS_WARN("GETTING BACKWARD PATH, SIZE: %d", (int)waypoints_to_publish.size());
          }
          waypoints_from_autonomy = smfw.getRawPath();
          goal_markers.points.clear();
          for (auto const& wp : waypoints_from_autonomy)
          {
            vis_point.x = wp.x;
            vis_point.y = wp.y;
            vis_point.z = .1;
            goal_markers.points.push_back(vis_point);
          }
          goals_from_autonomy.publish(goal_markers);
          for (auto const& wp : waypoints_to_publish)
          {
              pub.publish(wp);
              ros::spinOnce();
              rate.sleep();
          }
      }
      else
      {
        ros::spinOnce();
        rate.sleep();
      }
    }
}