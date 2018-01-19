// this is the waypoint master, right now it:
// gets the current robot position from the tf from map to base_link
// gets waypoints from /global_planner_goal

// soon it will
// update status at /drive_controller_status
// take input from the costmap about where the path should not go

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <waypoint_controller/waypoint_controller.h>
#include <waypoint_controller/waypointWithManeuvers.h>

#include <geometry_msgs/Pose2D.h>

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
#include <wheel_params/wheel_params.h>
#include <vector>
#include <utility>

bool newWaypointHere = false;
pose newWaypoint;

void newGoalCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  // should add the waypoint to the queue of waypoints
  // first it shall just print it out
  newWaypoint.x = msg->x;
  newWaypoint.y = msg->y;
  newWaypoint.theta = msg->theta;
  newWaypointHere = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("global_planner_goal", 1, newGoalCallback);

  ros::Time curr;
  ros::Time last;
  ros::Duration delta;

  tf::StampedTransform transform;
  tf::TransformListener listener;

  pose theWay = {.x = 2.3f, .y = 0.0f, .theta = -1.42f };
  pose currPose;
  pose theCPP;

  std::vector<waypointWithManeuvers> navigationQueue;
  char *can_name = (char *)WHEEL_CAN_NETWORK;
  VescAccess fl(FRONT_LEFT_WHEEL_ID, WHEEL_GEAR_RATIO, WHEEL_OUTPUT_RATIO, MAX_WHEEL_VELOCITY, MAX_WHEEL_TORQUE,
                WHEEL_TORQUE_CONSTANT, can_name, 1);
  VescAccess fr(FRONT_RIGHT_WHEEL_ID, WHEEL_GEAR_RATIO, -1.0f * WHEEL_OUTPUT_RATIO, MAX_WHEEL_VELOCITY,
                MAX_WHEEL_TORQUE, WHEEL_TORQUE_CONSTANT, can_name, 1);
  VescAccess br(BACK_RIGHT_WHEEL_ID, WHEEL_GEAR_RATIO, -1.0f * WHEEL_OUTPUT_RATIO, MAX_WHEEL_VELOCITY, MAX_WHEEL_TORQUE,
                WHEEL_TORQUE_CONSTANT, can_name, 1);
  VescAccess bl(BACK_LEFT_WHEEL_ID, WHEEL_GEAR_RATIO, WHEEL_OUTPUT_RATIO, MAX_WHEEL_VELOCITY, MAX_WHEEL_TORQUE,
                WHEEL_TORQUE_CONSTANT, can_name, 1);

  listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(30));
  listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

  currPose.x = -1.0f * transform.getOrigin().x();
  currPose.y = -1.0f * transform.getOrigin().y();
  currPose.theta = -transform.getRotation().getAngle();

  WaypointController wc = WaypointController(.5f, 1.0f, currPose, &fl, &fr, &br, &bl);
  WaypointController::Status wcStat;

  ros::Rate rate(50.0);
  while (node.ok())
  {
    try  // get position
    {
      listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10));
      listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
      currPose.x = -1.0f * transform.getOrigin().x();
      currPose.y = -1.0f * transform.getOrigin().y();
      currPose.theta = -transform.getRotation().getAngle();
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    if (newWaypointHere)
    {
      wc.addWaypoint(newWaypoint, currPose);
    }

    // update controller
    ROS_INFO("GOING TO UPDATE");
    curr = ros::Time::now();
    delta = curr - last;
    wcStat = wc.update(currPose, (float)delta.toSec());  // need to get actual time
    last = curr;

    // print status
    if (wcStat == WaypointController::Status::ALLBAD)
      ROS_INFO("CONTROLLER SAYS BAD");
    else if (wcStat == WaypointController::Status::ALLGOOD)
      ROS_INFO("CONTROLLER SAYS GOOD");
    else if (wcStat == WaypointController::Status::GOALREACHED)
    {
      ROS_INFO("GOOOOOAAAAALLLLL!!");
      // if (isRunning)
      // {
      //   isRunning = false;
      //   ros::shutdown();
      // }
    }

    // print some info
    navigationQueue = wc.getNavigationQueue();
    theCPP = wc.getCPP();

    ROS_INFO("\nCPP x: %.4f \nCPP y: %.4f\nCPP th:%.4f", theCPP.x, theCPP.y, theCPP.theta);

    ROS_INFO("\nCurP x: %.4f \nCurP y: %.4f\nCurP th:%.4f", currPose.x, currPose.y, currPose.theta);

    ROS_INFO("Etp Estimate: %.4f", wc.getETpEstimate());
    ROS_INFO("Epp Estimate: %.4f", wc.getEPpEstimate());

    ROS_INFO("SetSpeeds: %.4f, %.4f", wc.getSetSpeeds().first, wc.getSetSpeeds().second);

    if (navigationQueue.size() > 0)
    {
      ROS_INFO("Nav mans Size: %d", (int)navigationQueue.at(0).mans.size());
      for (int k = wc.getCurrManeuverIndex(); k < navigationQueue.at(0).mans.size(); k++)
        ROS_INFO("Nav Man %d Param:\nradius: %.4f\nxc: %.4f\nyc: %.4f\ndistance: %.4f", k,
                 navigationQueue.at(0).mans.at(k).radius, navigationQueue.at(0).mans.at(k).xc,
                 navigationQueue.at(0).mans.at(k).yc, navigationQueue.at(0).mans.at(k).distance);
      pose manEnd = wc.getManeuverEnd();
      ROS_INFO("CurrManEnd:\nx: %.4f\ny: %.4f\nth: %.4f", manEnd.x, manEnd.y, manEnd.theta);
      ROS_INFO("Nav initial Pose:\nx: %.4f\ny: %.4f\nth: %.4f", navigationQueue.at(0).initialPose.x,
               navigationQueue.at(0).initialPose.y, navigationQueue.at(0).initialPose.theta);
      ROS_INFO("Nav terminal Pose:\nx: %.4f\ny: %.4f\nth: %.4f", navigationQueue.at(0).terminalPose.x,
               navigationQueue.at(0).terminalPose.y, navigationQueue.at(0).terminalPose.theta);
    }

    // ros end stuff
    ros::spinOnce();
    rate.sleep();
  }
}
