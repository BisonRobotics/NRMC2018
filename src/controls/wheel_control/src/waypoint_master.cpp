// this is the waypoint master, right now it:
// runs the superlocalizer and uses it to post the tf from map to base_link
// gets the current robot position from the tf from map to base_link
// gets waypoints from /global_planner_goal

// soon it will
// update status at /drive_controller_status

// after that it will (second sprint 2018 cycle)
// take a stream of incoming valid waypoints on the costmap and 
// pick the "best" ones to follow
// initial idea: start at the furthest waypoint and work backwords
// after iterating through the available waypoints, either choose the
// furthest valid one (could just pick the first one that works)

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <waypoint_controller/waypoint_controller.h>
#include <waypoint_controller/waypoint_with_maneuvers.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
#include <wheel_params/wheel_params.h>

#include <super_localizer/super_localizer.h>
#include <lp_research/lpresearchimu.h>
#include <apriltag_tracker_interface/apriltag_tracker_interface.h>

#include <vector>
#include <utility>

#define AXEL_LEN .5f
#define MAX_SPEED .5f

bool newWaypointHere = false;
pose newWaypoint;

void newGoalCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  // should add the waypoint to the queue of waypoints
  newWaypoint.x = msg->x;
  newWaypoint.y = msg->y;
  newWaypoint.theta = msg->theta;
  newWaypointHere = true;
}

geometry_msgs::TransformStamped create_tf(double x, double y, double theta)
{
  geometry_msgs::TransformStamped tfStamp;
  tfStamp.header.stamp = ros::Time::now();
  tfStamp.header.frame_id = "map";
  tfStamp.child_frame_id = "base_link";
  tfStamp.transform.translation.x = x;
  tfStamp.transform.translation.y = y;
  tfStamp.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  tfStamp.transform.rotation.x = q.x();
  tfStamp.transform.rotation.y = q.y();
  tfStamp.transform.rotation.z = q.z();
  tfStamp.transform.rotation.w = q.w();
  return tfStamp;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("global_planner_goal", 1, newGoalCallback);

  ros::Time curr;
  ros::Time last;
  ros::Duration delta;

  //tf::StampedTransform transform;
  //tf::TransformListener listener;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  pose theWay;
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

  //listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(30));
  //listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

  //initialize the localizer here
  ros::Time last_time = ros::Time::now();
  tf2_ros::TransformBroadcaster tfBroad;
  AprilTagTrackerInterface *aprilTags = new AprilTagTrackerInterface();
  LpResearchImu *lpResearchImu = new LpResearchImu("imu");
  SuperLocalizer superLocalizer(AXEL_LEN, 0,0,0, &fl, &fr, &br, &bl, lpResearchImu, aprilTags, SuperLocalizer_default_gains);
  LocalizerInterface::stateVector stateVector;

  //hang here until someone knows where we are

  bool hasFirstPose = false;
  while (!hasFirstPose)
  {
    //do initial localization
    superLocalizer.updateStateVector((ros::Time::now() - last_time).toSec());
    last_time = ros::Time::now();
    stateVector = superLocalizer.getStateVector();
    tfBroad.sendTransform(create_tf(stateVector.x_pos, stateVector.y_pos, stateVector.theta));

    //should figure out a way to wait until localization thinks its converged to the 
    //actual position

    try
    {
      ROS_INFO("Waiting for initial localization data");
      transformStamped = tfBuffer.lookupTransform("/map", "/base_link", ros::Time(0), ros::Duration(30));
      hasFirstPose = true;
    }
    catch (tf2::TransformException &ex)
    {
       ROS_WARN("%s",ex.what());
       hasFirstPose = false;
    }
  }
  //populate currPose from localization data
  currPose.x = transformStamped.transform.translation.x; //-1.0f * transform.getOrigin().x();
  currPose.y = transformStamped.transform.translation.y; //-1.0f * transform.getOrigin().y();
  tf2::Quaternion tempQuat(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y,
                      transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
  currPose.theta = tempQuat.getAngle() - M_PI;//-transform.getRotation().getAngle();

  //initialize waypoint controller
  WaypointController wc = WaypointController(AXEL_LEN, MAX_SPEED, currPose, &fl, &fr, &br, &bl);
  WaypointController::Status wcStat;

  ros::Rate rate(50.0);
  while (node.ok())
  {
    //update localizer here
    superLocalizer.updateStateVector((ros::Time::now() - last_time).toSec());
    last_time = ros::Time::now();
    stateVector = superLocalizer.getStateVector();
    tfBroad.sendTransform(create_tf(stateVector.x_pos, stateVector.y_pos, stateVector.theta));
    
    try  // get position
    {
      //listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10));
      //listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
      transformStamped = tfBuffer.lookupTransform("/map", "/base_link", ros::Time(0), ros::Duration(0));
      //populate currPose from localization data
      currPose.x = transformStamped.transform.translation.x; //-1.0f * transform.getOrigin().x();
      currPose.y = transformStamped.transform.translation.y; //-1.0f * transform.getOrigin().y();
      tf2::Quaternion tempQuat(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y,
                          transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
      currPose.theta = tempQuat.getAngle() - M_PI;//-transform.getRotation().getAngle();
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      //ros::Duration(1.0).sleep();
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
    wcStat = wc.update(currPose, (float)delta.toSec());
    last = curr;

    //check if we are stuck by comparing commanded velocity to actual
    //maybe integrate error with some decay

    // print status also post to topic /drive_controller_status
    if (wcStat == WaypointController::Status::ALLBAD)
      ROS_WARN("CONTROLLER SAYS BAD");
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
