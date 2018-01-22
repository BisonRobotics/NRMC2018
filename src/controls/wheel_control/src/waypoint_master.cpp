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
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
#include <wheel_params/wheel_params.h>

#include <super_localizer/super_localizer.h>
#include <lp_research/lpresearchimu.h>
#include <apriltag_tracker_interface/apriltag_tracker_interface.h>
#include <sensor_msgs/JointState.h>

#include <vector>
#include <utility>

bool newWaypointHere = false;
pose newWaypoint;
bool halt = false;
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


void haltCallback (const std_msgs::Empty::ConstPtr& msg){
  halt = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("global_planner_goal", 1, newGoalCallback);
  ros::Publisher jspub = node.advertise<sensor_msgs::JointState> ("wheel_joints", 500);
  sensor_msgs::JointState jsMessage;
  jsMessage.name.push_back("front_left");
  jsMessage.name.push_back("front_right");
  jsMessage.name.push_back("back_right");
  jsMessage.name.push_back("back_left");
  jsMessage.velocity.push_back(0.0);
  jsMessage.velocity.push_back(0.0);
  jsMessage.velocity.push_back(0.0);
  jsMessage.velocity.push_back(0.0);
  tf2_ros::Buffer tfBuffer;
 // tf2_ros::TransformListener tfListener(tfBuffer);
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

  //initialize the localizer here
  ros::Time last_time = ros::Time::now();
  tf2_ros::TransformBroadcaster tfBroad;
  AprilTagTrackerInterface *aprilTags = new AprilTagTrackerInterface();
  LpResearchImu *lpResearchImu = new LpResearchImu("imu");
  SuperLocalizer superLocalizer(ROBOT_AXLE_LENGTH, 0,0,0, &fl, &fr, &br, &bl, lpResearchImu, aprilTags, SuperLocalizer_default_gains);
  LocalizerInterface::stateVector stateVector;
  ros::Subscriber haltsub = node.subscribe ("halt", 100, haltCallback);
  ros::Publisher mode_pub = node.advertise<std_msgs::String>  ("drive_controller_status", 1000);
  //hang here until someone knows where we are
  ROS_INFO ("Going into wait loop for local");

  ros::Rate rate(50.0);

  while (!superLocalizer.getIsDataGood() && ros::ok () )
  {
    //do initial localization
    superLocalizer.updateStateVector((ros::Time::now() - last_time).toSec());
    last_time = ros::Time::now();
    stateVector = superLocalizer.getStateVector();
    tfBroad.sendTransform(create_tf(stateVector.x_pos, stateVector.y_pos, stateVector.theta));
    ros::spinOnce ();
    rate.sleep ();
  }
  //populate currPose from localization data
/*  currPose.x = transformStamped.transform.translation.x; //-1.0f * transform.getOrigin().x();
  currPose.y = transformStamped.transform.translation.y; //-1.0f * transform.getOrigin().y();
  tf2::Quaternion tempQuat(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y,
                      transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
  currPose.theta = tempQuat.getAngle() - M_PI;//-transform.getRotation().getAngle();
*/
  //initialize waypoint controller
  WaypointController wc = WaypointController(ROBOT_AXLE_LENGTH, ROBOT_MAX_SPEED, currPose, &fl, &fr, &br, &bl);
  WaypointController::Status wcStat;
  std_msgs::String msg;
  std::stringstream ss;
  ros::Duration looptime;
  ROS_INFO ("Entering MAIN LOOP");
  while (ros::ok()) {
    //update localizer here
    looptime = ros::Time::now() - last_time;
    superLocalizer.updateStateVector(looptime.toSec());

    stateVector = superLocalizer.getStateVector();
    tfBroad.sendTransform(create_tf(stateVector.x_pos, stateVector.y_pos, stateVector.theta));
    currPose.x = stateVector.x_pos;
    currPose.y = stateVector.y_pos;
    currPose.theta = stateVector.theta;
    jsMessage.velocity[0] = fl.getLinearVelocity();
    jsMessage.velocity[1] = fr.getLinearVelocity();
    jsMessage.velocity[2] = br.getLinearVelocity();
    jsMessage.velocity[3] = bl.getLinearVelocity();
    if (newWaypointHere) {
      wc.addWaypoint(newWaypoint, currPose);
    }

    // update controller
    ROS_INFO("GOING TO UPDATE");

    wcStat = wc.update(currPose, looptime.toSec());


    //check if we are stuck by comparing commanded velocity to actual
    //maybe integrate error with some decay

    // print status also post to topic /drive_controller_status
    if (wcStat == WaypointController::Status::ALLBAD){
      ROS_WARN("CONTROLLER SAYS BAD");
      ss << "Mode : Bad";
    }else if (wcStat == WaypointController::Status::ALLGOOD) {
      ROS_INFO("CONTROLLER SAYS GOOD");
      ss << "Mode: Good";
    }
    else if (wcStat == WaypointController::Status::GOALREACHED)
    {
      ROS_INFO("GOOOOOAAAAALLLLL!!");
      wc.haltAndAbort();
      ss << "Mode: Chillin";
    }
    msg.data = ss.str ();
    mode_pub.publish (msg);
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
    if (halt){
      wc.haltAndAbort();
      break;
    }
    // ros end stuff
    jspub.publish(jsMessage);
    ros::spinOnce();
    last_time = ros::Time::now ();
    rate.sleep();
  }
}
