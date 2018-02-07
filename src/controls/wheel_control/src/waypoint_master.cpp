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

#define SIMULATING 0
// one for simulating, 0 for real deal

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <waypoint_controller/waypoint_controller.h>
#include <waypoint_controller/waypoint_with_maneuvers.h>

#include <geometry_msgs/Pose2D.h>
//#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
#include <wheel_params/wheel_params.h>

#include <super_localizer/super_localizer.h>
#include <sensor_msgs/JointState.h>

#include <visualization_msgs/Marker.h>

#ifndef SIMULATING
#error You must define a value (1/0) for SIMULATING
#endif

#if SIMULATING == 1
#include <sim_robot/sim_robot.h>
#else
#include <lp_research/lpresearchimu.h>
#include <apriltag_tracker_interface/apriltag_tracker_interface.h>
#endif

#include <vector>
#include <utility>

#define UPDATE_RATE_HZ 50.0

bool newWaypointHere = false;
pose newWaypoint;
bool halt = false;

void newGoalCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  // this callback will flag that there is a new waypoint if there are no future
  // or current maneuvers. If thera are future maneuvers then
  // this callback will quick check to see if the new waypoint will generate a
  // better manuever than the current planned maneuvers.
  // better is defined as X distance travelled per maneuver distance

  // when checking potential maneuvers against the costmap, if a
  // maneuver would violate the costmap, it is replaced with a caveman maneuver.
  // a caveman maneuver (turn in place, go fwd, turn in place) is guarenteed safe.

  // will need waypoint_controlelr_helper waypoint to maneuvers
  // get future goodness
  // dump future maneuvers
  // get terminal pose of current maneuver

  // if no future maneuvers, add this one
  // else
  // get terminal pose of current maneuver
  // waypoint to maneuvers of terminal pose and potential waypoint
  // get goodness of this maneuver
  // compare the goodnesses
  // either keep the current plan or dump it and switch to the new one

  pose potentialWaypoint;
  potentialWaypoint.x = msg->x;
  potentialWaypoint.y = msg->y;
  potentialWaypoint.theta = msg->theta;

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

#if SIMULATING == 1
geometry_msgs::TransformStamped create_sim_tf(double x, double y, double theta)
{
  geometry_msgs::TransformStamped tfStamp;
  tfStamp.header.stamp = ros::Time::now();
  tfStamp.header.frame_id = "map";
  tfStamp.child_frame_id = "sim_base_link";
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
#endif 



void haltCallback(const std_msgs::Empty::ConstPtr &msg)
{
  halt = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("additional_waypoint", 1, newGoalCallback);
  ros::Publisher jspub = node.advertise<sensor_msgs::JointState>("wheel_joints", 500);
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

  // pose theWay = {0,0,0};
  pose currPose = { 0, 0, 0 };
  pose theCPP = { 0, 0, 0 };

  std::vector<waypointWithManeuvers> navigationQueue;

#if SIMULATING == 1
  SimRobot sim(ROBOT_AXLE_LENGTH, .5, .5, 0);
  iVescAccess *fl = (sim.getFLVesc());
  iVescAccess *fr = (sim.getFRVesc());
  iVescAccess *br = (sim.getBRVesc());
  iVescAccess *bl = (sim.getBLVesc());

  ImuSensorInterface *imu = sim.getImu();
  PosSensorInterface *pos = sim.getPos();
#else
  char *can_name = (char *)WHEEL_CAN_NETWORK;
  iVescAccess *fl = new VescAccess(FRONT_LEFT_WHEEL_ID, WHEEL_GEAR_RATIO, WHEEL_OUTPUT_RATIO, MAX_WHEEL_VELOCITY,
                                   MAX_WHEEL_TORQUE, WHEEL_TORQUE_CONSTANT, can_name, 1);
  iVescAccess *fr = new VescAccess(FRONT_RIGHT_WHEEL_ID, WHEEL_GEAR_RATIO, -1.0f * WHEEL_OUTPUT_RATIO,
                                   MAX_WHEEL_VELOCITY, MAX_WHEEL_TORQUE, WHEEL_TORQUE_CONSTANT, can_name, 1);
  iVescAccess *br = new VescAccess(BACK_RIGHT_WHEEL_ID, WHEEL_GEAR_RATIO, -1.0f * WHEEL_OUTPUT_RATIO,
                                   MAX_WHEEL_VELOCITY, MAX_WHEEL_TORQUE, WHEEL_TORQUE_CONSTANT, can_name, 1);
  iVescAccess *bl = new VescAccess(BACK_LEFT_WHEEL_ID, WHEEL_GEAR_RATIO, WHEEL_OUTPUT_RATIO, MAX_WHEEL_VELOCITY,
                                   MAX_WHEEL_TORQUE, WHEEL_TORQUE_CONSTANT, can_name, 1);

  PosSensorInterface *pos = new AprilTagTrackerInterface();
  ImuSensorInterface *imu = new LpResearchImu("imu");
#endif

  // initialize the localizer here
  ros::Time lastTime;
  ros::Time currTime;
  ros::Duration loopTime;
  bool firstTime = true;
  tf2_ros::TransformBroadcaster tfBroad;
  std::vector<std::pair<double, double> > waypoint_set;
  SuperLocalizer superLocalizer(ROBOT_AXLE_LENGTH, 0, 0, 0, fl, fr, br, bl, imu, pos, SuperLocalizer_default_gains);

  LocalizerInterface::stateVector stateVector;
  ros::Subscriber haltsub = node.subscribe("halt", 100, haltCallback);
  ros::Publisher mode_pub = node.advertise<std_msgs::String>("drive_controller_status", 1000);
  ros::Publisher path_marker_pub = node.advertise<visualization_msgs::Marker>("waypoint_path", 10000);
  visualization_msgs::Marker line_strip;
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = .1;
  line_strip.color.g = 1;
  line_strip.color.a = 1;
  line_strip.header.frame_id = "/map";
  geometry_msgs::Point vis_point;
  // hang here until someone knows where we are
  ROS_INFO("Going into wait loop for localizer");

  ros::Rate rate(UPDATE_RATE_HZ);
  ros::Duration idealLoopTime(1.0 / UPDATE_RATE_HZ);

  while (!superLocalizer.getIsDataGood() && ros::ok())
  {
    // do initial localization
    if (firstTime)
    {
      firstTime = false;
      currTime = ros::Time::now();
      lastTime = currTime - idealLoopTime;
      loopTime = (currTime - lastTime);
    }
    else
    {
      lastTime = currTime;
      currTime = ros::Time::now();
      loopTime = (currTime - lastTime);
    }
#if SIMULATING == 1
    sim.update((loopTime).toSec());
#endif
    superLocalizer.updateStateVector(loopTime.toSec());
    stateVector = superLocalizer.getStateVector();
    tfBroad.sendTransform(create_tf(stateVector.x_pos, stateVector.y_pos, stateVector.theta));
    ros::spinOnce();
    rate.sleep();
  }
  // populate currPose from localization data
  /*  currPose.x = transformStamped.transform.translation.x; //-1.0f * transform.getOrigin().x();
    currPose.y = transformStamped.transform.translation.y; //-1.0f * transform.getOrigin().y();
    tf2::Quaternion tempQuat(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y,
                        transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
    currPose.theta = tempQuat.getAngle() - M_PI;//-transform.getRotation().getAngle();
  */
  // initialize waypoint controller
  WaypointController wc =
      WaypointController(ROBOT_AXLE_LENGTH, ROBOT_MAX_SPEED, currPose, fl, fr, br, bl, 1.0 / UPDATE_RATE_HZ,
                         waypoint_default_gains);
  WaypointController::Status wcStat;
  std_msgs::String msg;
  std::stringstream ss;
  geometry_msgs::PoseStamped poser;
  firstTime = true;
  while (ros::ok())
  {
      ROS_INFO ("\n");
      ROS_INFO ("Top");
    // update localizer here
    if (firstTime)
    {
      firstTime = false;
      currTime = ros::Time::now();
      lastTime = currTime - idealLoopTime;
      loopTime = currTime - lastTime;
    }
    else
    {
      lastTime = currTime;
      currTime = ros::Time::now();
      loopTime = currTime - lastTime;
    }
    ROS_INFO("Looptime : %.5f", loopTime.toSec());
#if SIMULATING == 1
    sim.update(loopTime.toSec());

    tfBroad.sendTransform(create_sim_tf(sim.getX(), sim.getY(), sim.getTheta()));
#endif

    superLocalizer.updateStateVector(loopTime.toSec());
    stateVector = superLocalizer.getStateVector();
    tfBroad.sendTransform(create_tf(stateVector.x_pos, stateVector.y_pos, stateVector.theta));

    currPose.x = stateVector.x_pos;
    currPose.y = stateVector.y_pos;
    currPose.theta = stateVector.theta;
    jsMessage.velocity[0] = fl->getLinearVelocity();
    jsMessage.velocity[1] = fr->getLinearVelocity();
    jsMessage.velocity[2] = br->getLinearVelocity();
    jsMessage.velocity[3] = bl->getLinearVelocity();

      ROS_INFO ("FrontLeftVel : %.4f", jsMessage.velocity[0] );
      ROS_INFO ("FrontRightVel : %.4f", jsMessage.velocity[1]);
      ROS_INFO ("BackRightVel : %.4f", jsMessage.velocity[2]);
      ROS_INFO ("BackLeftVel : %.4f", jsMessage.velocity[3]);
    if (newWaypointHere)
    {
      waypoint_set = wc.addWaypoint(newWaypoint, currPose);
      line_strip.points.clear();
      for (auto const& waypoint : waypoint_set)
      {
//	poser.header.stamp = ros::Time::now ();
//	poser.header.seq++;
//	poser.header.frame_id = "map";
//	poser.pose.position.x = waypoint.first;
//	poser.pose.position.y = waypoint.second;
//	poser.pose.position.z = 0.0f;
//	pose_pub.publish (poser);
        vis_point.x = waypoint.first;
        vis_point.y = waypoint.second;
        vis_point.z = .4;
        line_strip.points.push_back(vis_point);
      }
      path_marker_pub.publish(line_strip);
      newWaypointHere = false;
        ROS_INFO ("NewWaypoint : 1");
    } else {
        ROS_INFO ("NewWaypoint : 0");
    }

    // update controller


    wcStat = wc.update(currPose, loopTime.toSec());

    // TODO
    // check if we are stuck by comparing commanded velocity to actual
    // maybe integrate error with some decay

    // print status also post to topic /drive_controller_status
    if (wcStat == WaypointController::Status::ALLBAD)
    {
      ROS_WARN("Mode : 0");
      ss << "Mode : Bad";
    }
    else if (wcStat == WaypointController::Status::ALLGOOD)
    {
      ROS_INFO("Mode : 1");
      ss << "Mode: Good";
    }
    else if (wcStat == WaypointController::Status::GOALREACHED)
    {
      ROS_INFO("Mode : 2");
      wc.haltAndAbort();
      ss << "Mode: Chillin";
    }
    msg.data = ss.str();
    mode_pub.publish(msg);
    // print some info
    navigationQueue = wc.getNavigationQueue();
    theCPP = wc.getCPP();

    ROS_INFO("CPPx : %.4f", theCPP.x);
    ROS_INFO ("CPPy : %.4f", theCPP.y);
    ROS_INFO ("CPPth : %.4f", theCPP.theta);

    ROS_INFO("CurPx : %.4f", currPose.x);
    ROS_INFO("CurPy : %.4f", currPose.y);
    ROS_INFO("CurPth : %.4f", currPose.theta);

    ROS_INFO("EtpEstimate : %.4f", wc.getETpEstimate());
    ROS_INFO("EppEstimate : %.4f", wc.getEPpEstimate());

    ROS_INFO("SetSpeed1 : %.4f", wc.getSetSpeeds().first);
    ROS_INFO ("SetSpeed2 : %.4f", wc.getSetSpeeds().second);
    ROS_INFO("CmdSpeed1 : %.4f", wc.getCmdSpeeds().first);
    ROS_INFO ("CmdSpeed2 : %.4f", wc.getCmdSpeeds().second);

    if (navigationQueue.size() > 0) {
        ROS_INFO ("NavManSize : %d", (int) navigationQueue.at(0).mans.size());

        ROS_INFO ("NavMan0rad : %.4f", navigationQueue.at(0).mans.at(0).radius);
        ROS_INFO ("NavMan0nxc : %.4f", navigationQueue.at(0).mans.at(0).xc);
        ROS_INFO ("NavMan0yc : %.4f", navigationQueue.at(0).mans.at(0).yc);
        ROS_INFO ("NavMan0dist : %.4f", navigationQueue.at(0).mans.at(0).distance);
        if (navigationQueue.at(0).mans.size() >= 2) {
            ROS_INFO ("NavMan1rad : %.4f", navigationQueue.at(0).mans.at(1).radius);
            ROS_INFO ("NavMan1nxc : %.4f", navigationQueue.at(0).mans.at(1).xc);
            ROS_INFO ("NavMan1yc : %.4f", navigationQueue.at(0).mans.at(1).yc);
            ROS_INFO ("NavMan1dist : %.4f", navigationQueue.at(0).mans.at(1).distance);
        } else {
            ROS_INFO ("NavMan1rad : %.4f", 0.0);
            ROS_INFO ("NavMan1nxc : %.4f", 0.0);
            ROS_INFO ("NavMan1yc : %.4f", 0.0);
            ROS_INFO ("NavMan1dist : %.4f", 0.0);
        }
        pose manEnd = wc.getManeuverEnd();
        ROS_INFO("CurrManEndx : %.4f", manEnd.x);
        ROS_INFO("CurrManEndy : %.4f", manEnd.y);
        ROS_INFO("CurrManEndTh : %.4f", manEnd.theta);

        ROS_INFO("NavInitPosex : %.4f", navigationQueue.at(0).initialPose.x);
        ROS_INFO("NavInitPosey : %.4f", navigationQueue.at(0).initialPose.y);
        ROS_INFO("NavInitPoseth : %.4f", navigationQueue.at(0).initialPose.theta);

        ROS_INFO("NavTermPosex : %.4f", navigationQueue.at(0).terminalPose.x);
        ROS_INFO("NavTermPosey : %.4f", navigationQueue.at(0).terminalPose.y);
        ROS_INFO("NavTermPoseth : %.4f", navigationQueue.at(0).terminalPose.theta);
    } else {
        ROS_INFO ("NavManSize : %d", 0);

        ROS_INFO ("NavMan0rad : %.4f", 0.0);
        ROS_INFO ("NavMan0nxc : %.4f", 0.0);
        ROS_INFO ("NavMan0yc : %.4f", 0.0);
        ROS_INFO ("NavMan0dist : %.4f", 0.0);
        ROS_INFO ("NavMan1rad : %.4f", 0.0);
        ROS_INFO ("NavMan1nxc : %.4f", 0.0);
        ROS_INFO ("NavMan1yc : %.4f", 0.0);
        ROS_INFO ("NavMan1dist : %.4f", 0.0);


        ROS_INFO("CurrManEndx : %.4f", 0.0);
        ROS_INFO("CurrManEndy : %.4f", 0.0);
        ROS_INFO("CurrManEndTh : %.4f", 0.0);

        ROS_INFO("NavInitPosex : %.4f", 0.0);
        ROS_INFO("NavInitPosey : %.4f", 0.0);
        ROS_INFO("NavInitPoseth : %.4f",0.0);

        ROS_INFO("NavTermPosex : %.4f", 0.0);
        ROS_INFO("NavTermPosey : %.4f", 0.0);
        ROS_INFO("NavTermPoseth : %.4f", 0.0);

    }
    if (halt)
    {
      wc.haltAndAbort();
      break;
    }
    // ros end stuff
    jspub.publish(jsMessage);
    ros::spinOnce();
    rate.sleep();
  }
}
