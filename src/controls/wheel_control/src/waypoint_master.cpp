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
#include <waypoint_controller/waypoint_controller_helper.h>
#include <waypoint_controller/waypoint_with_maneuvers.h>

#include <geometry_msgs/Pose2D.h>
//#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
#include <wheel_params/wheel_params.h>

#include <super_localizer/super_localizer.h>
#include <sensor_msgs/JointState.h>

#include <visualization_msgs/Marker.h>
#include <imperio/DriveStatus.h>

#include <sim_robot/sim_robot.h>

#include <lp_research/lpresearchimu.h>
#include <apriltag_tracker_interface/apriltag_tracker_interface.h>

#include <vector>
#include <utility>

#define UPDATE_RATE_HZ 50.0

bool newWaypointHere = false;
pose newWaypoint;
bool halt = false;

double topicTheta=0;
bool thetaHere=false;


void newGoalCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
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

geometry_msgs::TransformStamped create_sim_tf(double x, double y, double theta)
{
  geometry_msgs::TransformStamped tfStamp;
  tfStamp.header.stamp = ros::Time::now();
  tfStamp.header.frame_id = "map";
  tfStamp.child_frame_id = "sim_base_link";
  tfStamp.transform.translation.x = x;
  tfStamp.transform.translation.y = y;
  tfStamp.transform.translation.z = 0.5;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  tfStamp.transform.rotation.x = q.x();
  tfStamp.transform.rotation.y = q.y();
  tfStamp.transform.rotation.z = q.z();
  tfStamp.transform.rotation.w = q.w();
  return tfStamp;
}

void haltCallback(const std_msgs::Empty::ConstPtr &msg)
{
  halt = true;
}

void initialThetaCallback(const std_msgs::Float64::ConstPtr &msg)
{
    topicTheta = msg->data;
    thetaHere = true;
}


bool areTheseEqual (imperio::DriveStatus status1, imperio::DriveStatus status2)
{
  return (status1.is_stuck.data == status2.is_stuck.data && status1.cannot_plan_path.data == status2.cannot_plan_path.data &&
          status1.in_motion.data == status2.in_motion.data && status1.has_reached_goal.data == status2.has_reached_goal.data);
}


int main(int argc, char **argv)
{
  // read ros param for simulating
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node("~");
  ros::NodeHandle globalNode;

  bool simulating;
  if (node.hasParam("simulating_driving"))
  {
    node.getParam("simulating_driving", simulating);
    if (simulating)
    {
      ROS_WARN("\nRUNNING DRIVING AS SIMULATION\n");
    }
    else
    {
      ROS_WARN("\nRUNNING DRIVING AS PHYSICAL\n");
    }
  }
  else
  {
    ROS_ERROR("\n\nsimulating_driving param not defined! aborting.\n\n");
    return -1;
  }


  ros::Subscriber sub = node.subscribe("additional_waypoint", 100, newGoalCallback);
  ros::Publisher jspub = globalNode.advertise<sensor_msgs::JointState>("joint_states", 500);

  ros::Publisher angleErrorPub = node.advertise<std_msgs::Float64>("angle_error", 30);
  ros::Publisher simAnglePub = node.advertise<std_msgs::Float64>("sim_angle", 30);
  ros::Publisher baseAnglePub = node.advertise<std_msgs::Float64>("base_angle", 30);

  std_msgs::Float64 angleErrorMsg;
  std_msgs::Float64 simAngleMsg;
  std_msgs::Float64 baseAngleMsg;

  tf2_ros::Buffer tfBuffer;
  // tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  // pose theWay = {0,0,0};
  pose currPose = { 0, 0, 0 };
  pose theCPP = { 0, 0, 0 };

  std::vector<waypointWithManeuvers> navigationQueue;

  SimRobot *sim;
  iVescAccess *fl, *fr, *br, *bl;
  ImuSensorInterface *imu;
  PosSensorInterface *pos;

  if (simulating == true)
  {
    sim = new SimRobot(ROBOT_AXLE_LENGTH, .5, .5, 0);
    fl = (sim->getFLVesc());
    fr = (sim->getFRVesc());
    br = (sim->getBRVesc());
    bl = (sim->getBLVesc());

    imu = sim->getImu();
    pos = sim->getPos();
  }
  else
  {
    sim = NULL;  // Make no reference to the sim if not simulating
    fl = new VescAccess(front_left_param);
    fr = new VescAccess(front_right_param);
    br = new VescAccess(back_right_param);
    bl = new VescAccess(back_left_param);
    pos = new AprilTagTrackerInterface("/pose_estimate", .1);
    imu = new LpResearchImu("imu_base_link");
  }

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
  ros::Publisher mode_pub = node.advertise<imperio::DriveStatus>("drive_controller_status", 1000, true);
  ros::Publisher path_marker_pub = node.advertise<visualization_msgs::Marker>("waypoint_path", 10000);
  ros::Publisher wholeQueue_pub = node.advertise<visualization_msgs::Marker>("whole_queue", 100);

  imperio::DriveStatus status_msg;
  imperio::DriveStatus last_msg;

  visualization_msgs::Marker line_strip;
  status_msg.header.seq = 0;
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = .1;
  line_strip.color.g = 1;
  line_strip.color.a = 1;
  line_strip.header.frame_id = "/map";

  visualization_msgs::Marker line_strip2;
  line_strip2.action = visualization_msgs::Marker::ADD;
  line_strip2.pose.orientation.w = 1;
  line_strip2.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip2.scale.x = .1;
  line_strip2.color.b = 1;
  line_strip2.color.a = 1;
  line_strip2.header.frame_id = "/map";

  double wheel_positions[4] = {0};

  geometry_msgs::Point vis_point;
  // hang here until someone knows where we are
  ROS_INFO("Going into wait loop for localizer and initial theta...");

  ros::Rate rate(UPDATE_RATE_HZ);
  ros::Duration idealLoopTime(1.0 / UPDATE_RATE_HZ);

  ros::Subscriber initialThetaSub = node.subscribe("initialTheta", 100, initialThetaCallback);

  while ((!superLocalizer.getIsDataGood() && ros::ok()) || (ros::ok() && !thetaHere))
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
    if (simulating)
    {
      sim->update((loopTime).toSec());
      tfBroad.sendTransform(create_sim_tf(sim->getX(), sim->getY(), sim->getTheta()));
    }
    superLocalizer.updateStateVector(loopTime.toSec());
    stateVector = superLocalizer.getStateVector();
    tfBroad.sendTransform(create_tf(stateVector.x_pos, stateVector.y_pos, stateVector.theta));
    ros::spinOnce();
    rate.sleep();
  }
  
  //zero point turn vescs here before waypoint controller is initialized
  //get number from topic  
  double topicthetatol = .1;
  if (node.hasParam("initial_theta_tolerance"))
  {
    node.getParam("initial_theta_tolerance", topicthetatol);
  }
  else
  {
    ROS_ERROR("\n\ninitial_theta_tolerance param not defined! aborting.\n\n");
    return -1;
  }

  double zeroPointTurnGain;
  if (node.hasParam("initial_theta_gain"))
  {
    node.getParam("initial_theta_gain",zeroPointTurnGain);
  }
  else
  {
    ROS_ERROR("\n\ninitial_theta_gain param not defined! aborting.\n\n");
    return -1;
  }

  ROS_INFO("Theta received, going into initial turn.");
  firstTime = true;
  while (ros::ok() && std::abs(WaypointControllerHelper::anglediff(stateVector.theta, topicTheta)) > topicthetatol)
  {
    double speed = zeroPointTurnGain * WaypointControllerHelper::anglediff(stateVector.theta, topicTheta);

    fr->setLinearVelocity(-speed);
    br->setLinearVelocity(-speed);
    fl->setLinearVelocity(speed);
    bl->setLinearVelocity(speed);

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
    if (simulating)
    {
        sim->update((loopTime).toSec());
        tfBroad.sendTransform(create_sim_tf(sim->getX(), sim->getY(), sim->getTheta()));
    }
    superLocalizer.updateStateVector(loopTime.toSec());
    stateVector = superLocalizer.getStateVector();
    tfBroad.sendTransform(create_tf(stateVector.x_pos, stateVector.y_pos, stateVector.theta));
    ros::spinOnce();
    rate.sleep();
  }

  // initialize waypoint controller
  WaypointController wc = WaypointController(ROBOT_AXLE_LENGTH, ROBOT_MAX_SPEED, currPose, fl, fr, br, bl,
                                             1.0 / UPDATE_RATE_HZ, waypoint_default_gains);
  ROS_INFO("Top");
  ROS_INFO("Xposgain : %.4f", SuperLocalizer_default_gains.x_pos);
  ROS_INFO("Yposgain : %.4f", SuperLocalizer_default_gains.y_pos);
  ROS_INFO("Thetagain : %.4f", SuperLocalizer_default_gains.theta);
  ROS_INFO("Xvelgain : %.4f", SuperLocalizer_default_gains.x_vel);
  ROS_INFO("Yvelgain : %.4f", SuperLocalizer_default_gains.y_vel);
  ROS_INFO("Omegagain : %.4f", SuperLocalizer_default_gains.omega);
  ROS_INFO("Xaccelgain : %.4f", SuperLocalizer_default_gains.x_accel);
  ROS_INFO("Yaccelgain : %.4f", SuperLocalizer_default_gains.y_accel);
  ROS_INFO("Alphagain : %.4f", SuperLocalizer_default_gains.alpha);

  ROS_INFO("eppgain : %.4f", waypoint_default_gains.eppgain);
  ROS_INFO("epdgain : %.4f", waypoint_default_gains.epdgain);
  ROS_INFO("etpgain : %.4f", waypoint_default_gains.etpgain);
  ROS_INFO("etdgain : %.4f", waypoint_default_gains.etdgain);
  ROS_INFO("epplpgain : %.4f", waypoint_default_gains.epplpgain);
  ROS_INFO("etplpgain : %.4f", waypoint_default_gains.etplpgain);
  ROS_INFO("wheelalpha : %.4f", waypoint_default_gains.wheelalpha);
  WaypointController::Status wcStat;
  std_msgs::String msg;
  std::stringstream ss;
  // geometry_msgs::PoseStamped poser;
  firstTime = true;
  while (ros::ok())
  {
    ROS_INFO("\n");
    ROS_INFO("Top");
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
    if (simulating)
    {
      sim->update(loopTime.toSec());

      tfBroad.sendTransform(create_sim_tf(sim->getX(), sim->getY(), sim->getTheta()));
      // also publish marker
    }

    superLocalizer.updateStateVector(loopTime.toSec());
    stateVector = superLocalizer.getStateVector();

    tfBroad.sendTransform(create_tf(stateVector.x_pos, stateVector.y_pos, stateVector.theta));
    // also publish marker

    currPose.x = stateVector.x_pos;
    currPose.y = stateVector.y_pos;
    currPose.theta = stateVector.theta;
    // can we infer effective wheel velocities (the velocity of the wheel if we were moving
    //                                         how we are, but no slip)
    // need to estimate speed and turn radius
    // speed is easy, just norm of velocities
    // turn radius...  ddistance / dtheta?
    // turn radius =  speed*dt / alpha * dt ;  //do we want to average this? over a second maybe?
    // also be sure to clamp radius at something (1000)
    sensor_msgs::JointState jsMessage;

    jsMessage.name.push_back("frame_to_front_left_wheel");
    jsMessage.name.push_back("frame_to_front_right_wheel");
    jsMessage.name.push_back("frame_to_back_right_wheel");
    jsMessage.name.push_back("frame_to_back_left_wheel");

    jsMessage.header.stamp = ros::Time::now();
    wheel_positions[0] += fl->getLinearVelocity() / UPDATE_RATE_HZ;
    wheel_positions[1] += fr->getLinearVelocity() / UPDATE_RATE_HZ;
    wheel_positions[2] += br->getLinearVelocity() / UPDATE_RATE_HZ;
    wheel_positions[3] += bl->getLinearVelocity() / UPDATE_RATE_HZ;
    jsMessage.position.push_back(wheel_positions[0]);
    jsMessage.position.push_back(wheel_positions[1]);
    jsMessage.position.push_back(wheel_positions[2]);
    jsMessage.position.push_back(wheel_positions[3]);

    jsMessage.velocity.push_back(fl->getLinearVelocity());
    jsMessage.velocity.push_back(fr->getLinearVelocity());
    jsMessage.velocity.push_back(br->getLinearVelocity());
    jsMessage.velocity.push_back(bl->getLinearVelocity());

    jspub.publish(jsMessage);

    ROS_INFO("FrontLeftVel : %.4f", jsMessage.velocity[0]);
    ROS_INFO("FrontRightVel : %.4f", jsMessage.velocity[1]);
    ROS_INFO("BackRightVel : %.4f", jsMessage.velocity[2]);
    ROS_INFO("BackLeftVel : %.4f", jsMessage.velocity[3]);

    if (newWaypointHere)
    {
      waypoint_set = wc.addWaypoint(newWaypoint, currPose);
      line_strip.points.clear();
      for (auto const &waypoint : waypoint_set)
      {
        vis_point.x = waypoint.first;
        vis_point.y = waypoint.second;
        vis_point.z = .4;
        line_strip.points.push_back(vis_point);
      }
      path_marker_pub.publish(line_strip);
      newWaypointHere = false;
      ROS_INFO("NewWaypoint : 1");
    }
    else
    {
      ROS_INFO("NewWaypoint : 0");
    }

    // update controller

    wcStat = wc.update(stateVector, loopTime.toSec());

    // TODO
    // check if we are stuck by comparing commanded velocity to actual
    // maybe integrate error with some decay
    status_msg.has_reached_goal.data = 0;
    status_msg.in_motion.data = 0;
    status_msg.cannot_plan_path.data = 0;
    status_msg.is_stuck.data = 0;
    status_msg.header.stamp = ros::Time::now();
    status_msg.header.seq++;
    // print status also post to topic /drive_controller_status
    if (wcStat == WaypointController::Status::OFFPATH)
    {
      ROS_WARN("Mode : -1");
      ss << "Mode : OFFPATH";
      status_msg.cannot_plan_path.data = 0;
    }
    if (wcStat == WaypointController::Status::OVERSHOT)
    {
      ROS_WARN("Mode : 0");
      ss << "Mode : OVERSHOT";
      status_msg.in_motion.data = 1;
    }
    else if (wcStat == WaypointController::Status::ALLGOOD)
    {
      ROS_INFO("Mode : 1");
      ss << "Mode: ALLGOOD";
      status_msg.in_motion.data = 1;
    }
    else if (wcStat == WaypointController::Status::GOALREACHED)
    {
      ROS_INFO("Mode : 2");
      wc.haltAndAbort();
      ss << "Mode: GOALRECHED";
      status_msg.has_reached_goal.data = 1;
    }
    if (!firstTime)
    {
      if (!areTheseEqual(status_msg, last_msg))
      {
        mode_pub.publish(status_msg);
      }
    } else {
      mode_pub.publish(status_msg);
    }

    last_msg = status_msg;
    // print some info
    navigationQueue = wc.getNavigationQueue();
    theCPP = wc.getCPP();
    // iterate through navigationqueue elements
    line_strip2.points.clear();
    line_strip2.color.r = 0;
    line_strip2.color.b = 1;
    for (auto const &myMan : navigationQueue)
    {
      waypoint_set = WaypointControllerHelper::waypointWithManeuvers2points(myMan);
      for (auto const &waypoint : waypoint_set)
      {
        vis_point.x = waypoint.first;
        vis_point.y = waypoint.second;
        vis_point.z = .2;
        line_strip2.points.push_back(vis_point);
      }
      line_strip2.color.b -= (line_strip2.color.b <= 0.0) ? -1 : .1;
      line_strip2.color.r += (line_strip2.color.r >= 1.0) ? 0 : .1;
    }
    wholeQueue_pub.publish(line_strip2);

    // publish wc.getEPpEstimate() as topic
    // publish sim theta sim->getTheta()
    // publish base link theta stateVector.theta
    angleErrorMsg.data = wc.getEPpEstimate();
    baseAngleMsg.data = stateVector.theta;
    angleErrorPub.publish(angleErrorMsg);
    baseAnglePub.publish(baseAngleMsg);
    if (simulating)
    {
      simAngleMsg.data = sim->getTheta();
      simAnglePub.publish(simAngleMsg);
    }

    ROS_INFO("CPPx : %.4f", theCPP.x);
    ROS_INFO("CPPy : %.4f", theCPP.y);
    ROS_INFO("CPPth : %.4f", theCPP.theta);

    ROS_INFO("CurPx : %.4f", currPose.x);
    ROS_INFO("CurPy : %.4f", currPose.y);
    ROS_INFO("CurPth : %.4f", currPose.theta);

    ROS_INFO("Dist2endOnPath : %.4f", wc.getDist2endOnPath());
    ROS_INFO("Dist2endAbs : %.4f", wc.getDist2endAbs());


    ROS_INFO("EtpEstimate : %.4f", wc.getETpEstimate());
    ROS_INFO("EppEstimate : %.4f", wc.getEPpEstimate());

    ROS_INFO("SetSpeed1 : %.4f", wc.getSetSpeeds().first);
    ROS_INFO("SetSpeed2 : %.4f", wc.getSetSpeeds().second);
    ROS_INFO("CmdSpeed1 : %.4f", wc.getCmdSpeeds().first);
    ROS_INFO("CmdSpeed2 : %.4f", wc.getCmdSpeeds().second);

    ROS_INFO("currMan Index : %d", wc.getCurrManeuverIndex());

    if (navigationQueue.size() > 0)
    {
      ROS_INFO("NavManSize : %d", (int)navigationQueue.at(0).mans.size());

      ROS_INFO("NavMan0rad : %.4f", navigationQueue.at(0).mans.at(0).radius);
      ROS_INFO("NavMan0nxc : %.4f", navigationQueue.at(0).mans.at(0).xc);
      ROS_INFO("NavMan0yc : %.4f", navigationQueue.at(0).mans.at(0).yc);
      ROS_INFO("NavMan0dist : %.4f", navigationQueue.at(0).mans.at(0).distance);
      if (navigationQueue.at(0).mans.size() >= 2)
      {
        ROS_INFO("NavMan1rad : %.4f", navigationQueue.at(0).mans.at(1).radius);
        ROS_INFO("NavMan1nxc : %.4f", navigationQueue.at(0).mans.at(1).xc);
        ROS_INFO("NavMan1yc : %.4f", navigationQueue.at(0).mans.at(1).yc);
        ROS_INFO("NavMan1dist : %.4f", navigationQueue.at(0).mans.at(1).distance);
      }
      else
      {
        ROS_INFO("NavMan1rad : %.4f", 0.0);
        ROS_INFO("NavMan1nxc : %.4f", 0.0);
        ROS_INFO("NavMan1yc : %.4f", 0.0);
        ROS_INFO("NavMan1dist : %.4f", 0.0);
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
    }
    else
    {
      ROS_INFO("NavManSize : %d", 0);

      ROS_INFO("NavMan0rad : %.4f", 0.0);
      ROS_INFO("NavMan0nxc : %.4f", 0.0);
      ROS_INFO("NavMan0yc : %.4f", 0.0);
      ROS_INFO("NavMan0dist : %.4f", 0.0);
      ROS_INFO("NavMan1rad : %.4f", 0.0);
      ROS_INFO("NavMan1nxc : %.4f", 0.0);
      ROS_INFO("NavMan1yc : %.4f", 0.0);
      ROS_INFO("NavMan1dist : %.4f", 0.0);

      ROS_INFO("CurrManEndx : %.4f", 0.0);
      ROS_INFO("CurrManEndy : %.4f", 0.0);
      ROS_INFO("CurrManEndTh : %.4f", 0.0);

      ROS_INFO("NavInitPosex : %.4f", 0.0);
      ROS_INFO("NavInitPosey : %.4f", 0.0);
      ROS_INFO("NavInitPoseth : %.4f", 0.0);

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
    ros::spinOnce();
    rate.sleep();
  }
}
