#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <waypoint_controller/waypoint_controller.h>
#include <waypoint_controller/waypointWithManeuvers.h>
#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
#include <wheel_params/wheel_params.h>
#include <vector>
#include <utility>
//#include <actionlib/server/simple_action_server.h>
//#include <wheel_control/waypointAction.h>
#include <std_msgs/Bool.h>
bool stop = false;
//typedef actionlib::SimpleActionServer<wheel_control::waypointAction> Server;
/*bool new_goal = false;
float gx, gy, gtheta;
bool stop = false;
void execute(const wheel_control::waypointGoalConstPtr& goal, Server *as){
	if (!new_goal){
		gx = goal->waypoint[0];
		gy = goal->waypoint[1];
		gtheta = goal->waypoint[2];
		new_goal = true;
		
	}
}

*/
void callback(const std_msgs::Bool::ConstPtr &msg){
	if (msg->data){
		stop = true;
	}
}


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");
  float goalx, goaly, goalr;
  goalx = 3.0f;
  goaly = 1.0f;
  goalr = 1.0f;
  bool isRunning = true;
  ros::NodeHandle node;
  /*
  Initial pose of x = .75, y = -1.0,th = 0;
  theWay x = 2, y = -.5, th = pi/4; works


  */
  //Server server(node, "drive_a_distance", boost::bind(&execute, _1, &server), false);
  //server.start ();
  ros::Subscriber sub = node.subscribe("stopper", 1000, callback);
  ros::Time curr;
  ros::Time last;
  ros::Duration delta;
  tf::StampedTransform transform;
  tf::TransformListener listener;
  pose theWay = {.x = 3.0f, .y = 0.0f, .theta = 1.0f};
  pose currPose;
  pose theCPP;
  listener.waitForTransform ("/map", "/base_link", ros::Time(0), ros::Duration(30));
        listener.lookupTransform("/map", "/base_link",
                               ros::Time(0), transform);
  //pose wcInitial = {.x = .75, .y = 1.0f, .theta = 0 };
  currPose.x = -1.0f*transform.getOrigin().x();
  currPose.y = -1.0f*transform.getOrigin().y();

   //transform.getRotation().getAxis(); //need to make sure axis is roughly +Z
  currPose.theta = -transform.getRotation().getAngle();
  std::vector<waypointWithManeuvers> navigationQueue;
  char * can_name = (char *)WHEEL_CAN_NETWORK;
  VescAccess fl(FRONT_LEFT_WHEEL_ID, WHEEL_GEAR_RATIO,WHEEL_OUTPUT_RATIO, MAX_WHEEL_VELOCITY,MAX_WHEEL_TORQUE,WHEEL_TORQUE_CONSTANT, can_name, 1);
  VescAccess fr(FRONT_RIGHT_WHEEL_ID, WHEEL_GEAR_RATIO, -1.0f*WHEEL_OUTPUT_RATIO,MAX_WHEEL_VELOCITY,MAX_WHEEL_TORQUE,WHEEL_TORQUE_CONSTANT, can_name, 1);
  VescAccess br(BACK_RIGHT_WHEEL_ID, WHEEL_GEAR_RATIO, -1.0f*WHEEL_OUTPUT_RATIO,MAX_WHEEL_VELOCITY,MAX_WHEEL_TORQUE,WHEEL_TORQUE_CONSTANT, can_name, 1);
  VescAccess bl(BACK_LEFT_WHEEL_ID, WHEEL_GEAR_RATIO, WHEEL_OUTPUT_RATIO,MAX_WHEEL_VELOCITY,MAX_WHEEL_TORQUE,WHEEL_TORQUE_CONSTANT, can_name, 1);

  WaypointController wc = WaypointController(.5f, 1.0f,currPose, &fl, &fr, &br, &bl);
    WaypointController::Status wcStat;

std::vector<std::pair<float, float> > returnPoints = wc.addWaypoint(theWay, currPose);
  //ros::service::waitForService("spawn");
  //ros::ServiceClient add_turtle =
   // node.serviceClient<turtlesim::Spawn>("spawn");
  //turtlesim::Spawn srv;
  //add_turtle.call(srv);


  last = ros::Time::now ();
  ros::Rate rate(50.0);
  while (node.ok())
  {
    try{
      listener.waitForTransform ("/map", "/base_link", ros::Time(0), ros::Duration(10));
      listener.lookupTransform("/map", "/base_link",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
	if (stop){
		ROS_INFO ("HALT");
		wc.haltAndAbort();
		node.shutdown ();
	}
	/*	theWay.x = gx;
		theWay.y = gy;
		theWay.theta = gtheta;
		new_goal = false;
		isRunning = true;
	}*/
   currPose.x = -1.0f*transform.getOrigin().x();
   currPose.y = -1.0f*transform.getOrigin().y();

   //transform.getRotation().getAxis(); //need to make sure axis is roughly +Z
   currPose.theta = transform.getRotation().getAngle();

   ROS_INFO("GOING TO UPDATE 5");
   curr = ros::Time::now ();
   delta = curr - last;
   wcStat = wc.update(currPose, (float)delta.toSec()); //need to get actual time
   last = curr;

   if (wcStat == WaypointController::Status::ALLBAD) ROS_INFO("CONTROLLER SAYS BAD");
   else if (wcStat == WaypointController::Status::ALLGOOD) ROS_INFO("CONTROLLER SAYS GOOD");
   else if (wcStat == WaypointController::Status::GOALREACHED){
	ROS_INFO("GOOOOOAAAAALLLLL!!");
//	if (server.isActive()){
	if (isRunning){
//		server.setSucceeded ();
		isRunning = false;
		ros::shutdown ();
	}
//	}
   }
   navigationQueue = wc.getNavigationQueue();
   theCPP = wc.getCPP();

   ROS_INFO("\nCPP x: %.4f \nCPP y: %.4f\nCPP th:%.4f", theCPP.x, theCPP.y, theCPP.theta);

   ROS_INFO("\nCurP x: %.4f \nCurP y: %.4f\nCurP th:%.4f", currPose.x, currPose.y, currPose.theta);

   ROS_INFO("Etp Estimate: %.4f",wc.getETpEstimate());
   ROS_INFO("Epp Estimate: %.4f",wc.getEPpEstimate());


   ROS_INFO("SetSpeeds: %.4f, %.4f", wc.getSetSpeeds().first, wc.getSetSpeeds().second);

   if (navigationQueue.size() >0)
   {
      ROS_INFO("Nav mans Size: %d", (int)navigationQueue.at(0).mans.size());
      for (int k=wc.getCurrManeuverIndex(); k<navigationQueue.at(0).mans.size();k++) ROS_INFO("Nav Man %d Param:\nradius: %.4f\nxc: %.4f\nyc: %.4f\ndistance: %.4f",k,navigationQueue.at(0).mans.at(k).radius,navigationQueue.at(0).mans.at(k).xc,navigationQueue.at(0).mans.at(k).yc,navigationQueue.at(0).mans.at(k).distance);
      pose manEnd = wc.getManeuverEnd();
      ROS_INFO("CurrManEnd:\nx: %.4f\ny: %.4f\nth: %.4f", manEnd.x, manEnd.y, manEnd.theta);
      ROS_INFO("Nav initial Pose:\nx: %.4f\ny: %.4f\nth: %.4f",navigationQueue.at(0).initialPose.x, navigationQueue.at(0).initialPose.y, navigationQueue.at(0).initialPose.theta);
      ROS_INFO("Nav terminal Pose:\nx: %.4f\ny: %.4f\nth: %.4f",navigationQueue.at(0).terminalPose.x, navigationQueue.at(0).terminalPose.y, navigationQueue.at(0).terminalPose.theta);

   }
   //ROS_INFO("Lvel: %.4f   Rvel: %.4f" ,fl.linVel, fr.linVel);
  // daJoint.name = { "wheel_front_left", "wheel_front_right", "wheel_back_right", "wheel_back_left"};
  // daJoint.velocity = {fl.linVel, fr.linVel,br.linVel, bl.linVel};

   //wheel_vels.publish(daJoint);

    ros::spinOnce ();
    rate.sleep();
  }
  return 0;
};
