#include <ros/ros.h>
#include <wheel_control/distanceAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <position_controller/position_controller.h>


float velocity = 0.5f;
PositionController pos (velocity);
typedef actionlib::SimpleActionServer<wheel_control::distanceAction> Server;

void execute(const wheel_control::distanceGoalConstPtr& goal,
             Server* as)  // Note: "Action" is not appended to DoDishes here
{
  pos.setDistance (goal->distance);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheel_controller");
  ros::NodeHandle n;
  Server server(n, "drive a distance", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::Rate r (100);
  wheel_control::distanceFeedback feedback;
  wheel_control::distanceResult result;
  while (ros::ok()){
    pos.update (0.0f, 0.0f); 
    if (server.isActive()){
      if (!pos.isMoving ()){
        server.setSucceeded ();
      }else {
        feedback.distance_travelled = pos.getDistance() - pos.getDistanceRemaining();
        server.publishFeedback (feedback );
      } 
    } 
    ros::spinOnce ();
    r.sleep ();
  }
  return 0;
}
