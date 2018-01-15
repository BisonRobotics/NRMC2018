#include <ros/ros.h>
#include <wheel_control/distanceAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <position_controller/position_controller.h>
#include <localizer/localizer.h>

float velocity = 0.5f;
PositionController pos(velocity);
typedef actionlib::SimpleActionServer<wheel_control::distanceAction> Server;

void execute(const wheel_control::distanceGoalConstPtr& goal,
             Server* as)  // Note: "Action" is not appended to DoDishes here
{
  pos.setDistance(goal->distance);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheel_controller");
  ros::NodeHandle n;
  Localizer loc =
      Localizer(.5f, 0, 0, 0, pos.front_left_wheel, pos.front_right_wheel, pos.back_right_wheel, pos.back_left_wheel);
  Server server(n, "drive_a_distance", boost::bind(&execute, _1, &server), false);
  loc.updateStateVector(.1f);
  pos.update(loc.getStateVector().x_pos, loc.getStateVector().y_pos);
  server.start();
  ros::Rate r(10);
  wheel_control::distanceFeedback feedback;
  wheel_control::distanceResult result;
  while (ros::ok())
  {
    loc.updateStateVector(.1f);
    pos.update(loc.getStateVector().x_pos, loc.getStateVector().y_pos);
    if (server.isActive())
    {
      if (!pos.isMoving())
      {
        server.setSucceeded();
      }
      else
      {
        feedback.distance_travelled = pos.getDistance() - pos.getDistanceRemaining();
        server.publishFeedback(feedback);
      }
    }
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
