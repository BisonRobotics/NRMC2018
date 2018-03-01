#include<dig_control/OutriggerAction.h>
#include<actionlib/server/simple_action_server.h>


void execute(const dig_control::outriggerGoalConstPtr & goal, 
             actionlib::SimpleActionServer<dig_control::outriggerAction> * serv_ptr)
{


  as->setSucceeded();
}






int main(int argc, char** argv)
{
  ros::init(argc, argv, "outriggerAction.h");
  ros::NodeHandle n;
  actionlib::SimpleActionServer<dig_control::outriggerAction> server(n, "outriggers_server", 
                                       boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}