#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dig_control/DigAction.h>
#include <dig_control/DumpAction.h>

bool dig_commanded=false;
bool dump_commanded=false;
bool dig_happening=false;
bool dump_happening=false;

void callback (const sensor_msgs::Joy::ConstPtr &msg)
{
    if (msg->buttons[5] && !dig_happening && !dump_happening)
    {
        if (msg->buttons[0])    //A
        {
            dig_commanded = true;
        }
        else if (msg->buttons[2])
        {
            dump_commanded = true;
        }
    }
}



int main (int argc, char **argv)
{
    ros::init (argc, argv, "dig_teleop");
    ros::NodeHandle nh;
    ros::Subscriber joy = nh.subscribe ("joy",5,callback);
    actionlib::SimpleActionClient<dig_control::DigAction> dig("dig_server",true);
    actionlib::SimpleActionClient<dig_control::DumpAction> dump("dump_server",true);
    dig.waitForServer();
    dump.waitForServer();
    dig_control::DigGoal dig_goal;
    dig_control::DumpGoal dump_goal;
    ros::Rate r (10);
    dig_goal.angle = 0;
    dump_goal.angle = 0;


    while (ros::ok())
    {
        if (dig_commanded){
            dig_happening = true;
            dig.sendGoal(dig_goal);
            dig.waitForResult(ros::Duration(30.0));
            dig_happening = false;
            dig_commanded = false;
            dump_commanded = false;
            dump_happening = false;
        }
        else if (dump_commanded)
        {
            dump_happening = true;
            dump.sendGoal (dump_goal);
            dump.waitForResult (ros::Duration(30.0));
            dump_happening = false;
            dump_commanded = false;
            dig_happening = false;
            dig_commanded = false;
        }
        r.sleep();
        ros::spinOnce();
    }
}