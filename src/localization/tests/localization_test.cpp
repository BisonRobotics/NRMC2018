#include <apriltag_tracker_interface/apriltag_tracker_interface.h>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_interface/teleop_interface.h"
#include "super_localizer/super_localizer.h"
#include "wheel_params/wheel_params.h"
#include "lp_research/lpresearchimu.h"


float velocity_left = 0.0f;
float velocity_right = 0.0f;

void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
    if (joy->buttons[4])
    {
        velocity_left = joy->axes[1];
        velocity_right = joy->axes[4];
    }
    else
    {
        velocity_left = 0.0f;
        velocity_right = 0.0f;
    }
}


int main (int argc, char **argv){
    ros::init (argc, argv, "localization_tester");
    ros::NodeHandle n;
    ros::Rate r (100);
    ros::Subscriber sub = n.subscribe("joy", 30, callback);
    TeleopInterface teleopInterface (.5f);
    AprilTagTrackerInterface *aprilTags = new AprilTagTrackerInterface();
    LpResearchImu lpResearchImu ("imu");
    SuperLocalizer superLocalizer(ROBOT_AXLE_LENGTH, 0.0f, 0.0f, 0.0f, teleopInterface.bl, teleopInterface.br, teleopInterface.fl, teleopInterface.fr,
       &lpResearchImu, aprilTags, SuperLocalizer_default_gains);
    ros::Time last_time = ros::Time::now();
    while (ros::ok()){
        teleopInterface.update (velocity_left, velocity_right);
        superLocalizer.updateStateVector((ros::Time::now() - last_time).toSec ());
        r.sleep ();
        last_time = ros::Time::now ();
    }

}