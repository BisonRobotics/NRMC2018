#include <sensor_msgs/Joy.h>
#include <safety_vesc/linear_safety_controller.h>
#include <ros/ros.h>
#include "wheel_params/wheel_params.h"
#include "vesc_access/vesc_access.h"

#define NAMED_PIPE  "Linear_Test"

float torque = 0;
double linear_scalar;


void callback (const sensor_msgs::Joy::ConstPtr &msg)
{
    if (msg->buttons[4])
    {
        linear_scalar = msg->axes[1]*linear_scalar;
    }
    else
    {
        torque = 0;
    }
}

int main (int argc, char**argv)
{
    ros::init (argc, argv, "linear_test");
    ros::NodeHandle gl_n;
    ros::Subscriber my_sub = gl_n.subscribe("linear_joy",100, callback);

    if (!gl_n.getParam("linear_scale_factor", linear_scalar))
    {
        linear_scalar = .01;
        ROS_INFO_NAMED (NAMED_PIPE,"Linear scalar not found");
    }
    else
    {
        ROS_INFO_NAMED (NAMED_PIPE, "Linear scalar : %.4f", linear_scalar);
    }
    iVescAccess *linear_vesc = new VescAccess (linear_param,true);
    LinearSafetyController linearSafetyController(linear_joint_params,linear_vesc);
    ros::Rate r(10);
    bool init_yet=false;

    while (ros::ok() && !init_yet)
    {
        init_yet = linearSafetyController.init();
        r.sleep ();
    }

    while (ros::ok())
    {
        ros::Time last = ros::Time::now();
        if (std::fabs(torque) > .001) {
            linearSafetyController.setTorque(torque);
        } else {
            linearSafetyController.stop();
        }
        r.sleep ();
        ros::spinOnce();
        linearSafetyController.update((ros::Time::now()-last).toSec());
        ROS_INFO_NAMED(NAMED_PIPE, "linear position: %.4f", linearSafetyController.getPositionEstimate());
    }
}
