#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>

bool start_scan = false;

bool scan_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    start_scan = true;
    return true;
}


int main (int argc, char **argv)
{
    ros::init (argc, argv, "dummy_node");
    ros::NodeHandle nh;

    ros::ServiceServer scan_service = nh.advertiseService("zr300/scan", scan_callback);
    ros::Rate r (10);
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("zr300/mapping_is_good",1);
    std_msgs::Bool mapping_good;
    mapping_good.data = 0;
    while (ros::ok())
    {
        mapping_good.data=start_scan;
        pub.publish (mapping_good);
        r.sleep();
        ros::spinOnce ();
    }
}