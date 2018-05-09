#ifndef PROJECT_APRILTAG_TRACKER_INTERFACE_H
#define PROJECT_APRILTAG_TRACKER_INTERFACE_H

#include <sensor_access/pos_sensor_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
class AprilTagTrackerInterface : public PosSensorInterface
{
public:
  AprilTagTrackerInterface(std::string topic, double timeout);
  ~AprilTagTrackerInterface();
  double getX() override;
  double getY() override;
  double getTheta() override;
  bool isFloating() override;
  ReadableSensors::ReadStatus receiveData() override;
  double getZ () override;
private:
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::NodeHandle nh_;
  bool is_floating;
  void callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  double x;
  double y;
  double theta;
  double z;
  double qtToTheta(geometry_msgs::Quaternion);
  ros::Duration timeout;
  ros::Time last_time;
  void updateIsFloating();
};

#endif  // PROJECT_APRILTAG_TRACKER_INTERFACE_H
