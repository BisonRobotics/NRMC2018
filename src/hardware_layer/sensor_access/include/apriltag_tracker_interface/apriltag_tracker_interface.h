#ifndef PROJECT_APRILTAG_TRACKER_INTERFACE_H
#define PROJECT_APRILTAG_TRACKER_INTERFACE_H

#include <sensor_access/pos_sensor_interface.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class AprilTagTrackerInterface : public PosSensorInterface
{
public:
  AprilTagTrackerInterface(void);
  ~AprilTagTrackerInterface();
  double getX() override;
  double getY() override;
  double getTheta() override;
  ReadableSensors::ReadStatus receiveData() override;

private:
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener *tfListener;

  double x;
  double y;
  double theta;
  tf2::Stamped<tf2::Transform> map_to_position_estimate_tf;
};


#endif //PROJECT_APRILTAG_TRACKER_INTERFACE_H
