#ifndef PROJECT_LOW_PASS_H
#define PROJECT_LOW_PASS_H

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/obstacle_layer.h>
#include <dynamic_reconfigure/server.h>
#include <boost/circular_buffer.hpp>
#include "opencv2/opencv.hpp"
#include <std_srvs/SetBool.h>

namespace low_pass_namespace
{
class LowPassLayer : public costmap_2d::ObstacleLayer
{
public:
  explicit LowPassLayer();
  void onInitialize() override;
  bool updateEnable (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void updateBounds (double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y);
private:
  static constexpr unsigned int size_of_kern = 7;
  ros::ServiceServer enable_service;
  bool my_personal_enabled_;
};
}

#endif  // PROJECT_LOW_PASS_H
