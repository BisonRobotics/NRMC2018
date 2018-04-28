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
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
  bool updateEnable (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
private:
  static constexpr unsigned int size_of_kern = 35;
  ros::ServiceServer enable_service;
};
}

#endif  // PROJECT_LOW_PASS_H
