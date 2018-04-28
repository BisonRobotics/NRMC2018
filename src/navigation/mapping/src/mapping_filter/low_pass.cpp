#include "mapping_filter/low_pass.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(low_pass_namespace::LowPassLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace low_pass_namespace
{
LowPassLayer::LowPassLayer()
{
}

void LowPassLayer::onInitialize()
{
  ObstacleLayer::onInitialize();
  ros::NodeHandle nh("~/" + name_);
  ros::NodeHandle gl_nh;
  current_ = true;
  default_value_ = NO_INFORMATION;  // we can change this later if we want
  enabled_ = false;
  enable_service = gl_nh.advertiseService ("enable_mapping", &LowPassLayer::updateEnable, this);
}

bool LowPassLayer::updateEnable (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  enabled_ = req.data;
  ROS_INFO_STREAM ("Enabled: " << enabled_);
  res.success=true;
  return true;
}

void LowPassLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (enabled_)
  {
    unsigned char *my_map = master_grid.getCharMap();
    unsigned int size_of_map = master_grid.getSizeInCellsX() * master_grid.getSizeInCellsY();
    cv::Mat input = cv::Mat(master_grid.getSizeInCellsY(), master_grid.getSizeInCellsX(), CV_8U, my_map,
                            cv::Mat::AUTO_STEP);
    cv::Mat filtered;

    cv::medianBlur(input, filtered, size_of_kern);

    std::memcpy(my_map, filtered.data, sizeof(unsigned char) * size_of_map);
    ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
  }
}
}
