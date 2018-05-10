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
  enabled_ = true;
  enable_service = nh.advertiseService ("enable_mapping", &LowPassLayer::updateEnable, this);
  my_personal_enabled_ = false;
}


bool LowPassLayer::updateEnable (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  my_personal_enabled_ = req.data;
  ROS_INFO_STREAM ("Enabled: " << enabled_);
  res.success=true;
  return true;
}

void LowPassLayer::updateBounds (double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y)
{
  if (my_personal_enabled_)
  {
      ObstacleLayer::updateBounds (robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
      unsigned int number_of_cols_map = this->getSizeInCellsY();
      unsigned int number_of_rows_map = this->getSizeInCellsX();

      cv::Mat input (number_of_cols_map, number_of_rows_map, CV_8U, costmap_);
      cv::Mat filtered (number_of_cols_map, number_of_rows_map, CV_8U);
      cv::medianBlur (input, filtered, size_of_kern);
      std::memcpy (costmap_, filtered.data, sizeof(uint8_t)*number_of_cols_map*number_of_rows_map);
  }
}
}
