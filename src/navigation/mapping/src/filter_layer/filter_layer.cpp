#include "filter_layer/filter_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(filter_layer_namespace::FilterLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace filter_layer_namespace
{

  FilterLayer::FilterLayer()
  {

  }

  void FilterLayer::onInitialize ()
  {
    current_ = true;
    default_value_ = NO_INFORMATION;
    matchSize();
  }

  void FilterLayer::updateBounds (double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y)
  {
    if (!enabled_){
      return;
    }

    *min_x = robot_x;
    *max_x = robot_x + 8.0;
    *min_y = -3.0;
    *max_y = 3.0 ;

  }

  void FilterLayer::updateCosts (costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    cv::Mat current_frame = cv::Mat (master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY(), CV_8U, master_grid.getCharMap());
    buffer.push (current_frame);
    if (this->buffer.size()>queue_size)
    {
      // now we can start the filtering

    }
  }

  void FilterLayer::reset ()
  {

  }

  void FilterLayer::matchSize ()
  {
    Costmap2D* master = layered_costmap_->getCostmap ();
    resizeMap (master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
               master->getOriginX(), master->getOriginY());
    while (!buffer.empty())
    {
      buffer.pop();
    }
  }

}


