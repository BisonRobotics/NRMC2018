#include "mapping_filter/low_pass.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(low_pass_namespace::LowPassLayer, costmap_2d::Layer)


using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;


namespace low_pass_namespace
{
  LowPassLayer::LowPassLayer () : buffman (this->size_of_buffer)
  {

  }

  void LowPassLayer::onInitialize ()
  {
    ObstacleLayer::onInitialize();
  }

  void LowPassLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    for (unsigned int i = min_i; i < max_i; i++)
    {
      for (unsigned int j = min_j; j <max_j; j++)
      {
        
      }
    }
  }

}
