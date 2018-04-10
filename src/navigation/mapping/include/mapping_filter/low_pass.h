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

namespace low_pass_namespace
{
  class LowPassLayer : public costmap_2d::ObstacleLayer
  {
  public:
    explicit LowPassLayer ();
    ~LowPassLayer();
    void onInitialize () override;
    void updateCosts (costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
  private:
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv;
    static constexpr unsigned int size_of_kern = 5;
  };
}



#endif //PROJECT_LOW_PASS_H
