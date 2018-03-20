#ifndef PROJECT_FILTER_LAYER_H
#define PROJECT_FILTER_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/core/mat.hpp>


namespace filter_layer_namespace
{
  class FilterLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
  {
  public:
    FilterLayer ();
    virtual void onInitialize ();
    virtual void updateBounds (double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y);
    virtual void updateCosts (costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);
    virtual void reset ();
    virtual void matchSize ();
    bool isDiscretized (){
      return true;
    }
  private:
    std::queue<cv::Mat> buffer;
    static constexpr unsigned int queue_size = 15;
    static constexpr double weights[queue_size] = {1/(queue_size*1.0)}; // simple average for now
    static constexpr double cutoff = 240.5;

  };
}

#endif //PROJECT_FILTER_LAYER_H
