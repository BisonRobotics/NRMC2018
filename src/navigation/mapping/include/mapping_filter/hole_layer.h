#ifndef PROJECT_HOLE_LAYER_H
#define PROJECT_HOLE_LAYER_H

#include "low_pass.h"

namespace hole_layer
{
  class HoleLayer : public low_pass_namespace::LowPassLayer
  {
  public:
    explicit HoleLayer(){hole_cost=128;};
    void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                      double *min_y, double *max_x, double *max_y);
    void onInitialize ();
  private:
    uint8_t hole_cost;
  };
}

#endif //PROJECT_HOLE_LAYER_H