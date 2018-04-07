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

  LowPassLayer::~LowPassLayer ()
  {
    delete dsrv;
    delete map;
  }
  void LowPassLayer::onInitialize ()
  {
    ObstacleLayer::onInitialize();
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    default_value_ = NO_INFORMATION;  // we can change this later if we want
    dsrv = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb
        = boost::bind (&LowPassLayer::reconfigureCB,this, _1, _2);
    dsrv->setCallback(cb);
  }

  void LowPassLayer::matchSize ()
  {
    ObstacleLayer::matchSize(); // call the default behavior
    delete[] map;// delete the old map
    Costmap2D *master = layered_costmap_->getCostmap();
    map = new unsigned char[master->getSizeInCellsY()*master->getSizeInCellsX()];// reallocate the internal map
    buffman.clear();// flush the buffer
  }

  void LowPassLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    // copy the array into the buffer
    char* array = new char [master_grid.getSizeInCellsX()*master_grid.getSizeInCellsY()];
    for (int i = min_i; i < max_i; i++)
    {
      for (int j = min_j; j < max_j;j++)
      {
        array[getIndex (i,j)] = costmap_[getIndex (i,j)];
      }
    }
    buffman.push_back (costmap_);

    if (buffman.full()) // if we have all of the observations
    {
      unsigned char *master_map = master_grid.getCharMap();
      updateFilter (min_i, min_j, max_i, max_j);  // run the filter
      for (int i = min_i; i < max_i; i++)
      {
        for (int j = min_j; j < max_j; j++)
        {
          unsigned int index = getIndex(i,j);
          master_map[index] = map[index];
          // its guaranteed by our filter that map[index] will not exceed 255
        }
      }
    }
    delete[] array;
  }

  void LowPassLayer::updateFilter (int min_i, int min_j, int max_i, int max_j)
  {
    // for now we'll do a simple average & threshold
    // this is particularly inneficient if we stick with an average
    // if we stick with an average, we just have to subtract the last item
    // and add in the first item, but this will allow for a more general filter
    // down the road
    for (int i = min_i; i < max_i; i++)
    {
      for (int j = min_j; j < max_j; j++)
      {
        unsigned int index = getIndex(i, j);
        unsigned int sum_var = 0;
        for (unsigned int ctr = 0; ctr < size_of_buffer; ctr++)
        {
          sum_var += buffman[ctr][index];  // add up all maps
        }

        sum_var = map[index]/size_of_buffer; // convert to unity gain
        if (sum_var > costmap_2d::LETHAL_OBSTACLE) // threshold in case of goofy runoff
        {
          map[index] = costmap_2d::LETHAL_OBSTACLE;
        }
        else
        {
          map[index] = sum_var;
        }
      }
    }
  }

  void LowPassLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
  }
}
