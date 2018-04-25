#include "mapping_filter/hole_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(hole_layer::HoleLayer, costmap_2d::Layer)

namespace hole_layer
{

  void HoleLayer::onInitialize()
  {
    LowPassLayer::onInitialize();
    ros::NodeHandle nh;
    int cost;
    if(nh.getParam ("hole_cost",cost))
    {
        if (cost < 255) {
            hole_cost = cost;
            ROS_INFO ("hole cost : %i ", hole_cost);
        }  else  {
            hole_cost = 255;
        }
    }
    else {
        ROS_INFO ("Couldn't find param");
        hole_cost = 128;
    }
  }



  void HoleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                               double *min_y, double *max_x, double *max_y)
  {
      if (rolling_window_)
      {
          updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
      }
      if (!enabled_)
      {
          return;
      }
      useExtraBounds(min_x, min_y, max_x, max_y);

      bool current = true;
      std::vector<costmap_2d::Observation> observations, clearing_observations;

      // get the marking observations
      current = current && getMarkingObservations(observations);

      // get the clearing observations
      current = current && getClearingObservations(clearing_observations);

      // update the global current status
      current_ = current;

      // raytrace freespace
      for (unsigned int i = 0; i < clearing_observations.size(); ++i)
      {
          raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
      }

      // place the new obstacles into a priority queue... each with a priority of zero to begin with
      for (std::vector<costmap_2d::Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
      {
          const costmap_2d::Observation& obs = *it;

          const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

          double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

          for (unsigned int i = 0; i < cloud.points.size(); ++i)
          {
              double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

              // if the obstacle is too high or too far away from the robot we won't add it
              if (pz > max_obstacle_height_)
              {
                  ROS_DEBUG("The point is too high");
                  continue;
              }

              // compute the squared distance from the hitpoint to the pointcloud's origin
              double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
                               + (pz - obs.origin_.z) * (pz - obs.origin_.z);

              // if the point is far enough away... we won't consider it
              if (sq_dist >= sq_obstacle_range)
              {
                  ROS_DEBUG("The point is too far away");
                  continue;
              }

              // now we need to compute the map coordinates for the observation
              unsigned int mx, my;
              if (!worldToMap(px, py, mx, my))
              {
                  ROS_DEBUG("Computing map coords failed");
                  continue;
              }

              unsigned int index = getIndex(mx, my);
              costmap_[index] = hole_cost;
              touch(px, py, min_x, min_y, max_x, max_y);
          }
      }

      updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  }


}