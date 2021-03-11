#ifndef GRID_LAYER_H_
#define GRID_LAYER_H_
#include <ros/ros.h>
#include <layered_costmap.h>
#include <layer.h>

namespace roborts_costmap
{
 
class GridLayer : public roborts_costmap::Layer, public roborts_costmap::Costmap2D
{
public:
 GridLayer() {}
  virtual ~GridLayer() {}
  bool is_current_;
  bool is_enabled_;
  virtual void OnInitialize();
  virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void UpdateCosts(roborts_costmap::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void MatchSize();
};
}
#endif

