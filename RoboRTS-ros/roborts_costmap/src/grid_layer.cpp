#include <grid_layer.h>
using roborts_costmap::LETHAL_OBSTACLE;
using roborts_costmap::NO_INFORMATION;
using roborts_costmap::FREE_SPACE;
namespace roborts_costmap
{
 
unsigned flag = 0;
 
void GridLayer::OnInitialize()
{
  ros::NodeHandle nh("~/" + name_);

  is_current_= true;
  is_enabled_=true;
  default_value_ = NO_INFORMATION;
  MatchSize();
}
//
void GridLayer::MatchSize()
{
  Costmap2D* master = layered_costmap_->GetCostMap();
  ResizeMap(master->GetSizeXCell(), master->GetSizeXCell(), master->GetResolution(),
            master->GetOriginX(), master->GetOriginY());
}
 

void GridLayer::UpdateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!is_enabled_)
    return;
  
  if (flag == 0)
  {
	  flag = 1;
  }else
	return;
 
  double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
  unsigned int mx;
  unsigned int my;
  if(World2Map(mark_x, mark_y, mx, my)){
	       SetCost(mx, my,LETHAL_OBSTACLE);
  }
  
  *min_x = std::min(*min_x, mark_x);
  *min_y = std::min(*min_y, mark_y);
  *max_x = std::max(*max_x, mark_x);
  *max_y = std::max(*max_y, mark_y);
}
 
void GridLayer::UpdateCosts(roborts_costmap::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!is_enabled_)
    return;
 
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = GetIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.SetCost(i, j, costmap_[index]); 
    }
  }
}
 
} // end namespace

