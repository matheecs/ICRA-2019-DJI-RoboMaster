/**
 * caution zone layer
 * 2019.1.28
 **/
#ifndef ROBORTS_COSTMAP_CAUTION_LAYER_H
#define ROBORTS_COSTMAP_CAUTION_LAYER_H

#include <nav_msgs/OccupancyGrid.h>
#include "io/io.h"
#include "map_common.h"
#include "costmap_layer.h"

namespace roborts_costmap {

class CautionLayer : public CostmapLayer {

 public:
  CautionLayer() {}
  virtual ~CautionLayer() {}
  virtual void OnInitialize();
  virtual void changeInitPose(tf::Stamped<tf::Pose> const & initial_pose);
  virtual void Activate();
  virtual void Deactivate();
  //参数min_i, min_j, max_i, max_j均为map grid下的
  virtual void UpdateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  //参数min_i, min_j, max_i, max_j均为world下的
  virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void MatchSize();

 private:
  double caution_center_x_, caution_center_y_, caution_width_, caution_height_;
  double caution_min_x_, caution_min_y_, caution_max_x_, caution_max_y_;
  double mark_x_, mark_y_;
  unsigned char caution_cost_value_;
  bool is_world_;
  std::string global_frame_;
  float initial_pose_x_;
  float initial_pose_y_;
  float initial_pose_a_;
};


} // namespace roborts_costmap
#endif
