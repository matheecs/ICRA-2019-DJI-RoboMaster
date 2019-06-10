/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <local_planner/map_grid.h>

namespace roborts_local_planner{
MapGrid::MapGrid(): size_x_(0), size_y_(0) {}

MapGrid::MapGrid(unsigned int size_x, unsigned int size_y):
                 size_x_(size_x), size_y_(size_y)
                 {
                     commonInit();
                 }
MapGrid::MapGrid(const MapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
}

void MapGrid::commonInit(){
    //don't allow construction of zero size grid
    ROS_ASSERT(size_y_ != 0 && size_x_ != 0);

    map_.resize(size_y_ * size_x_);
    //make each cell aware of its location in the grid
    for(unsigned int i = 0; i < size_y_; ++i){
      for(unsigned int j = 0; j < size_x_; ++j){
        unsigned int id = size_x_ * i + j;
        map_[id].cx = j;
        map_[id].cy = i;
      }
    }
}

MapGrid& MapGrid::operator= (const MapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
    return *this;
}

void MapGrid::sizeCheck(unsigned int size_x, unsigned int size_y){
    if(map_.size() != size_x * size_y)
      map_.resize(size_x * size_y);

    if(size_x_ != size_x || size_y_ != size_y){
      size_x_ = size_x;
      size_y_ = size_y;

      for(unsigned int i = 0; i < size_y_; ++i){
        for(unsigned int j = 0; j < size_x_; ++j){
          int index = size_x_ * i + j;
          map_[index].cx = j;
          map_[index].cy = i;
        }
      }
    }
}

void MapGrid::resetPathDist(){
    for(unsigned int i = 0; i < map_.size(); ++i) {
      map_[i].target_dist = unreachableCellCosts();
      map_[i].target_mark = false;
      map_[i].within_robot = false;
    }
}

inline bool MapGrid::updatePathCell(MapCell* current_cell, MapCell* check_cell,
                                    const roborts_costmap::Costmap2D* costmap){

    //if the cell is an obstacle set the max path distance
    unsigned char cost = costmap->GetCost(check_cell->cx, check_cell->cy);
    if(! getCell(check_cell->cx, check_cell->cy).within_robot &&
        (cost == roborts_costmap::LETHAL_OBSTACLE ||
         cost == roborts_costmap::INSCRIBED_INFLATED_OBSTACLE ||
         cost == roborts_costmap::NO_INFORMATION)){
      check_cell->target_dist = obstacleCosts();
      return false;
    }

    double new_target_dist = current_cell->target_dist + 1;
    if (new_target_dist < check_cell->target_dist) {
      check_cell->target_dist = new_target_dist;
    }
    return true;
} 

void MapGrid::adjustPlanResolution(const TrajectoryDataBase& global_plan_in,
                                    TrajectoryDataBase& global_plan_out, 
                                    double resolution)
{
  if (global_plan_in.path_.size() == 0) {
    return;
  }
  double last_x = global_plan_in.path_[0].GetPosition().coeffRef(0);
  double last_y = global_plan_in.path_[0].GetPosition().coeffRef(1);
  global_plan_out.path_.push_back(global_plan_in.path_[0]);
  
  // we can take "holes" in the plan smaller than 2 grid cells (squared = 4)
  double min_sq_resolution = resolution * resolution * 4;

  for (unsigned int i = 1; i < global_plan_in.path_.size(); ++i) {
    double loop_x = global_plan_in.path_[i].GetPosition().coeffRef(0);
    double loop_y = global_plan_in.path_[i].GetPosition().coeffRef(1);
    double sqdist = (loop_x - last_x) * (loop_x - last_x) + (loop_y - last_y) * (loop_y - last_y);
    if (sqdist > min_sq_resolution) {
      int steps = ((sqrt(sqdist) - sqrt(min_sq_resolution)) / resolution) - 1;
      
      // add a points in-between
      double deltax = (loop_x - last_x) / steps;
      double deltay = (loop_y - last_y) / steps;
      // TODO: Interpolate orientation
      for (int j = 1; j < steps; ++j) {
          DataBase pose;
          Eigen::Vector2d position(last_x + j * deltax, last_y + j * deltay);
          pose.SetPosition(position);
          pose.SetTheta( global_plan_in.path_[i].GetTheta());
          global_plan_out.path_.push_back(pose);
        }
      }
    global_plan_out.path_.push_back(global_plan_in.path_[i]);
    last_x = loop_x;
    last_y = loop_y;
  }
}

void MapGrid::computeTargetDistance(std::queue<MapCell*>& dist_queue, const roborts_costmap::Costmap2D* costmap)
{
  MapCell* current_cell;
  MapCell* check_cell;
  unsigned int last_col = size_x_ - 1;
  unsigned int last_row = size_y_ - 1;  
  while(!dist_queue.empty()){
    current_cell = dist_queue.front();

    dist_queue.pop();

    if(current_cell->cx > 0){
      check_cell = current_cell - 1; //指针 指向前一个cell
      if(!check_cell->target_mark){
        //mark the cell as visisted
        check_cell->target_mark = true;
        if(updatePathCell(current_cell, check_cell, costmap)) {
          dist_queue.push(check_cell);
        }
      }
    }

    if(current_cell->cx < last_col){
      check_cell = current_cell + 1;
      if(!check_cell->target_mark){
        check_cell->target_mark = true;
        if(updatePathCell(current_cell, check_cell, costmap)) {
          dist_queue.push(check_cell);
        }
      }
    }

    if(current_cell->cy > 0){
      check_cell = current_cell - size_x_;
      if(!check_cell->target_mark){
        check_cell->target_mark = true;
        if(updatePathCell(current_cell, check_cell, costmap)) {
          dist_queue.push(check_cell);
        }
      }
    }

    if(current_cell->cy < last_row){
      check_cell = current_cell + size_x_;
      if(!check_cell->target_mark){
        check_cell->target_mark = true;
        if(updatePathCell(current_cell, check_cell, costmap)) {
          dist_queue.push(check_cell);
        }
      }
    }
  }

}

void MapGrid::setTargetCells(const roborts_costmap::Costmap2D* costmap,
                             const TrajectoryDataBase& global_plan) {
  sizeCheck(costmap->GetSizeXCell(), costmap->GetSizeYCell());

  bool started_path = false;
  
  std::queue<MapCell*> path_dist_queue;
  TrajectoryDataBase adjusted_global_plan;
  adjustPlanResolution(global_plan, adjusted_global_plan, costmap->GetResolution());
  if (adjusted_global_plan.path_.size() != global_plan.path_.size()) {
    ROS_DEBUG("Adjusted global plan resolution, added %zu points", adjusted_global_plan.path_.size() - global_plan.path_.size());
  }
  
  unsigned int i;
  // put global path points into local map until we reach the border of the local map
  for (i = 0; i < adjusted_global_plan.path_.size(); ++i) {
    double g_x = adjusted_global_plan.path_[i].GetPosition().coeffRef(0);
    double g_y = adjusted_global_plan.path_[i].GetPosition().coeffRef(1);
    unsigned int map_x, map_y;
    if (costmap->World2Map(g_x, g_y, map_x, map_y) && 
        costmap->GetCost(map_x, map_y) != roborts_costmap::NO_INFORMATION) {
      MapCell& current = getCell(map_x, map_y);
      current.target_dist = 0.0;
      current.target_mark = true;
      path_dist_queue.push(&current);
      started_path = true;
    } else if (started_path) {
        break;
      }
    }
  if (!started_path) {
    ROS_ERROR("None of the %d first of %zu (%zu) points of the global plan were in the local costmap and free",
       i, adjusted_global_plan.path_.size(), global_plan.path_.size());
    return;
  }
  computeTargetDistance(path_dist_queue, costmap);
}

void MapGrid::setLocalGoal(const roborts_costmap::Costmap2D* costmap,
                           const TrajectoryDataBase& global_plan) {
  sizeCheck(costmap->GetSizeXCell(), costmap->GetSizeYCell());

  int local_goal_x = -1;
  int local_goal_y = -1;
  bool started_path = false;

  TrajectoryDataBase adjusted_global_plan;
  adjustPlanResolution(global_plan, adjusted_global_plan, costmap->GetResolution());

  // skip global path points until we reach the border of the local map
  for (unsigned int i = 0; i < adjusted_global_plan.path_.size(); ++i) {
    double g_x = adjusted_global_plan.path_[i].GetPosition().coeffRef(0);
    double g_y = adjusted_global_plan.path_[i].GetPosition().coeffRef(1);
    unsigned int map_x, map_y;
    if (costmap->World2Map(g_x, g_y, map_x, map_y) && 
         costmap->GetCost(map_x, map_y) != roborts_costmap::NO_INFORMATION) {
      local_goal_x = map_x;
      local_goal_y = map_y;
      started_path = true;
    } else {
      if (started_path) {
        break;
      }// else we might have a non pruned path, so we just continue
    }
  }
  if (!started_path) {
    ROS_ERROR("None of the points of the global plan were in the local costmap, global plan points too far from robot");
      return;
  }

  std::queue<MapCell*> path_dist_queue;
  if (local_goal_x >= 0 && local_goal_y >= 0) {
    MapCell& current = getCell(local_goal_x, local_goal_y);
    costmap->Map2World(local_goal_x, local_goal_y, goal_x_, goal_y_);
    current.target_dist = 0.0;
    current.target_mark = true;
    path_dist_queue.push(&current);
  }
  computeTargetDistance(path_dist_queue, costmap);
}

}