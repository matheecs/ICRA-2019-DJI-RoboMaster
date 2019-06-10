/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "lazy_theta_star_planner.h"

namespace roborts_global_planner{

using roborts_common::ErrorCode;
using roborts_common::ErrorInfo;

LazyThetaStarPlanner::LazyThetaStarPlanner(CostmapPtr costmap_ptr) :
    GlobalPlannerBase::GlobalPlannerBase(costmap_ptr),
    gridmap_width_(costmap_ptr_->GetCostMap()->GetSizeXCell()),
    gridmap_height_(costmap_ptr_->GetCostMap()->GetSizeYCell()),
    cost_(costmap_ptr_->GetCostMap()->GetCharMap()) {

  LazyThetaStarPlannerConfig lazy_theta_star_planner_config;
  std::string full_path = ros::package::getPath("roborts_planning") + "/global_planner/lazy_theta_star_planner/"\
      "config/lazy_theta_star_planner_config.prototxt";

  if (!roborts_common::ReadProtoFromTextFile(full_path.c_str(),
                                           &lazy_theta_star_planner_config)) {
    ROS_ERROR("Cannot load lazy theta star planner protobuf configuration file.");
  }
  //  LazyThetaStarPlanner param config
  cost_time_ = lazy_theta_star_planner_config.cost_time();
  heuristic_factor_ = lazy_theta_star_planner_config.heuristic_factor();
  inaccessible_cost_ = lazy_theta_star_planner_config.inaccessible_cost();
  largest_distance_up_ = lazy_theta_star_planner_config.largest_distance_up();
  largest_distance_down_ = lazy_theta_star_planner_config.largest_distance_down();
  goal_distance_ = lazy_theta_star_planner_config.goal_distance();
  lazy_theta_inaccessible_cost_ = lazy_theta_star_planner_config.lazy_theta_inaccessible_cost();
  goal_search_tolerance_ = lazy_theta_star_planner_config.goal_search_tolerance()/costmap_ptr->GetCostMap()->GetResolution();
  sam_thre_ = lazy_theta_star_planner_config.sample_threshold();
}

LazyThetaStarPlanner::~LazyThetaStarPlanner(){
  cost_ = nullptr;
}

ErrorInfo LazyThetaStarPlanner::Plan(const geometry_msgs::PoseStamped &start,
                                     const geometry_msgs::PoseStamped &goal,
                                     std::vector<geometry_msgs::PoseStamped> &path) {

  unsigned int start_x, start_y, goal_x, goal_y, tmp_goal_x, tmp_goal_y;
  unsigned int valid_goal[2];
  unsigned  int shortest_dist = std::numeric_limits<unsigned int>::max();
  bool goal_valid = false;

  if (!costmap_ptr_->GetCostMap()->World2Map(start.pose.position.x,
                                             start.pose.position.y,
                                             start_x,
                                             start_y)) {
    ROS_WARN("Failed to transform start pose from map frame to costmap frame");
    return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                     "Start pose can't be transformed to costmap frame.");
  }
  if (!costmap_ptr_->GetCostMap()->World2Map(goal.pose.position.x,
                                             goal.pose.position.y,
                                             goal_x,
                                             goal_y)) {
    ROS_WARN("Failed to transform goal pose from map frame to costmap frame");
    return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                     "Goal pose can't be transformed to costmap frame.");
  }
  if (costmap_ptr_->GetCostMap()->GetCost(goal_x,goal_y)<inaccessible_cost_){
    valid_goal[0] = goal_x;
    valid_goal[1] = goal_y;
    goal_valid = true;
  }else{
    tmp_goal_x = goal_x;
    tmp_goal_y = goal_y - goal_search_tolerance_;

    while(tmp_goal_y <= goal_y + goal_search_tolerance_){
      tmp_goal_x = goal_x - goal_search_tolerance_;
      while(tmp_goal_x <= goal_x + goal_search_tolerance_){
        unsigned char cost = costmap_ptr_->GetCostMap()->GetCost(tmp_goal_x, tmp_goal_y);
        unsigned int dist = abs(goal_x - tmp_goal_x) + abs(goal_y - tmp_goal_y);
        if (cost < inaccessible_cost_ && dist < shortest_dist ) {
          shortest_dist = dist;
          valid_goal[0] = tmp_goal_x;
          valid_goal[1] = tmp_goal_y;
          goal_valid = true;
        }
        tmp_goal_x += 1;
      }
      tmp_goal_y += 1;
    }
  }
  ErrorInfo error_info;
  if (!goal_valid){
    error_info=ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR);
    path.clear();
  }
  else{
    unsigned int start_index, goal_index;
    start_index = costmap_ptr_->GetCostMap()->GetIndex(start_x, start_y);
    goal_index = costmap_ptr_->GetCostMap()->GetIndex(valid_goal[0], valid_goal[1]);

    costmap_ptr_->GetCostMap()->SetCost(start_x, start_y,roborts_costmap::FREE_SPACE);

    if(start_index == goal_index){
      error_info=ErrorInfo::OK();
      path.clear();
      path.push_back(start);
      path.push_back(goal);
    }
    else{
      error_info = SearchPath(start_index, goal_index, path);
      if ( error_info.IsOK() ){
        path.back().pose.orientation = goal.pose.orientation;
        path.back().pose.position.z = goal.pose.position.z;
      }
    }

  }


  return error_info;
}

bool LazyThetaStarPlanner::LargerDistance(const int &index1, const int &index2, const int &goal_index)
{
  unsigned int mx1, my1, mx2, my2, mgx, mgy;
  costmap_ptr_->GetCostMap()->Index2Cells(index1, mx1, my1);
  costmap_ptr_->GetCostMap()->Index2Cells(index2, mx2, my2);
  costmap_ptr_->GetCostMap()->Index2Cells(goal_index, mgx, mgy);
  unsigned int distance = sqrt( (mx1-mx2)*(mx1-mx2)+(my1-my2)*(my1-my2));
  unsigned int curr_goal_distance = sqrt( (mx2-mgx)*(mx2-mgx)+(my2-mgy)*(my2-mgy));
  if(curr_goal_distance > goal_distance_ && distance > largest_distance_up_)
    return true;
  else if(curr_goal_distance > goal_distance_ && distance <= largest_distance_up_)
    return false;
  else if(curr_goal_distance <= goal_distance_ && distance > largest_distance_down_)
    return true;
  else if(curr_goal_distance <= goal_distance_ && distance <= largest_distance_down_)
    return false;
  
}

bool LazyThetaStarPlanner::checkLineofSight(const int &index1, const int &index2)
{
  unsigned int mx1, my1, mx2, my2;
  costmap_ptr_->GetCostMap()->Index2Cells(index1, mx1, my1);
  costmap_ptr_->GetCostMap()->Index2Cells(index2, mx2, my2);

  int diff_x = mx2 - mx1, diff_y = my2 - my1;
  int dir_x, dir_y; //dirction of movement
  int offset_x, offset_y; //offset of movement
  if ( diff_y < 0) {
    diff_y = -diff_y;
    dir_y = -1; // North
    offset_y = 0;
  } else {
    dir_y = 1; // South
    offset_y = 1;
  }
  
  if ( diff_x < 0) {
    diff_x = -diff_x;
    dir_x = -1; // West
    offset_x = 0;
  } else {
    dir_x = 1; // East
    offset_x = 1;
  }

  int k = 0;
  if ( diff_x >= diff_y)
  { //move along the x axis & increment/decrement y when k >= diff_x (斜率)
    while (mx1 != mx2)
    {
      k += diff_y;
      if ( k >= diff_x)
      {
        if (costmap_ptr_->GetCostMap()->GetCost(mx1+offset_x, my1+offset_y) > lazy_theta_inaccessible_cost_)
          return false;

        k -= diff_x;
        my1 += dir_y;
      }

      if (k!=0 && costmap_ptr_->GetCostMap()->GetCost(mx1+offset_x, my1+offset_y) > lazy_theta_inaccessible_cost_)
          return false;

      // moving along a horizontal line, either the north or the south cell should be unblocked.
      if (diff_y == 0 
          && (costmap_ptr_->GetCostMap()->GetCost(mx1+offset_x, my1) > lazy_theta_inaccessible_cost_ 
          || costmap_ptr_->GetCostMap()->GetCost(mx1+offset_x, my1+1) > lazy_theta_inaccessible_cost_ ))
          return false;

      mx1 += dir_x;
    }
  }
  else
  { //move along the y axis & increment/decrement x when k >= diff_y (斜率)
    while (mx1 != mx2)
    {
      k += diff_x;
      if ( k >= diff_y)
      {
        if (costmap_ptr_->GetCostMap()->GetCost(mx1+offset_x, my1+offset_y) > lazy_theta_inaccessible_cost_)
          return false;

        k -= diff_y;
        mx1 += dir_x;
      }

      if (k!=0 && costmap_ptr_->GetCostMap()->GetCost(mx1+offset_x, my1+offset_y) > lazy_theta_inaccessible_cost_)
          return false;

      // moving along a vertical line, either the west or the east cell should be unblocked.
      if (diff_y == 0 
          && (costmap_ptr_->GetCostMap()->GetCost(mx1, my1+offset_y) > lazy_theta_inaccessible_cost_ 
          || costmap_ptr_->GetCostMap()->GetCost(mx1+1, my1+offset_y) > lazy_theta_inaccessible_cost_ ))
          return false;

      my1 += dir_y;
    }
  }

  return true;
}

bool cmp(pickmin a,pickmin b)
{
	return a.curr_g<b.curr_g;
}

ErrorInfo LazyThetaStarPlanner::SearchPath(const int &start_index,
                                           const int &goal_index,
                                           std::vector<geometry_msgs::PoseStamped> &path) {

  g_score_.clear();
  f_score_.clear();
  parent_.clear();
  state_.clear();
  gridmap_width_ = costmap_ptr_->GetCostMap()->GetSizeXCell();
  gridmap_height_ = costmap_ptr_->GetCostMap()->GetSizeYCell();
  ROS_INFO("Search in a map %d", gridmap_width_*gridmap_height_);
  cost_ = costmap_ptr_->GetCostMap()->GetCharMap();
  g_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
  f_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
  parent_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
  state_.resize(gridmap_height_ * gridmap_width_, SearchState::NOT_HANDLED);

  std::priority_queue<int, std::vector<int>, Compare> openlist;
  g_score_.at(start_index) = 0;
  //parent_.at(start_index) = start_index;
  openlist.push(start_index);

  std::vector<int> neighbors_index;
  int current_index, move_cost, h_score, count = 0;

  /*for(int i=0; i<gridmap_width_*gridmap_height_; i++){
    if (cost_[i] >= inaccessible_cost_){
      cost_[i] *= cost_time_;
    }
  }*/
  
  while (!openlist.empty()) {

    current_index = openlist.top();
    openlist.pop();
    state_.at(current_index) = SearchState::CLOSED;

    // if there is not a line of sight or 
    // the distance between this point and its parent is larger than a threshold,
    // change the g_score and its parent


    if (current_index == goal_index) {
      ROS_INFO("Search takes %d cycle counts", count);
      break;
    }

    GetNineNeighbors(current_index, neighbors_index);

    for (auto neighbor_index : neighbors_index) 
    {
      if (neighbor_index < 0 ||
          neighbor_index >= gridmap_height_ * gridmap_width_) {
        continue;
      }

      if (//cost_[neighbor_index] >= inaccessible_cost_ ||
          state_.at(neighbor_index) == SearchState::CLOSED) {
        continue;
      }
      
      if(current_index != start_index && parent_.at(current_index) != NULL &&
         !LargerDistance(current_index, parent_.at(current_index), goal_index) &&
         checkLineofSight(parent_.at(current_index), neighbor_index))
      {
        GetMoveCost(parent_.at(current_index), neighbor_index, move_cost);
        if(g_score_.at(neighbor_index) > g_score_.at(parent_.at(current_index)) + move_cost + cost_[neighbor_index])
        {
          g_score_.at(neighbor_index) =  g_score_.at(parent_.at(current_index)) + move_cost + cost_[neighbor_index];
          parent_.at(neighbor_index) = parent_.at(current_index);
        }
      }

      else
      {
        GetMoveCost(current_index, neighbor_index, move_cost);
        if (g_score_.at(neighbor_index) > g_score_.at(current_index) + move_cost + cost_[neighbor_index]) {

          g_score_.at(neighbor_index) = g_score_.at(current_index) + move_cost + cost_[neighbor_index];
          parent_.at(neighbor_index) = current_index;

          
        }
      }

      if (state_.at(neighbor_index) == SearchState::NOT_HANDLED) {
            GetManhattanDistance(neighbor_index, goal_index, h_score, false);
            f_score_.at(neighbor_index) = g_score_.at(neighbor_index) + h_score;
            openlist.push(neighbor_index);
            state_.at(neighbor_index) = SearchState::OPEN;
          }
    }
    count++;

/* lazy!!! */

//     GetNineNeighbors(current_index, neighbors_index);
//     if ( current_index != start_index &&
//          LargerDistance(current_index, parent_.at(current_index), goal_index) 
//          || !checkLineofSight(current_index, parent_.at(current_index)) )
//     {
//       std::vector<pickmin> neighbor_closed;
      
//       for (auto neighbor_index : neighbors_index) {
//         if (neighbor_index < 0 ||  neighbor_index >= gridmap_height_ * gridmap_width_) 
//           continue;
//         // if (cost_[neighbor_index] >= inaccessible_cost_ )
//         //   continue;
//         if (state_.at(neighbor_index) == SearchState::CLOSED ) {
//           GetMoveCost(current_index, neighbor_index, move_cost);
//           pickmin tmp;
//           tmp.curr_g = g_score_.at(neighbor_index) + move_cost;
//           tmp.index = neighbor_index;
//           neighbor_closed.push_back(tmp);
//         } 
//       }
      
//       if(neighbor_closed.size() > 0){
//         parent_.at(current_index) = max_element(std::begin(neighbor_closed),
//                                                  std::end(neighbor_closed), cmp)->index;
//         g_score_.at(current_index) = max_element(std::begin(neighbor_closed),
//                                                  std::end(neighbor_closed), cmp)->curr_g;
//       }
      
//     }

//     if (current_index == goal_index) {
//       ROS_INFO("Search takes %d cycle counts", count);
//       break;
//     }

//     for (auto neighbor_index : neighbors_index) {

//       if (neighbor_index < 0 ||
//           neighbor_index >= gridmap_height_ * gridmap_width_) {
//         continue;
//       }

//       if (//cost_[neighbor_index] >= inaccessible_cost_ ||
//           state_.at(neighbor_index) == SearchState::CLOSED) {
//         continue;
//       }

//       /*
//       GetMoveCost(current_index, neighbor_index, move_cost);      
//       if (g_score_.at(neighbor_index) > g_score_.at(current_index) + move_cost + cost_[neighbor_index]) 
//       {
//         g_score_.at(neighbor_index) = g_score_.at(current_index) + move_cost + cost_[neighbor_index];
//         parent_.at(neighbor_index) = current_index;

//         if (state_.at(neighbor_index) == SearchState::NOT_HANDLED) {
//           GetManhattanDistance(neighbor_index, goal_index, h_score);
//           f_score_.at(neighbor_index) = g_score_.at(neighbor_index) + h_score;
//           openlist.push(neighbor_index);
//           state_.at(neighbor_index) = SearchState::OPEN;
//         }
//       }
//       */
//      int last_parent = parent_.at(current_index); //相对于目前的邻居节点，当前节点的父亲节点算是过去节点的父亲
//      GetManhattanDistance(last_parent, neighbor_index, move_cost, false);
//      if (g_score_.at(neighbor_index) > g_score_.at(last_parent) + move_cost + cost_[neighbor_index])
//      {
//        g_score_.at(neighbor_index) = g_score_.at(last_parent) + move_cost + cost_[neighbor_index];
//        parent_.at(neighbor_index) = last_parent;

//        if (state_.at(neighbor_index) == SearchState::NOT_HANDLED) {
//           GetManhattanDistance(neighbor_index, goal_index, h_score, true);
//           f_score_.at(neighbor_index) = g_score_.at(neighbor_index) + h_score;
//           openlist.push(neighbor_index);
//           state_.at(neighbor_index) = SearchState::OPEN;
//         }
//      }
//     }
//     count++;
  }

  if (current_index != goal_index) {
    ROS_WARN("Global planner can't search the valid path!");
    return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR, "Valid global path not found.");
  }

  unsigned int iter_index = current_index, iter_x, iter_y;

  geometry_msgs::PoseStamped iter_pos;
  iter_pos.pose.orientation.w = 1;
  iter_pos.header.frame_id = "map";
  path.clear();
  costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
  costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
  path.push_back(iter_pos);

  while (iter_index != start_index) {
    iter_index = parent_.at(iter_index);
//    if(cost_[iter_index]>= inaccessible_cost_){
//      LOG_INFO<<"Cost changes through planning for"<< static_cast<unsigned int>(cost_[iter_index]);
//    }
    costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
    costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
    
    // if the point is less than the sample threshold, we sample
    geometry_msgs::PoseStamped last_path = path.back();
    float sam_dis = sqrt((iter_pos.pose.position.x-last_path.pose.position.x) *
               (iter_pos.pose.position.x-last_path.pose.position.x) +
               (iter_pos.pose.position.y-last_path.pose.position.y) *
               (iter_pos.pose.position.y-last_path.pose.position.y));
    //std::cout<< "sam_dis" << sam_dis<< std::endl;
    if ( sam_dis > sam_thre_)
    {
      int sam_num = sam_dis/sam_thre_;
      //std::cout<<"sam_num"<<sam_num<<std::endl;
      float sam_x_inter = (iter_pos.pose.position.x-last_path.pose.position.x) / sam_num;
      float sam_y_inter = (iter_pos.pose.position.y-last_path.pose.position.y) / sam_num;
      float x_start = last_path.pose.position.x;
      float y_start = last_path.pose.position.y;
      geometry_msgs::PoseStamped inter_path;
      while(sam_num > 1)
      {
        sam_num --;
        x_start += sam_x_inter;
        y_start += sam_y_inter;
        inter_path.pose.position.x = x_start;
        inter_path.pose.position.y = y_start;
        path.push_back(inter_path);
        //std::cout<< "inter_path" << inter_path<< std::endl;
      }
    }

    path.push_back(iter_pos);
  }
  //std::cout<<"path size"<< path.size()<<std::endl;

  std::reverse(path.begin(),path.end());

  return ErrorInfo(ErrorCode::OK);

}

ErrorInfo LazyThetaStarPlanner::GetMoveCost(const int &current_index,
                                    const int &neighbor_index,
                                    int &move_cost) const {
  if (abs(neighbor_index - current_index) == 1 ||
      abs(neighbor_index - current_index) == gridmap_width_) {
    move_cost = 10;
  } else if (abs(neighbor_index - current_index) == (gridmap_width_ + 1) ||
      abs(neighbor_index - current_index) == (gridmap_width_ - 1)) {
    move_cost = 14;
  } else {
    return ErrorInfo(ErrorCode::GP_MOVE_COST_ERROR,
                     "Move cost can't be calculated cause current neighbor index is not accessible");
  }
  return ErrorInfo(ErrorCode::OK);
}

void LazyThetaStarPlanner::GetManhattanDistance(const int &index1, const int &index2, int &manhattan_distance, bool use_cost_time) {
  manhattan_distance = heuristic_factor_* 10 * (abs(index1 / gridmap_width_ - index2 / gridmap_width_) +
      abs(index1 % gridmap_width_ - index2 % gridmap_width_));
  if(use_cost_time && checkLineofSight(index1, index2)){
    manhattan_distance *= cost_time_;
  }
}

void LazyThetaStarPlanner::GetNineNeighbors(const int &current_index, std::vector<int> &neighbors_index) const {
  neighbors_index.clear();
  if(current_index - gridmap_width_ >= 0){
    neighbors_index.push_back(current_index - gridmap_width_);       //up
  }
  if(current_index - gridmap_width_ - 1 >= 0 && (current_index - gridmap_width_ - 1 + 1) % gridmap_width_!= 0){
    neighbors_index.push_back(current_index - gridmap_width_ - 1); //left_up
  }
  if(current_index - 1 >= 0 && (current_index - 1 + 1) % gridmap_width_!= 0){
    neighbors_index.push_back(current_index - 1);        //left
  }
  if(current_index + gridmap_width_ - 1 < gridmap_width_* gridmap_height_
      && (current_index + gridmap_width_ - 1 + 1) % gridmap_width_!= 0){
    neighbors_index.push_back(current_index + gridmap_width_ - 1); //left_down
  }
  if(current_index + gridmap_width_ < gridmap_width_* gridmap_height_){
    neighbors_index.push_back(current_index + gridmap_width_);     //down
  }
  if(current_index + gridmap_width_ + 1 < gridmap_width_* gridmap_height_
      && (current_index + gridmap_width_ + 1 ) % gridmap_width_!= 0){
    neighbors_index.push_back(current_index + gridmap_width_ + 1); //right_down
  }
  if(current_index  + 1 < gridmap_width_* gridmap_height_
      && (current_index  + 1 ) % gridmap_width_!= 0) {
    neighbors_index.push_back(current_index + 1);                   //right
  }
  if(current_index - gridmap_width_ + 1 >= 0
      && (current_index - gridmap_width_ + 1 ) % gridmap_width_!= 0) {
    neighbors_index.push_back(current_index - gridmap_width_ + 1); //right_up
  }
}

} //namespace roborts_global_planner
