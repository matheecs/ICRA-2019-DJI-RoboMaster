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

#include "local_planner/robot_position_cost.h"

namespace roborts_local_planner {

RobotPositionCost::RobotPositionCost(const roborts_costmap::Costmap2D& cost_map) : costmap_(cost_map) {

}

RobotPositionCost::~RobotPositionCost() {

}

double RobotPositionCost::PointCost(int x, int y) {
  unsigned char cost = costmap_.GetCost(x, y);
  if(cost == LETHAL_OBSTACLE || cost == NO_INFORMATION){
    return -1;
  }

  return cost;
}

double RobotPositionCost::LineCost(int x0, int x1, int y0, int y1) {
  double line_cost = 0.0;
  double point_cost = -1.0;

  for(FastLineIterator line( x0, y0, x1, y1 ); line.IsValid(); line.Advance()) {
    point_cost = PointCost(line.GetX(), line.GetY()); //current point's cost

    if(point_cost < 0){
      return -1;
    }

    if(line_cost < point_cost){
      line_cost = point_cost;
    }

  }
  return line_cost;
}

double RobotPositionCost::FootprintCost (const Eigen::Vector2d& position, const std::vector<Eigen::Vector2d>& footprint) {

  unsigned int cell_x, cell_y;

  if (!costmap_.World2Map(position.coeffRef(0), position.coeffRef(1), cell_x, cell_y)) {
    return -1.0;
  }

  if (footprint.size() < 3) {
    unsigned char cost = costmap_.GetCost(cell_x, cell_y);

    if (cost == LETHAL_OBSTACLE  || cost == NO_INFORMATION /*|| cost == INSCRIBED_INFLATED_OBSTACLE*/) {
      return -1.0;
    }
    return cost;
  }

  unsigned int x0, x1, y0, y1;
  double line_cost = 0.0;
  double footprint_cost = 0.0;

  for(unsigned int i = 0; i < footprint.size() - 1; ++i) {
    if (!costmap_.World2Map(footprint[i].coeffRef(0), footprint[i].coeffRef(1), x0, y0)) {
      return -1.0;
    }

    if(!costmap_.World2Map(footprint[i + 1].coeffRef(0), footprint[i + 1].coeffRef(1), x1, y1)) {
      return -1.0;
    }

    line_cost = LineCost(x0, x1, y0, y1);
    footprint_cost = std::max(line_cost, footprint_cost);

    if(line_cost < 0) {
      return -1.0;
    }
  }

  if(!costmap_.World2Map(footprint.back().coeffRef(0), footprint.back().coeffRef(1), x0, y0)) {
    return -1.0;
  }

  if(!costmap_.World2Map(footprint.front().coeffRef(0), footprint.front().coeffRef(1), x1, y1)) {
    return -1.0;
  }

  line_cost = LineCost(x0, x1, y0, y1);
  footprint_cost = std::max(line_cost, footprint_cost);

  if(line_cost < 0) {
    return -1.0;
  }

  return footprint_cost;
}

double RobotPositionCost::FootprintCost(double x,
                                        double y,
                                        double theta,
                                        const std::vector<Eigen::Vector2d> &footprint_spec,
                                        double inscribed_radius,
                                        double circumscribed_radius) {
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  std::vector<Eigen::Vector2d> oriented_footprint;
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    Eigen::Vector2d new_pt;
    new_pt.coeffRef(0) = x + (footprint_spec[i].coeffRef(0) * cos_th - footprint_spec[i].coeffRef(1) * sin_th);
    new_pt.coeffRef(1) = y + (footprint_spec[i].coeffRef(0) * sin_th + footprint_spec[i].coeffRef(1) * cos_th);
    oriented_footprint.push_back(new_pt);
  }

  Eigen::Vector2d robot_position;
  robot_position.coeffRef(0) = x;
  robot_position.coeffRef(1) = y;

  if(inscribed_radius==0.0){
    CalculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);
  }

  return FootprintCost(robot_position, oriented_footprint);
}
int RobotPositionCost::FootprintCostOrientation(double x, double y, double theta,
            const std::vector<Eigen::Vector2d>& footprint_spec, int check_size)
{
  double cos_th = cos(theta);
  double sin_th = sin(theta); 
  double tan_th = sin_th/cos_th;

  // std::vector<Eigen::Vector2d> oriented_footprint;
  // for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
  //   Eigen::Vector2d new_pt;
  //   new_pt.coeffRef(0) = x + (footprint_spec[i].coeffRef(0) * cos_th - footprint_spec[i].coeffRef(1) * sin_th);
  //   new_pt.coeffRef(1) = y + (footprint_spec[i].coeffRef(0) * sin_th + footprint_spec[i].coeffRef(1) * cos_th);
  //   oriented_footprint.push_back(new_pt);
  // }
  
  unsigned int mx, my;
  if (!costmap_.World2Map(x, y, mx, my)) {
    return -1.0;
  }
  // ROS_INFO("%d print!",footprint_spec.size());
  
  if (footprint_spec.size() == 4)
  {
    unsigned char cost = costmap_.GetCost(mx, my);
    // -0.340000, -0.280000
    // -0.340000, 0.280000
    // 0.340000, 0.280000
    // 0.340000, -0.280000
    float resolution = costmap_.GetResolution();
    
    int check_left, check_right, check_up, check_down;
    check_left = floor(((footprint_spec[1].coeffRef(1) + footprint_spec[2].coeffRef(1))/resolution + 0.5)/2) ;
    check_right = floor((-(footprint_spec[0].coeffRef(1) + footprint_spec[3].coeffRef(1))/resolution + 0.5)/2) ;
    check_up = floor(((footprint_spec[2].coeffRef(0) + footprint_spec[3].coeffRef(0))/resolution + 0.5)/2) ;
    check_down = floor((-(footprint_spec[0].coeffRef(0) + footprint_spec[1].coeffRef(0))/resolution + 0.5)/2) ;
    
    int check_y_init, check_x_init, check_y_end, check_x_end, check_y_init_mid, check_y_end_mid;
    bool left = false, right = false, up = false, down = false;
    //ROS_INFO("pose: %d, %d, %f", x, y, theta);

    //left
    check_y_init = my + check_left;
    check_x_init = mx - (check_down+check_size);
    check_y_end = my + (check_left+check_size);
    check_x_end = mx + (check_up+check_size);
    std::cout<<"check left:" << mx<<" "<<my<<" ("<<check_x_init<<","<<check_y_init<<
            ") ("<<check_x_end<<","<<check_y_end<<")"<<std::endl;
    
    for (int i=check_x_init; i<check_x_end; i++)
    {
      //ROS_INFO("check_x_init, check_x_end %d, %d", check_x_init,check_x_end);
      for (int j=check_y_init; j<check_y_end; j++)
      {
       //ROS_INFO("check_y_init_mid, check_y_end_mid %d, %d", check_y_init_mid,check_y_end_mid); 
        if(i>0 && j>0 && i<costmap_.GetSizeXCell() && j<costmap_.GetSizeYCell()  
                    && costmap_.GetCost(i,j)>INSCRIBED_INFLATED_OBSTACLE){
          left = true;
          break;
        }
      }
      if(left)
        break;
    }

    //right
    check_y_init = my - (check_right+check_size);
    check_x_init = mx - (check_up+check_size);
    check_y_end = my - check_right;
    check_x_end = mx + (check_down+check_size);
    std::cout<<"check right:" << mx<<" "<<my<<" ("<<check_x_init<<","<<check_y_init<<
            ") ("<<check_x_end<<","<<check_y_end<<")"<<std::endl;
    for (int i=check_x_init; i<check_x_end; i++)
    {
      //ROS_INFO("check_x_init, check_x_end %d, %d", check_x_init,check_x_end);
      for (int j=check_y_init; j<check_y_end; j++)
      {
       //ROS_INFO("check_y_init_mid, check_y_end_mid %d, %d", check_y_init_mid,check_y_end_mid); 
        if(i>0 && j>0 && i<costmap_.GetSizeXCell() && j<costmap_.GetSizeYCell()  
                    && costmap_.GetCost(i,j)>INSCRIBED_INFLATED_OBSTACLE){
          right = true;
          break;
        }
      }
      if(right)
        break;
    }

    //up
    check_y_init = my - (check_right+check_size);
    check_x_init = mx + check_up;
    check_y_end = my + (check_left+check_size);
    check_x_end = mx + (check_up+check_size);
    std::cout<<"check up:" << mx<<" "<<my<<" ("<<check_x_init<<","<<check_y_init<<
            ") ("<<check_x_end<<","<<check_y_end<<")"<<std::endl;
    for (int i=check_x_init; i<check_x_end; i++)
    {
      //ROS_INFO("check_x_init, check_x_end %d, %d", check_x_init,check_x_end);
      for (int j=check_y_init; j<check_y_end; j++)
      {
       //ROS_INFO("check_y_init_mid, check_y_end_mid %d, %d", check_y_init_mid,check_y_end_mid); 
        if(i>0 && j>0 && i<costmap_.GetSizeXCell() && j<costmap_.GetSizeYCell()  
                    && costmap_.GetCost(i,j)>INSCRIBED_INFLATED_OBSTACLE){
          up = true;
          break;
        }
      }
      if(up)
        break;
    }

    //down
    check_y_init = my - (check_right+check_size);
    check_x_init = mx - (check_down+check_size);
    check_y_end = my + (check_left+check_size);
    check_x_end = mx - check_down;
    std::cout<<"check down:" << mx<<" "<<my<<" ("<<check_x_init<<","<<check_y_init<<
            ") ("<<check_x_end<<","<<check_y_end<<")"<<std::endl;
    
    for (int i=check_x_init; i<check_x_end; i++)
    {
      //ROS_INFO("check_x_init, check_x_end %d, %d", check_x_init,check_x_end);
      for (int j=check_y_init; j<check_y_end; j++)
      {
       //ROS_INFO("check_y_init_mid, check_y_end_mid %d, %d", check_y_init_mid,check_y_end_mid); 
        if(i>0 && j>0 && i<costmap_.GetSizeXCell() && j<costmap_.GetSizeYCell()  
                    && costmap_.GetCost(i,j)>INSCRIBED_INFLATED_OBSTACLE){
          down = true;
          break;
        }
      }
      if(down)
        break;
    }

    // // left
    // check_y_init = my - check_left * cos_th + (check_up+check_size) * sin_th ;
    // check_y_init_mid = floor(check_y_init);
    // check_x_init = floor( mx - check_left * sin_th - (check_up+check_size) * cos_th);
    // check_y_end = floor( check_y_init - check_size / cos_th );
    // check_y_end_mid = check_y_end;
    // check_x_end = floor( mx - check_left * sin_th + (check_up+check_size) * cos_th );
    
    // for (int i=check_x_init; i<check_x_end; i++)
    // {
    //   //ROS_INFO("check_x_init, check_x_end %d, %d", check_x_init,check_x_end);
    //   for (int j=check_y_init_mid; j>check_y_end_mid; j--)
    //   {
    //    //ROS_INFO("check_y_init_mid, check_y_end_mid %d, %d", check_y_init_mid,check_y_end_mid); 
    //     if(i>0 && j>0 && i<costmap_.GetSizeXCell() && j<costmap_.GetSizeYCell()  
    //                 && costmap_.GetCost(i,j)>INSCRIBED_INFLATED_OBSTACLE){
    //       left = true;
    //       break;
    //     }
    //   }
    //   check_y_init_mid -= tan_th;
    //   check_y_end_mid -= tan_th;
    //   if(left)
    //     break;
    // }

    // // right
    // check_y_init = my + check_left * cos_th + (check_up+check_size) * sin_th;
    // check_y_init_mid = ceil( check_y_init );
    // check_x_init = floor( mx + check_left * sin_th - (check_up+check_size) * cos_th );
    // check_y_end = ceil( check_y_init - check_size / cos_th );
    // check_y_end_mid = check_y_end;
    // check_x_end = floor( mx + check_left * sin_th + (check_up+check_size) * cos_th);
    // for (int i=check_x_init; i<check_x_end; i++)
    // {
    //   for (int j=check_y_init_mid; j>check_y_end_mid; j--)
    //   {
    //     if(i>0 && j>0 && i<costmap_.GetSizeXCell() && j<costmap_.GetSizeYCell()  
    //                 && costmap_.GetCost(i,j)>INSCRIBED_INFLATED_OBSTACLE){
    //       right = true;
    //       break;
    //     }
    //   }
    //   check_y_init_mid -= tan_th;
    //   check_y_end_mid -= tan_th;
    //   if(right)
    //     break;
    // }

    // // up
    // check_y_init = my - check_left * cos_th - check_up * sin_th;
    // check_y_init_mid = ceil( check_y_init );
    // check_x_init = ceil( mx - check_left * sin_th + check_up * cos_th );
    // check_y_end = ceil( check_y_init - check_size / sin_th );
    // check_y_end_mid = check_y_end;
    // check_x_end = ceil( mx + check_left * sin_th + (check_up+check_size) * cos_th);
    // for (int i=check_x_init; i<check_x_end; i++)
    // {
    //   for (int j=check_y_init_mid; j>check_y_end_mid; j--)
    //   {
    //     if(i>0 && j>0 && i<costmap_.GetSizeXCell() && j<costmap_.GetSizeYCell()  
    //                 && costmap_.GetCost(i,j)>INSCRIBED_INFLATED_OBSTACLE){
    //       up = true;
    //       break;
    //     }
    //   }
    //   check_y_init_mid += 1/tan_th;
    //   check_y_end_mid += 1/tan_th;
    //   if(up)
    //     break;
    // }
    // // down
    // check_y_init = my - check_left * cos_th + (check_up+check_size) * sin_th;
    // check_y_init_mid = ceil( check_y_init );
    // check_x_init = floor( mx - check_left * sin_th - (check_up+check_size) * cos_th );
    // check_y_end = ceil( check_y_init - check_size / sin_th );
    // check_y_end_mid = check_y_end;
    // check_x_end = floor( mx + check_left * sin_th - check_up * cos_th);
    // for (int i=check_x_init; i<check_x_end; i++)
    // {
    //   for (int j=check_y_init_mid; j>check_y_end_mid; j--)
    //   {
    //     if(i>0 && j>0 && i<costmap_.GetSizeXCell() && j<costmap_.GetSizeYCell()  
    //                 && costmap_.GetCost(i,j)>INSCRIBED_INFLATED_OBSTACLE){
    //       down = true;
    //       break;
    //     }
    //   }
    //   check_y_init_mid += 1/tan_th;
    //   check_y_end_mid += 1/tan_th;
    //   if(down)
    //     break;
    // }

    return (int(left<<3)+int(right<<2)+int(up<<1)+int(down));

  }
  //非四足迹，不考虑
  return -1;
}

} // namespace roborts_local_planner

