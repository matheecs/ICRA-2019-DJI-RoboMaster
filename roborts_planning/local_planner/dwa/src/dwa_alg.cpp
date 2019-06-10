/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <dwa/dwa_alg.h>


namespace roborts_local_planner {
DWAAlg::DWAAlg(std::vector<Eigen::Vector2d>& footprint_spec,
               std::shared_ptr<roborts_local_planner::RobotPositionCost> robot_cost,
               roborts_costmap::Costmap2D* costmap, double robot_inscribed_radius, 
               double robot_circumscribed_radius, double max_vel_x, double max_vel_y, 
               double max_vel_theta, double min_vel_x, double min_vel_y,
               double min_vel_theta, double acc_lim_x, double acc_lim_y, 
               double acc_lim_theta, double min_in_place_vel_th, 
               double backup_vel, double heading_lookahead, bool dwa, bool heading_scoring,
               bool simple_attractor, bool holonomic_robot, double vx_samples, 
               double vy_samples, double vtheta_samples, double sim_time, 
               double sim_granularity, double angular_sim_granularity,
               double heading_scoring_timestep, double pdist_scale, double gdist_scale, 
               double occdist_scale, double sim_period, double escape_reset_dist, 
               double escape_reset_theta, double oscillation_reset_dist) :
               footprint_spec_(footprint_spec), robot_cost_(robot_cost),
               costmap_(costmap), robot_inscribed_radius_(robot_inscribed_radius), 
               robot_circumscribed_radius_(robot_circumscribed_radius), 
               max_vel_x_(max_vel_x), max_vel_y_(max_vel_y), 
               max_vel_theta_(max_vel_theta), min_vel_x_(min_vel_x), 
               min_vel_y_(min_vel_y), min_vel_theta_(min_vel_theta), 
               acc_lim_x_(acc_lim_x), acc_lim_y_(acc_lim_y), acc_lim_theta_(acc_lim_theta),
               min_in_place_vel_th_(min_in_place_vel_th), backup_vel_(backup_vel),
               heading_lookahead_(heading_lookahead), dwa_(dwa), 
               heading_scoring_(heading_scoring), simple_attractor_(simple_attractor),
               holonomic_robot_(holonomic_robot), vx_samples_(vx_samples), 
               vy_samples_(vy_samples), vtheta_samples_(vtheta_samples), sim_time_(sim_time), 
               sim_granularity_(sim_granularity), angular_sim_granularity_(angular_sim_granularity),
               heading_scoring_timestep_(heading_scoring_timestep), pdist_scale_(pdist_scale), 
               gdist_scale_(gdist_scale), occdist_scale_(occdist_scale), sim_period_(sim_period), 
               escape_reset_dist_(escape_reset_dist), escape_reset_theta_(escape_reset_theta),
               oscillation_reset_dist_(oscillation_reset_dist)
{
    //the robot is not stuck to begin with
    stuck_left_ = false;
    stuck_right_ = false;
    stuck_left_strafe_ = false;
    stuck_right_strafe_ = false;
    rotating_left_ = false;
    rotating_right_ = false;
    strafe_left_ = false;
    strafe_right_ = false;

    escaping_ = false;
}

void DWAAlg::updatePlan(const TrajectoryDataBase& new_plan, bool compute_dists){
    global_plan_.path_.resize(new_plan.path_.size());
    for(unsigned int i = 0; i < new_plan.path_.size(); ++i){
        global_plan_.path_[i] = new_plan.path_[i];
    }

    if (compute_dists) {
      //reset the map for new operations
      path_map_.resetPathDist();
      goal_map_.resetPathDist();

      //make sure that we update our path based on the global plan and compute costs
      path_map_.setTargetCells(costmap_, global_plan_);
      goal_map_.setLocalGoal(costmap_, global_plan_);
      ROS_DEBUG("Path/Goal distance computed");
    }
}

double DWAAlg::headingDiff(int cell_x, int cell_y, double x, double y, double heading){
    double heading_diff = DBL_MAX;
    unsigned int goal_cell_x, goal_cell_y;
    //find a clear line of sight from the robot's cell to a point on the path
    for (int i = global_plan_.path_.size() - 1; i >=0; --i) {
      if (costmap_->World2Map(global_plan_.path_[i].GetPosition().coeffRef(0), 
                             global_plan_.path_[i].GetPosition().coeffRef(1), goal_cell_x, goal_cell_y)) {
        if (robot_cost_->LineCost(cell_x, goal_cell_x, cell_y, goal_cell_y) >= 0) {
          double gx, gy;
          costmap_->Map2World(goal_cell_x, goal_cell_y, gx, gy);
          double v1_x = gx - x;
          double v1_y = gy - y;
          double v2_x = cos(heading);
          double v2_y = sin(heading);

          double perp_dot = v1_x * v2_y - v1_y * v2_x;
          double dot = v1_x * v2_x + v1_y * v2_y;

          //get the signed angle
          double vector_angle = atan2(perp_dot, dot);

          heading_diff = fabs(vector_angle);
          return heading_diff;
        }
      }
    }
    return heading_diff;
}

bool DWAAlg::generateTrajectory(double x, double y, double theta, double vx, 
                                double vy, double vtheta, double vx_samp, 
                                double vy_samp, double vtheta_samp, double acc_x, 
                                double acc_y, double acc_theta, 
                                double impossible_cost, TrajectoryDataBase& traj){

    // make sure the configuration doesn't change mid run
    boost::mutex::scoped_lock l(configuration_mutex_);

    double x_i = x;
    double y_i = y;
    double theta_i = theta;

    double vx_i = vx, vy_i = vy, vtheta_i = vtheta;

    //compute the magnitude of the velocities
    double vmag = sqrt(vx_samp * vx_samp + vy_samp * vy_samp);
    //compute the number of steps we must take along this trajectory to be "safe"
    int num_steps;
    if(!heading_scoring_) {
      num_steps = int(std::max( (vmag * sim_time_) / sim_granularity_, 
                           fabs(vtheta_samp) / angular_sim_granularity_) + 0.5);
    } else {
      num_steps = int(sim_time_ / sim_granularity_ + 0.5);
    }

    //we at least want to take one step... even if we won't move, we want to score our current position
    if(num_steps == 0) {
      num_steps = 1;
    }

    double dt = sim_time_ / num_steps;
    double time = 0.0;

    //create a potential trajectory
    traj.path_.clear();
    traj.xv_ = vx_samp; 
    traj.yv_ = vy_samp; 
    traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;

    //initialize the costs for the trajectory
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;
    double heading_diff = 0.0;

    for(int i = 0; i < num_steps; ++i){
      //get map coordinates of a point
      unsigned int cell_x, cell_y;

      //we don't want a path that goes off the know map
      if(!costmap_->World2Map(x_i, y_i, cell_x, cell_y)){
        traj.cost_ = -1.0;
        ROS_INFO("unknown map");
        return false;
      }

      //check the point on the trajectory for legality
      double footprint_cost = robot_cost_->FootprintCost(x_i, y_i, theta_i, footprint_spec_, 
                                        robot_inscribed_radius_, robot_circumscribed_radius_);
      //if the footprint hits an obstacle this trajectory is invalid
      if(footprint_cost < 0){
        traj.cost_ = -1.0;
        //ROS_INFO("footprint_cost inlegal");
        return false;
      }

      occ_cost = std::max(std::max(occ_cost, footprint_cost), 
                          double(costmap_->GetCost(cell_x, cell_y)));

      //do we want to follow blindly
      if (simple_attractor_) {
        goal_dist = (x_i - global_plan_.path_[global_plan_.path_.size() -1].GetPosition().coeffRef(0)) * 
          (x_i - global_plan_.path_[global_plan_.path_.size() -1].GetPosition().coeffRef(0)) + 
          (y_i - global_plan_.path_[global_plan_.path_.size() -1].GetPosition().coeffRef(1)) * 
          (y_i - global_plan_.path_[global_plan_.path_.size() -1].GetPosition().coeffRef(1));
      } else {
        bool update_path_and_goal_distances = true;

        // with heading scoring, we take into account heading diff, and also only score
        // path and goal distance for one point of the trajectory
        if (heading_scoring_) {
          if (time >= heading_scoring_timestep_ && time < heading_scoring_timestep_ + dt) {
            heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);
          } else {
            update_path_and_goal_distances = false;
          }
        }

        if (update_path_and_goal_distances) {
          //update path and goal distances
          path_dist = path_map_(cell_x, cell_y).target_dist;
          goal_dist = goal_map_(cell_x, cell_y).target_dist;

          //if a point on this trajectory has no clear path to goal it is invalid
          if(impossible_cost <= goal_dist || impossible_cost <= path_dist){
            traj.cost_ = -2.0;
            //ROS_INFO("a point on this trajectory has no clear path to goal it is invalid");
            return false;
          }
        }
      }

      //the point is legal... add it to the trajectory
      Eigen::Vector2d tmppos(x_i, y_i);
      DataBase tmp(tmppos, theta_i);
      traj.path_.push_back(tmp);

      //calculate velocities
      vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
      vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
      vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

      //calculate positions
      x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
      y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
      theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

      //increment time
      time += dt;
    } // end for i < numsteps

    double cost = -1.0;
    if (!heading_scoring_) {
      cost = pdist_scale_ * path_dist + goal_dist * gdist_scale_ + occdist_scale_ * occ_cost;
    } else {
      cost = occdist_scale_ * occ_cost + pdist_scale_ * path_dist + 0.3 * heading_diff + goal_dist * gdist_scale_;
    }
    traj.cost_ = cost;
    
    return true;  
}

bool DWAAlg::createTrajectories(double x, double y, double theta, 
                            double vx, double vy, double vtheta, 
                            double acc_x, double acc_y, double acc_theta,
                            TrajectoryDataBase& best_path)
{
    //compute feasible velocity limits in robot space
    double max_vel_x, max_vel_y, max_vel_theta;
    double min_vel_x, min_vel_y, min_vel_theta;
    //should we use the dynamic window approach?
    if (dwa_) {
      max_vel_x = std::max(std::min(max_vel_x_, vx + acc_x * sim_period_), min_vel_x_);
      min_vel_x = std::max(min_vel_x_, vx - acc_x * sim_period_);

      max_vel_y = std::max(std::min(max_vel_y_, vy + acc_y * sim_period_), min_vel_y_);
      min_vel_y = std::max(min_vel_y_, vy - acc_y * sim_period_);

      max_vel_theta = std::min(max_vel_theta_, vtheta + acc_theta * sim_period_);
      min_vel_theta = std::max(min_vel_theta_, vtheta - acc_theta * sim_period_);
    } else {
      max_vel_x = std::max(std::min(max_vel_x_, vx + acc_x * sim_time_), min_vel_x_);
      min_vel_x = std::max(min_vel_x_, vx - acc_x * sim_time_);

      max_vel_y = std::max(std::min(max_vel_y_, vy + acc_y * sim_time_), min_vel_y_);
      min_vel_y = std::max(min_vel_y_, vy - acc_y * sim_time_);

      max_vel_theta = std::min(max_vel_theta_, vtheta + acc_theta * sim_time_);
      min_vel_theta = std::max(min_vel_theta_, vtheta - acc_theta * sim_time_);
    }
    //we want to sample the velocity space regularly
    double dvx = (max_vel_x - min_vel_x) / (vx_samples_ - 1);
    double dvy = (max_vel_y - min_vel_y) / (vy_samples_ - 1);
    double dvtheta = (max_vel_theta - min_vel_theta) / (vtheta_samples_ - 1);

    double vx_samp = min_vel_x;
    double vtheta_samp = min_vel_theta;
    double vy_samp = min_vel_y; //0.0;

    //keep track of the best trajectory seen so far
    TrajectoryDataBase* best_traj = &traj_one_;
    best_traj->cost_ = -1.0;

    TrajectoryDataBase* comp_traj = &traj_two_;
    comp_traj->cost_ = -1.0;

    TrajectoryDataBase* swap = NULL;

    //any cell with a cost greater than the size of the map is impossible
    double impossible_cost = path_map_.obstacleCosts();

    bool generate_succeed_x, generate_succeed_y,generate_succeed_theta, generate_succeed_back;
    //if we're performing an escape we won't allow moving forward
    if (!escaping_) {
      //loop through all x velocities
      ROS_INFO("!escaping!");
      for(int i = 0; i < vx_samples_; ++i) {
        vtheta_samp = 0;
        //first sample the straight trajectory   -> x
        generate_succeed_x = generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
                                acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
     
        //if the new trajectory is better... let's take it
        if(generate_succeed_x && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }

        vtheta_samp = min_vel_theta;
        //next sample all theta trajectories  -> theta
        for(int j = 0; j < vtheta_samples_ - 1; ++j){
          generate_succeed_theta = generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
                                           acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

          //if the new trajectory is better... let's take it
          if(generate_succeed_theta && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
            swap = best_traj;
            best_traj = comp_traj;
            comp_traj = swap;
          }
          vtheta_samp += dvtheta;
        }
        vx_samp += dvx;
      }
      if(holonomic_robot_)//only explore y velocities with holonomic robots
      {
        vx_samp = 0.1;
        vy_samp = 0.1;
        vtheta_samp = 0.0;
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
                                           acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        //if the new trajectory is better... let's take it
        if(generate_succeed_y && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }
        vx_samp = 0.1;
        vy_samp = -0.1;
        vtheta_samp = 0.0;
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
                                           acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
        if(generate_succeed_y && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }
      }
    }

    ROS_INFO("rotating!");

    //next we want to generate trajectories for rotating in place
    vtheta_samp = min_vel_theta;
    vx_samp = 0.0;
    vy_samp = 0.0;
    generate_succeed_theta = false;

    //let's try to rotate toward open space
    double heading_dist = DBL_MAX;

    for(int i = 0; i < vtheta_samples_; ++i) {
      //enforce a minimum rotational velocity because the base can't handle small in-place rotations
      double vtheta_samp_limited = vtheta_samp > 0 ? std::max(vtheta_samp, min_in_place_vel_th_) 
        : std::min(vtheta_samp, -1.0 * min_in_place_vel_th_);

      generate_succeed_theta = generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp_limited, 
                                         acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

      //if the new trajectory is better... let's take it... 
      //note if we can legally rotate in place we prefer to do that rather than move with y velocity
      if(generate_succeed_theta
          && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0 || best_traj->yv_ != 0.0) 
          && (vtheta_samp > dvtheta || vtheta_samp < -1 * dvtheta)){
        
        DataBase traj_back = comp_traj->path_.back();
        double x_r = traj_back.GetPosition().coeffRef(0), y_r = traj_back.GetPosition().coeffRef(1), th_r = traj_back.GetTheta();

        x_r += heading_lookahead_ * cos(th_r);
        y_r += heading_lookahead_ * sin(th_r);
        unsigned int cell_x, cell_y;

        //make sure that we'll be looking at a legal cell
        if (costmap_->World2Map(x_r, y_r, cell_x, cell_y)) {
          double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
          if (ahead_gdist < heading_dist) {
            //if we haven't already tried rotating left since we've moved forward
            if (vtheta_samp < 0 && !stuck_left_) {
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
            //if we haven't already tried rotating right since we've moved forward
            else if(vtheta_samp > 0 && !stuck_right_) {
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
          }
        }
      }
      vtheta_samp += dvtheta;
    }

    //do we have a legal trajectory
    if (best_traj->cost_ >= 0) {
      // avoid oscillations of in place rotation and in place strafing
      if ( ! (best_traj->xv_ > 0)) {
        if (best_traj->thetav_ < 0) {
          if (rotating_right_) {
            stuck_right_ = true;
          }
          rotating_left_ = true;
        } else if (best_traj->thetav_ > 0) {
          if (rotating_left_){
            stuck_left_ = true;
          }
          rotating_right_ = true;
        } else if(best_traj->yv_ > 0) {
          if (strafe_right_) {
            stuck_right_strafe_ = true;
          }
          strafe_left_ = true;
        } else if(best_traj->yv_ < 0){
          if (strafe_left_) {
            stuck_left_strafe_ = true;
          }
          strafe_right_ = true;
        }

        //set the position we must move a certain distance away from
        prev_x_ = x;
        prev_y_ = y;
      }

      double dist = sqrt((x - prev_x_) * (x - prev_x_) + (y - prev_y_) * (y - prev_y_));
      if (dist > oscillation_reset_dist_) {
        rotating_left_ = false;
        rotating_right_ = false;
        strafe_left_ = false;
        strafe_right_ = false;
        stuck_left_ = false;
        stuck_right_ = false;
        stuck_left_strafe_ = false;
        stuck_right_strafe_ = false;
      }

      dist = sqrt((x - escape_x_) * (x - escape_x_) + (y - escape_y_) * (y - escape_y_));
      if(dist > escape_reset_dist_ ||
          fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_){
        escaping_ = false;
      }

      best_path = *best_traj;
      return true;
    }

    //finally, if we can't do anything else, we want to generate trajectories that move backwards slowly
    vtheta_samp = 0.0;
    vx_samp = backup_vel_;
    vy_samp = 0.0;
    generate_succeed_back = generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
                                               acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

    if(generate_succeed_back)
    {
      //we'll allow moving backwards slowly even when the static map shows it as blocked
      swap = best_traj;
      best_traj = comp_traj;
      comp_traj = swap;

      double dist = sqrt((x - prev_x_) * (x - prev_x_) + (y - prev_y_) * (y - prev_y_));
      if (dist > oscillation_reset_dist_) {
        rotating_left_ = false;
        rotating_right_ = false;
        strafe_left_ = false;
        strafe_right_ = false;
        stuck_left_ = false;
        stuck_right_ = false;
        stuck_left_strafe_ = false;
        stuck_right_strafe_ = false;
      }

      //only enter escape mode when the planner has given a valid goal point
      if (!escaping_ && best_traj->cost_ > -2.0) {
        escape_x_ = x;
        escape_y_ = y;
        escape_theta_ = theta;
        escaping_ = true;
      }

      dist = sqrt((x - escape_x_) * (x - escape_x_) + (y - escape_y_) * (y - escape_y_));

      if (dist > escape_reset_dist_ ||
          fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_) {
        escaping_ = false;
      }


      //if the trajectory failed because the footprint hits something, we're still going to back up
      if(best_traj->cost_ == -1.0)
        best_traj->cost_ = 1.0;

      best_path = *best_traj;
      return true;
    }
    
    return false;
    /*
    //if we're performing an escape we won't allow moving forward
    if (!escaping_) {
      //loop through all x velocities
      for(int i = 0; i < vx_samples_; ++i) {
        //vtheta_samp = 0;
        //first sample the straight trajectory   -> x
        generate_succeed_x = generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
                                acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
     
        //if the new trajectory is better... let's take it
        if(generate_succeed_x && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }

        vtheta_samp = min_vel_theta;
        //next sample all theta trajectories  -> theta
        for(int j = 0; j < vtheta_samples_ - 1; ++j){
          generate_succeed_theta = generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
                                           acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

          //if the new trajectory is better... let's take it
          if(generate_succeed_theta && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
            swap = best_traj;
            best_traj = comp_traj;
            comp_traj = swap;
          }

          if(holonomic_robot_)//only explore y velocities with holonomic robots
          {
              for(int k = 0; k < vy_samples_ - 1; ++k){
                generate_succeed_y = generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
                                           acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

                //if the new trajectory is better... let's take it
                if(generate_succeed_y && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
                  swap = best_traj;
                  best_traj = comp_traj;
                  comp_traj = swap;
                }
                vy_samp += dvy;
              }
          }
          vtheta_samp += dvtheta;
        }
        vx_samp += dvx;
      }
    }

    //next we want to generate trajectories for rotating in place
    vtheta_samp = min_vel_theta;
    vx_samp = 0.0;
    vy_samp = 0.0;
    generate_succeed_theta = false;

    //let's try to rotate toward open space
    double heading_dist = DBL_MAX;

    for(int i = 0; i < vtheta_samples_; ++i) {
      //enforce a minimum rotational velocity because the base can't handle small in-place rotations
      double vtheta_samp_limited = vtheta_samp > 0 ? std::max(vtheta_samp, min_in_place_vel_th_) 
        : std::min(vtheta_samp, -1.0 * min_in_place_vel_th_);

      generate_succeed_theta = generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp_limited, 
                                         acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

      //if the new trajectory is better... let's take it... 
      //note if we can legally rotate in place we prefer to do that rather than move with y velocity
      if(generate_succeed_theta
          && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0 || best_traj->yv_ != 0.0) 
          && (vtheta_samp > dvtheta || vtheta_samp < -1 * dvtheta)){
        
        DataBase traj_back = comp_traj->path_.back();
        double x_r = traj_back.GetPosition().coeffRef(0), y_r = traj_back.GetPosition().coeffRef(1), th_r = traj_back.GetTheta();

        x_r += heading_lookahead_ * cos(th_r);
        y_r += heading_lookahead_ * sin(th_r);
        unsigned int cell_x, cell_y;

        //make sure that we'll be looking at a legal cell
        if (costmap_->World2Map(x_r, y_r, cell_x, cell_y)) {
          double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
          if (ahead_gdist < heading_dist) {
            //if we haven't already tried rotating left since we've moved forward
            if (vtheta_samp < 0 && !stuck_left_) {
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
            //if we haven't already tried rotating right since we've moved forward
            else if(vtheta_samp > 0 && !stuck_right_) {
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
          }
        }
      }
      vtheta_samp += dvtheta;
    }

    //do we have a legal trajectory
    if (best_traj->cost_ >= 0) {
      // avoid oscillations of in place rotation and in place strafing
      if ( ! (best_traj->xv_ > 0)) {
        if (best_traj->thetav_ < 0) {
          if (rotating_right_) {
            stuck_right_ = true;
          }
          rotating_left_ = true;
        } else if (best_traj->thetav_ > 0) {
          if (rotating_left_){
            stuck_left_ = true;
          }
          rotating_right_ = true;
        } else if(best_traj->yv_ > 0) {
          if (strafe_right_) {
            stuck_right_strafe_ = true;
          }
          strafe_left_ = true;
        } else if(best_traj->yv_ < 0){
          if (strafe_left_) {
            stuck_left_strafe_ = true;
          }
          strafe_right_ = true;
        }

        //set the position we must move a certain distance away from
        prev_x_ = x;
        prev_y_ = y;
      }

      double dist = sqrt((x - prev_x_) * (x - prev_x_) + (y - prev_y_) * (y - prev_y_));
      if (dist > oscillation_reset_dist_) {
        rotating_left_ = false;
        rotating_right_ = false;
        strafe_left_ = false;
        strafe_right_ = false;
        stuck_left_ = false;
        stuck_right_ = false;
        stuck_left_strafe_ = false;
        stuck_right_strafe_ = false;
      }

      dist = sqrt((x - escape_x_) * (x - escape_x_) + (y - escape_y_) * (y - escape_y_));
      if(dist > escape_reset_dist_ ||
          fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_){
        escaping_ = false;
      }

      best_path = *best_traj;
      return true;
    }

    //finally, if we can't do anything else, we want to generate trajectories that move backwards slowly
    vtheta_samp = 0.0;
    vx_samp = backup_vel_;
    vy_samp = 0.0;
    generate_succeed_back = generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
                                               acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

    if(generate_succeed_back)
    {
      //we'll allow moving backwards slowly even when the static map shows it as blocked
      swap = best_traj;
      best_traj = comp_traj;
      comp_traj = swap;

      double dist = sqrt((x - prev_x_) * (x - prev_x_) + (y - prev_y_) * (y - prev_y_));
      if (dist > oscillation_reset_dist_) {
        rotating_left_ = false;
        rotating_right_ = false;
        strafe_left_ = false;
        strafe_right_ = false;
        stuck_left_ = false;
        stuck_right_ = false;
        stuck_left_strafe_ = false;
        stuck_right_strafe_ = false;
      }

      //only enter escape mode when the planner has given a valid goal point
      if (!escaping_ && best_traj->cost_ > -2.0) {
        escape_x_ = x;
        escape_y_ = y;
        escape_theta_ = theta;
        escaping_ = true;
      }

      dist = sqrt((x - escape_x_) * (x - escape_x_) + (y - escape_y_) * (y - escape_y_));

      if (dist > escape_reset_dist_ ||
          fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_) {
        escaping_ = false;
      }


      //if the trajectory failed because the footprint hits something, we're still going to back up
      if(best_traj->cost_ == -1.0)
        best_traj->cost_ = 1.0;

      best_path = *best_traj;
      return true;
    }
    
    return false;   
    */

}

bool DWAAlg::checkTrajectory(double x, double y, double theta, double vx, double vy, 
                         double vtheta, double vx_samp, double vy_samp, double vtheta_samp){
    double cost = scoreTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp);

    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vx_samp, vy_samp, vtheta_samp, cost);

    //otherwise the check fails
    return false;
}

double DWAAlg::scoreTrajectory(double x, double y, double theta, double vx, double vy, 
                      double vtheta, double vx_samp, double vy_samp, double vtheta_samp) {
    TrajectoryDataBase t; 
    double impossible_cost = path_map_.obstacleCosts();
    generateTrajectory(x, y, theta, vx, vy, vtheta,
                       vx_samp, vy_samp, vtheta_samp,
                       acc_lim_x_, acc_lim_y_, acc_lim_theta_,
                       impossible_cost, t);

    return double( t.cost_ );
}

bool DWAAlg::findBestPath(const DataBase& global_pose, 
                     const geometry_msgs::Twist& global_vel,
                     TrajectoryDataBase& local_path,
                     geometry_msgs::Twist& drive_velocities)
{
    Eigen::Vector3f pos(global_pose.GetPosition().coeffRef(0), global_pose.GetPosition().coeffRef(0), global_pose.GetTheta());
    Eigen::Vector3f vel(global_vel.linear.x, global_vel.linear.y, global_vel.angular.z);

    //reset the map for new operations
    path_map_.resetPathDist();
    goal_map_.resetPathDist();

    //temporarily remove obstacles that are within the footprint of the robot
    std::vector<Eigen::Vector2d> footprint_list;
    double x_i = pos[0], y_i = pos[1], theta_i = pos[2];
    //if we have footprint... do nothing 
    if (footprint_spec_.size() <= 1) {
      unsigned int mx, my;
      if (costmap_->World2Map(x_i, y_i, mx, my)) {
        Eigen::Vector2d center(mx, my);
        footprint_list.push_back(center);
      }
    }

    //mark cells within the initial footprint of the robot
    for (unsigned int i = 0; i < footprint_list.size(); ++i) {
      path_map_(footprint_list[i].coeffRef(0), footprint_list[i].coeffRef(1)).within_robot = true;
    }

    //make sure that we update our path based on the global plan and compute costs
    path_map_.setTargetCells(costmap_, global_plan_);
    goal_map_.setLocalGoal(costmap_, global_plan_);
    ROS_DEBUG("Path/Goal distance computed");

    //rollout trajectories and find the minimum cost one
    bool succeed = false;
    succeed = createTrajectories(pos[0], pos[1], pos[2], vel[0], vel[1], vel[2],
                                 acc_lim_x_, acc_lim_y_, acc_lim_theta_, local_path);
    ROS_INFO("Trajectories created %d", local_path.path_.size());
    /*for(int i=0; i<local_path.path_.size(); i++)
    {
      ROS_INFO("best %f, %f, %f \n", local_path.path_[i].GetPosition().coeffRef(0), local_path.path_[i].GetPosition().coeffRef(1), local_path.path_[i].GetTheta());
    }*/

    if(local_path.cost_ < 0 || !succeed){
      //drive_velocities.setIdentity();
      ROS_INFO("Trajectories fail");
      drive_velocities.linear.x = 0;
      drive_velocities.linear.y = 0;
      drive_velocities.linear.z = 0;
      drive_velocities.angular.x = 0;
      drive_velocities.angular.y = 0;
      drive_velocities.angular.z = 0;
      return false;
    }
    else{
      drive_velocities.linear.x = local_path.xv_;
      drive_velocities.linear.y = local_path.yv_;
      drive_velocities.linear.z = 0;
      drive_velocities.angular.z = local_path.thetav_;
      drive_velocities.angular.y = 0;
      drive_velocities.angular.z = 0;
      ROS_INFO("best vel %f, %f, %f ", local_path.xv_, local_path.yv_, local_path.thetav_);
      //tf::Matrix3x3 matrix;
      //matrix.setRotation(tf::createQuaternionFromYaw(local_path.thetav_));
      //drive_velocities.setBasis(matrix);
      return true;
    }

  }
};
