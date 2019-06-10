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

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
    LOG_INFO << "Footprint model 'polygon' loaded";
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
 * Author: Christoph Rösmann
 *********************************************************************/

#include "timed_elastic_band/teb_local_planner.h"


namespace roborts_local_planner {

TebLocalPlanner::TebLocalPlanner () {

}

TebLocalPlanner::~TebLocalPlanner () {

}
void TebLocalPlanner::DirectComputeGlobalVelocity(roborts_msgs::TwistAccel &cmd_vel){
  if (transformed_plan_.empty() || transformed_plan_.size()==1) {
    cmd_vel.twist.linear.x = 0;
    cmd_vel.twist.linear.y = 0;
    cmd_vel.twist.angular.z = 0;

    cmd_vel.accel.linear.x = 0;
    cmd_vel.accel.linear.y = 0;
    cmd_vel.accel.angular.z = 0;
  }
  ROS_WARN("direct!");
  /*std::cout<<"1x:"<<transformed_plan_[1].GetPosition().coeffRef(0)<<" 1y:"<<transformed_plan_[1].GetPosition().coeffRef(1)<<std::endl;
std::cout<<"0x:"<<transformed_plan_[0].GetPosition().coeffRef(0)<<" 0y:"<<transformed_plan_[0].GetPosition().coeffRef(1)<<std::endl;*/
  Eigen::Vector2d posdist = transformed_plan_[1].GetPosition() - transformed_plan_[0].GetPosition();
  double xdist = posdist.coeffRef(0), ydist = posdist.coeffRef(1);
  double vmax = 0.7;
  cmd_vel.twist.linear.x = vmax/sqrt(1+ydist*ydist/(xdist*xdist)) * xdist/fabs(xdist);
  cmd_vel.twist.linear.y = vmax/sqrt(1+xdist*xdist/(ydist*ydist)) * ydist/fabs(ydist);
  cmd_vel.twist.angular.z = 0;

  cmd_vel.accel.linear.x = 0;
  cmd_vel.accel.linear.y = 0;
  cmd_vel.accel.angular.z = 0;
}
roborts_common::ErrorInfo TebLocalPlanner::ComputeVelocityCommands(roborts_msgs::TwistAccel &cmd_vel) {

  if (!is_initialized_) {
    ROS_ERROR("timed_elastic_band doesn't be initialized");
    roborts_common::ErrorInfo algorithm_init_error(roborts_common::LP_ALGORITHM_INITILIZATION_ERROR,
                                                 "teb initialize failed");
    ROS_ERROR("%s",algorithm_init_error.error_msg().c_str());
    return algorithm_init_error;
  }

  GetPlan(temp_plan_);

  cmd_vel.twist.linear.x = 0;
  cmd_vel.twist.linear.y = 0;
  cmd_vel.twist.angular.z = 0;

  cmd_vel.accel.linear.x = 0;
  cmd_vel.accel.linear.y = 0;
  cmd_vel.accel.angular.z = 0;


  UpdateRobotPose();
  UpdateRobotVel();
  UpdateGlobalToPlanTranform();

  auto time_now = std::chrono::system_clock::now();
  oscillation_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - oscillation_).count() / 1000.0f;

  if (oscillation_time_ > 1.0) {
    if ((robot_pose_.GetPosition() - last_robot_pose_.GetPosition()).norm() < 0.1) {
      local_cost_.lock()->ClearCostMap();
    } else {
      oscillation_time_ = 0;
      oscillation_ = std::chrono::system_clock::now();
      last_robot_pose_ = robot_pose_;
    }
  }

  if (IsGoalReached()) {
    roborts_common::ErrorInfo algorithm_ok(roborts_common::OK, "reached the goal");
    ROS_INFO("reached the goal");
    return algorithm_ok;
  }

  PruneGlobalPlan();

  int goal_idx;

  if (!TransformGlobalPlan(&goal_idx)) {
    roborts_common::ErrorInfo PlanTransformError(roborts_common::LP_PLANTRANSFORM_ERROR, "plan transform error");
    ROS_ERROR("%s", PlanTransformError.error_msg().c_str());
    return PlanTransformError;
  }

  if (transformed_plan_.empty()) {
    roborts_common::ErrorInfo PlanTransformError(roborts_common::LP_PLANTRANSFORM_ERROR, "transformed plan is empty");
    ROS_ERROR("transformed plan is empty");
    return PlanTransformError;
  }

  /*tf::Stamped<tf::Pose> goal_point;
  tf::poseStampedMsgToTF(transformed_plan_.poses.back(), goal_point);
  robot_goal_.GetPosition().coeffRef(0) = goal_point.getOrigin().getX();
  robot_goal_.GetPosition().coeffRef(1) = goal_point.getOrigin().getY();*/

  /*if (global_plan_.poses.size() - goal_idx < 5) {
    robot_goal_.GetTheta() = tf::getYaw(global_plan_.poses.back().pose.orientation);
    transformed_plan_.poses.back().pose.orientation = tf::createQuaternionMsgFromYaw(robot_goal_.GetTheta());
  } else*/ if (global_plan_overwrite_orientation_) {
    transformed_plan_.back().SetTheta(EstimateLocalGoalOrientation(transformed_plan_.back(), goal_idx));
  }

  if (transformed_plan_.size()==1) {// plan only contains the goal
    transformed_plan_.insert(transformed_plan_.begin(), robot_pose_); // insert start (not yet initialized)
  } else {
    transformed_plan_.front() = robot_pose_;// update start;
  }


  obst_vector_.clear();


  robot_goal_ = transformed_plan_.back();
//  while (obst_vector_.empty()) {
//    usleep(1);
    UpdateObstacleWithCostmap(robot_goal_.GetPosition());
//  }

  UpdateViaPointsContainer();

  bool micro_control = false;
  if (global_plan_.poses.back().pose.position.z == 1) {
    micro_control = true;
  }
  // distance from the first to end
  Eigen::Vector2d posdist = transformed_plan_.back().GetPosition() - transformed_plan_.front().GetPosition();
  double xdist = posdist.coeffRef(0), ydist = posdist.coeffRef(1);
  double dist = sqrt(xdist*xdist + ydist*ydist);
  bool max_acc_2min = false;
  if(dist < param_config_.kinematics_opt().dist_thre()){
    max_acc_2min = true;
    ROS_ERROR("change the max acc to the min %f", dist);
  }
  

  bool success = optimal_->Optimal(transformed_plan_, &robot_current_vel_, 
                                    free_goal_vel_, micro_control, max_acc_2min);

  if (!success) {
    optimal_->ClearPlanner();
    roborts_common::ErrorInfo OptimalError(roborts_common::LP_OPTIMAL_ERROR, "optimal error");
    ROS_ERROR("optimal error");
    last_cmd_ = cmd_vel;
    return OptimalError;
  }

  int orient_back = -1;
  bool feasible = optimal_->IsTrajectoryFeasible(teb_error_info_, robot_cost_.get(), robot_footprint_, 
                                                 robot_footcontours_, orient_back,
                                                 robot_inscribed_radius_, robot_circumscribed_radius,
                                                 fesiable_step_look_ahead_, check_back_size_ );
  if (!feasible) {
//    cmd_vel.twist.linear.x = 0;
//    cmd_vel.twist.linear.y = 0;
//    cmd_vel.twist.angular.z = 0;
//
//    cmd_vel.accel.linear.x = 0;
//    cmd_vel.accel.linear.y = 0;
//    cmd_vel.accel.angular.z = 0;

    optimal_->ClearPlanner();
    last_cmd_ = cmd_vel;
    ROS_INFO("trajectory is not feasible");
    // switch(orient_back){
    //   case 0: //no obtscale
    //     break;
    //   case 1://down
    //     cmd_vel.twist.linear.x = 0.4;
    //     cmd_vel.twist.linear.y = 0;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;
    //     break;
      
    //   case 2://up
    //     cmd_vel.twist.linear.x = -0.4;
    //     cmd_vel.twist.linear.y = 0;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;
    //     break;

    //   case 3://up-down
    //     cmd_vel.twist.linear.x = 0;
    //     cmd_vel.twist.linear.y = 0.6;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;
    //     break;

    //   case 4://right
    //     cmd_vel.twist.linear.x = 0;
    //     cmd_vel.twist.linear.y = 0.6;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;
    //     break;
    //   case 5://right-down
    //     cmd_vel.twist.linear.x = 0.4;
    //     cmd_vel.twist.linear.y = 0.6;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;
    //     break;
    //   case 6://right-up
    //     cmd_vel.twist.linear.x = -0.4;
    //     cmd_vel.twist.linear.y = 0.6;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;
    //     break;
    //   case 7://right-up-down
    //     cmd_vel.twist.linear.x = 0;
    //     cmd_vel.twist.linear.y = 0.6;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;
    //     break;
    //   case 8://left
    //     cmd_vel.twist.linear.x = 0;
    //     cmd_vel.twist.linear.y = -0.6;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;
    //     break;
    //   case 9://left-down
    //     cmd_vel.twist.linear.x = 0.4;
    //     cmd_vel.twist.linear.y = -0.6;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;
    //     break;
    //   case 10://left-on
    //     cmd_vel.twist.linear.x = -0.4;
    //     cmd_vel.twist.linear.y = -0.6;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;
    //     break;
    //   case 11://left-on-down
    //     cmd_vel.twist.linear.x = 0;
    //     cmd_vel.twist.linear.y = -0.6;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;
    //     break;
    //   case 12://left-right
    //     cmd_vel.twist.linear.x = 0.4;
    //     cmd_vel.twist.linear.y = 0;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;
    //     break;
    //   case 13://left-right-down
    //     cmd_vel.twist.linear.x = 0.4;
    //     cmd_vel.twist.linear.y = 0;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;
    //     break;
    //   case 14://left-right-on
    //     cmd_vel.twist.linear.x = -0.4;
    //     cmd_vel.twist.linear.y = 0;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;
    //     break;
    //   case 15://left-right-down-on
    //     cmd_vel.twist.linear.x = 0;
    //     cmd_vel.twist.linear.y = 0;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;
    //     break;

    //   default:
    //     cmd_vel.twist.linear.x = 0;
    //     cmd_vel.twist.linear.y = 0;
    //     cmd_vel.twist.angular.z = 0;

    //     cmd_vel.accel.linear.x = 0;
    //     cmd_vel.accel.linear.y = 0;
    //     cmd_vel.accel.angular.z = 0;

    // }
    
    
    //RobotGetBack(cmd_vel);
    roborts_common::ErrorInfo trajectory_error(roborts_common::LP_ALGORITHM_TRAJECTORY_ERROR, "trajectory is not feasible");
    return trajectory_error;
  }

  if (!optimal_->GetVelocity(teb_error_info_, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                             cmd_vel.accel.linear.x, cmd_vel.accel.linear.y, cmd_vel.accel.angular.z)) {
    optimal_->ClearPlanner();
    ROS_ERROR("can not get the velocity");
    roborts_common::ErrorInfo velocity_error(roborts_common::LP_VELOCITY_ERROR, "velocity is not feasible");
    last_cmd_ = cmd_vel;
    return velocity_error;
  }

  bool need_reduce_speed;
  need_reduce_speed = optimal_->TestVelocity(robot_cost_.get(), robot_footprint_,
                                            robot_inscribed_radius_, robot_circumscribed_radius,
                                            fesiable_step_look_ahead_, reduce_step_look_ahead_);
  if(need_reduce_speed){
    cmd_vel.twist.linear.x *= 0.9;
    cmd_vel.twist.linear.y *= 0.9;
    // cmd_vel.twist.angular.z *= 1; // same as last
    cmd_vel.accel.linear.x = 0;
    cmd_vel.accel.linear.y = 0;
    cmd_vel.accel.angular.z = 0;
  }
  
  // else if(sqrt(cmd_vel.twist.linear.x*cmd_vel.twist.linear.x + cmd_vel.twist.linear.y * cmd_vel.twist.linear.y) < 1.2)
  // {
  //     cmd_vel.twist.linear.x *= 1.2;
  //     cmd_vel.twist.linear.y *= 1.2;
  //     cmd_vel.twist.angular.z *= 1.2;
  // }
  float tmp_vel_x = max_vel_x_, tep_vel_y = max_vel_y_, tmp_vel_theta = max_vel_theta_, tmp_vel_x_backwards = max_vel_x_backwards;
  /*if(max_acc_2min){
    max_vel_x_          = param_config_.kinematics_opt().max_vel_x_min();
    max_vel_x_backwards = param_config_.kinematics_opt().max_vel_x_backwards_min();
    max_vel_y_          = param_config_.kinematics_opt().max_vel_y_min();
    float v_trans = sqrt(cmd_vel.twist.linear.x*cmd_vel.twist.linear.x + cmd_vel.twist.linear.y*cmd_vel.twist.linear.y);
    if(v_trans < 0.16)
      max_vel_theta_      = param_config_.kinematics_opt().max_vel_theta();
    else
      max_vel_theta_      = param_config_.kinematics_opt().max_vel_theta_min();
  }*/
  SaturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                   max_vel_x_,max_vel_y_,max_vel_theta_,max_vel_x_backwards);
  /*if(max_acc_2min)
  {
    max_vel_x_ = tmp_vel_x;
    max_vel_x_backwards = tmp_vel_x_backwards;
    max_vel_y_ = tep_vel_y;
    max_vel_theta_ = tmp_vel_theta;
  }*/
  
  last_cmd_ = cmd_vel;
  ROS_ERROR("final speed: %f", sqrt(cmd_vel.twist.linear.x*cmd_vel.twist.linear.x+cmd_vel.twist.linear.y*cmd_vel.twist.linear.y));
//  cmd_vel.linear.x = 0;
//  cmd_vel.linear.y = 0;
//  cmd_vel.angular.z = 0;

  optimal_->Visualize();

  ROS_INFO("compute velocity succeed");
  return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
}

bool TebLocalPlanner::IsGoalReached () {

  tf::Stamped<tf::Pose> global_goal;
  tf::poseStampedMsgToTF(global_plan_.poses.back(), global_goal);
  global_goal.setData( plan_to_global_transform_ * global_goal );
  auto goal = DataConverter::LocalConvertTFData(global_goal);

  auto distance = (goal.first - robot_pose_.GetPosition()).norm();
  double delta_orient = g2o::normalize_theta( goal.second - robot_pose_.GetTheta());

  if (distance < xy_goal_tolerance_
      && fabs(delta_orient) < yaw_goal_tolerance_) {
    ROS_INFO("goal reached");
    return true;
  } else {
    return false;
  }

}

bool TebLocalPlanner::SetPlan(const nav_msgs::Path& plan, const geometry_msgs::PoseStamped& goal) {
  if (plan_mutex_.try_lock()) {
    ROS_INFO("set plan");
    if (plan.poses.empty()) {
      temp_plan_.poses.push_back(goal);
    } else {
      temp_plan_ = plan;
    }
    plan_mutex_.unlock();
  }
}

bool TebLocalPlanner::SetMaxVel(float maxvelx, float maxvely){
  max_vel_x_ = maxvelx;
  max_vel_y_ = maxvely;
  ROS_INFO("max x %f, %f", max_vel_x_, max_vel_y_);
}

bool TebLocalPlanner::GetPlan(const nav_msgs::Path& plan) {
  if (plan_mutex_.try_lock()) {
    global_plan_ = plan;
    plan_mutex_.unlock();
  }
}

bool TebLocalPlanner::PruneGlobalPlan() {
  if (global_plan_.poses.empty()) {
    return true;
  }
  try {
    tf::StampedTransform global_to_plan_transform;
    tf_.lock()->lookupTransform(global_plan_.poses.front().header.frame_id,
                                robot_tf_pose_.frame_id_, ros::Time(0),
                                global_to_plan_transform);
    tf::Stamped<tf::Pose> robot;
    robot.setData( global_to_plan_transform * robot_tf_pose_ );

    Eigen::Vector2d robot_to_goal(robot.getOrigin().x() - global_plan_.poses.back().pose.position.x,
                                  robot.getOrigin().y() - global_plan_.poses.back().pose.position.y);


    for (auto iterator = global_plan_.poses.begin(); iterator != global_plan_.poses.end(); ++iterator) {
      Eigen::Vector2d temp_vector (robot.getOrigin().x() - iterator->pose.position.x,
                                   robot.getOrigin().y() - iterator->pose.position.y);
      if (temp_vector.norm() < 0.8) {
        if (iterator == global_plan_.poses.begin()) {
          break;
        }
        global_plan_.poses.erase(global_plan_.poses.begin(), iterator);
        break;
      }
    }
  }
  catch (const tf::TransformException& ex) {
   ROS_ERROR("prune global plan false, %s", ex.what());
    return false;
  }
  return true;
}

bool TebLocalPlanner::TransformGlobalPlan(int *current_goal_idx) {


  transformed_plan_.clear();

  try {
    if (global_plan_.poses.empty()) {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }
    UpdateGlobalToPlanTranform();

    tf::Stamped<tf::Pose> robot_pose;
    tf_.lock()->transformPose(global_plan_.poses.front().header.frame_id, robot_tf_pose_, robot_pose);

    double dist_threshold = std::max(costmap_->GetSizeXCell() * costmap_->GetResolution() / 2.0,
                                     costmap_->GetSizeYCell() * costmap_->GetResolution() / 2.0);
    dist_threshold *= 0.85;

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;
    double new_sq_dist = 0;
    while (i < (int)global_plan_.poses.size()) {
      double x_diff = robot_pose.getOrigin().x() - global_plan_.poses[i].pose.position.x;
      double y_diff = robot_pose.getOrigin().y() - global_plan_.poses[i].pose.position.y;
      new_sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold) {
        sq_dist = new_sq_dist;
        break;
      }
      sq_dist = new_sq_dist;
      ++i;
    }

    tf::Stamped<tf::Pose> tf_pose;
    geometry_msgs::PoseStamped newer_pose;

    double plan_length = 0;

    while(i < (int)global_plan_.poses.size() &&
        sq_dist <= sq_dist_threshold && (cut_lookahead_dist_<=0 ||
        plan_length <= cut_lookahead_dist_)) {
      const geometry_msgs::PoseStamped& pose = global_plan_.poses[i];
      tf::poseStampedMsgToTF(pose, tf_pose);
      tf_pose.setData(plan_to_global_transform_ * tf_pose);
      tf_pose.stamp_ = plan_to_global_transform_.stamp_;
      tf_pose.frame_id_ = global_frame_;
      tf::poseStampedTFToMsg(tf_pose, newer_pose);
      auto temp = DataConverter::LocalConvertGData(newer_pose.pose);
      DataBase data_pose(temp.first, temp.second);

      transformed_plan_.push_back(data_pose);

      double x_diff = robot_pose.getOrigin().x() - global_plan_.poses[i].pose.position.x;
      double y_diff = robot_pose.getOrigin().y() - global_plan_.poses[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;

      if (i>0 && cut_lookahead_dist_>0) {
        plan_length += Distance(global_plan_.poses[i-1].pose.position.x, global_plan_.poses[i-1].pose.position.y,
                                global_plan_.poses[i].pose.position.x, global_plan_.poses[i].pose.position.y);
      }

      ++i;
    }

    if (transformed_plan_.empty()) {
      tf::poseStampedMsgToTF(global_plan_.poses.back(), tf_pose);
      tf_pose.setData(plan_to_global_transform_ * tf_pose);
      tf_pose.stamp_ = plan_to_global_transform_.stamp_;
      tf_pose.frame_id_ = global_frame_;
      tf::poseStampedTFToMsg(tf_pose, newer_pose);

      auto temp = DataConverter::LocalConvertGData(newer_pose.pose);
      DataBase data_pose(temp.first, temp.second);

      transformed_plan_.push_back(data_pose);

      if (current_goal_idx) {
        *current_goal_idx = int(global_plan_.poses.size())-1;
      }
    } else {
      if (current_goal_idx) {
        *current_goal_idx = i-1;
      }
    }

  }
  catch(tf::LookupException& ex) {
    //LOG_ERROR << "transform error, " << ex.what();
    return false;
  }
  catch(tf::ConnectivityException& ex) {
    ROS_ERROR("Connectivity Error: %s", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex) {
    ROS_ERROR("Extrapolation Error: %s", ex.what());
    if (global_plan_.poses.size() > 0) {
      ROS_INFO("Global Frame: %s Plan Frame size %d : %s", global_frame_.c_str(),\
               (unsigned int)global_plan_.poses.size(),\
               global_plan_.poses[0].header.frame_id.c_str());
    }
    return false;
  }

  return true;
}

double TebLocalPlanner::EstimateLocalGoalOrientation(const DataBase& local_goal,
                                                     int current_goal_idx, int moving_average_length) const {
  int n = (int)global_plan_.poses.size();


  if (current_goal_idx > n-moving_average_length-2) {
    if (current_goal_idx >= n-1) {
      return local_goal.GetTheta();
    } else {
      tf::Quaternion global_orientation;
      tf::quaternionMsgToTF(global_plan_.poses.back().pose.orientation, global_orientation);
      return  tf::getYaw(plan_to_global_transform_.getRotation() *  global_orientation );
    }
  }

  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 );

  std::vector<double> candidates;
  tf::Stamped<tf::Pose> tf_pose_k;
  tf::Stamped<tf::Pose> tf_pose_kp1;

  const geometry_msgs::PoseStamped& pose_k = global_plan_.poses.at(current_goal_idx);
  tf::poseStampedMsgToTF(pose_k, tf_pose_k);
  tf_pose_kp1.setData(plan_to_global_transform_ * tf_pose_k);

  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i) {

    const geometry_msgs::PoseStamped& pose_k1 = global_plan_.poses.at(i+1);
    tf::poseStampedMsgToTF(pose_k1, tf_pose_kp1);
    tf_pose_kp1.setData(plan_to_global_transform_ * tf_pose_kp1);

    candidates.push_back( std::atan2(tf_pose_kp1.getOrigin().getY() - tf_pose_k.getOrigin().getY(),
                                     tf_pose_kp1.getOrigin().getX() - tf_pose_k.getOrigin().getX() ));

    if (i<range_end-1)
      tf_pose_k = tf_pose_kp1;
  }
  return AverageAngles(candidates);
}

void TebLocalPlanner::UpdateObstacleWithCostmap(Eigen::Vector2d local_goal) {

  //Eigen::Vector2d robot_orient = robot_pose_.OrientationUnitVec();
  Eigen::Vector2d goal_orient = local_goal - robot_pose_.GetPosition();

  for (unsigned int i=0; i<costmap_->GetSizeXCell()-1; ++i) {
    for (unsigned int j=0; j<costmap_->GetSizeYCell()-1; ++j) {
      if (costmap_->GetCost(i,j) == roborts_local_planner::LETHAL_OBSTACLE) {
        Eigen::Vector2d obs;
        costmap_->Map2World(i, j, obs.coeffRef(0), obs.coeffRef(1));

        Eigen::Vector2d obs_dir = obs - robot_pose_.GetPosition();
        if (obs_dir.dot(goal_orient) < 0
            && obs_dir.norm() > osbtacle_behind_robot_dist_) {
          continue;
        }

        obst_vector_.push_back(ObstaclePtr(new PointObstacle(obs)));
      }
    } // for y cell size
  }  // for x cell size

}

void TebLocalPlanner::UpdateViaPointsContainer() {

  via_points_.clear();

  double min_separation = param_config_.trajectory_opt().global_plan_viapoint_sep();
  if (min_separation<0) {
    return;
  }

  std::size_t prev_idx = 0;
  for (std::size_t i=1; i < transformed_plan_.size(); ++i) {// skip first one, since we do not need any point before the first min_separation [m]

    if (Distance( transformed_plan_[prev_idx].GetPosition().coeff(0), transformed_plan_[prev_idx].GetPosition().coeff(1),
                  transformed_plan_[i].GetPosition().coeff(0), transformed_plan_[i].GetPosition().coeff(1) ) < min_separation) {
      continue;
    }

    auto temp = transformed_plan_[i].GetPosition();
    via_points_.push_back(temp);
    prev_idx = i;
  }
}

void TebLocalPlanner::UpdateRobotPose() {
  local_cost_.lock()->GetRobotPose(robot_tf_pose_);
  Eigen::Vector2d position;
  position.coeffRef(0) = robot_tf_pose_.getOrigin().getX();
  position.coeffRef(1) = robot_tf_pose_.getOrigin().getY();
  robot_pose_ = DataBase(position, tf::getYaw(robot_tf_pose_.getRotation()));
}

void TebLocalPlanner::UpdateRobotVel() {
  tf::Stamped<tf::Pose> robot_vel_tf;
  odom_info_.GetVel(robot_vel_tf);
  robot_current_vel_.linear.x = robot_vel_tf.getOrigin().getX();
  robot_current_vel_.linear.y = robot_vel_tf.getOrigin().getY();
  robot_current_vel_.angular.z = tf::getYaw(robot_vel_tf.getRotation());
}

void TebLocalPlanner::UpdateGlobalToPlanTranform() {
  tf_.lock()->waitForTransform(global_frame_, ros::Time::now(),
                               global_plan_.poses.front().header.frame_id, global_plan_.poses.front().header.stamp,
                               global_plan_.poses.front().header.frame_id, ros::Duration(0.5));
  tf_.lock()->lookupTransform(global_frame_, ros::Time(),
                              global_plan_.poses.front().header.frame_id, global_plan_.poses.front().header.stamp,
                              global_plan_.poses.front().header.frame_id, plan_to_global_transform_);
}

void TebLocalPlanner::SaturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y,
                      double max_vel_theta, double max_vel_x_backwards) const {

  if (vx > max_vel_x) {
    vx = max_vel_x;
  }

  if (max_vel_x_backwards<=0) {
    ROS_INFO("Do not choose max_vel_x_backwards to be <=0. "\
    "Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  } else if (vx < -max_vel_x_backwards) {
    vx = -max_vel_x_backwards;
  }

  if (vy > max_vel_y) {
    vy = max_vel_y;
  } else if (vy < -max_vel_y) {
    vy = -max_vel_y;
  }

  if (omega > max_vel_theta) {
    omega = max_vel_theta;
  } else if (omega < -max_vel_theta) {
    omega = -max_vel_theta;
  }

}

double TebLocalPlanner::ConvertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const {
  if (omega==0 || v==0) {
    return 0;
  }

  double radius = v/omega;

  if (fabs(radius) < min_turning_radius) {
    radius = double(g2o::sign(radius)) * min_turning_radius;
  }

  return std::atan(wheelbase / radius);
}

roborts_common::ErrorInfo TebLocalPlanner::Initialize (std::shared_ptr<roborts_costmap::CostmapInterface> local_cost,
                                  std::shared_ptr<tf::TransformListener> tf, LocalVisualizationPtr visual) {

  if (!is_initialized_) {
    oscillation_ = std::chrono::system_clock::now();
    tf_ = tf;
    local_cost_ = local_cost;

    std::string full_path = ros::package::getPath("roborts_planning") + \
      "/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt";
    roborts_common::ReadProtoFromTextFile(full_path.c_str(), &param_config_);
    if (&param_config_ == nullptr) {
      ROS_ERROR("error occur when loading config file");
      roborts_common::ErrorInfo read_file_error(roborts_common::ErrorCode::LP_ALGORITHM_INITILIZATION_ERROR,
                                              "load algorithm param file failed");
      is_initialized_ = false;
      ROS_ERROR("%s", read_file_error.error_msg().c_str());
      return read_file_error;
    }
    check_back_size_ = param_config_.check_back_size();

    max_vel_x_          = param_config_.kinematics_opt().max_vel_x();
    max_vel_y_          = param_config_.kinematics_opt().max_vel_y();
    max_vel_theta_      = param_config_.kinematics_opt().max_vel_theta();
    max_vel_x_backwards = param_config_.kinematics_opt().max_vel_x_backwards();
    free_goal_vel_      = param_config_.tolerance_opt().free_goal_vel();

    xy_goal_tolerance_  = param_config_.tolerance_opt().xy_goal_tolerance();
    yaw_goal_tolerance_ = param_config_.tolerance_opt().yaw_goal_tolerance();

    cut_lookahead_dist_       = param_config_.trajectory_opt().max_global_plan_lookahead_dist();
    fesiable_step_look_ahead_ = param_config_.trajectory_opt().feasibility_check_no_poses();
    reduce_step_look_ahead_   = param_config_.reduce_check_look_ahead();

    osbtacle_behind_robot_dist_ = param_config_.obstacles_opt().costmap_obstacles_behind_robot_dist();

    global_plan_overwrite_orientation_ = param_config_.trajectory_opt().global_plan_overwrite_orientation();

    global_frame_ = local_cost_.lock()->GetGlobalFrameID();

    visual_ = visual;

    costmap_ = local_cost_.lock()->GetLayeredCostmap()->GetCostMap();

    robot_cost_ = std::make_shared<roborts_local_planner::RobotPositionCost>(*costmap_);

    obst_vector_.reserve(200);
    RobotFootprintModelPtr robot_model = GetRobotFootprintModel(param_config_);
    optimal_ = OptimalBasePtr(new TebOptimal(param_config_, &obst_vector_, robot_model, visual_, &via_points_));

    std::vector<geometry_msgs::Point> robot_footprint;
    robot_footprint = local_cost_.lock()->GetRobotFootprint();
    //robot_footprint_ = DataConverter::LocalConvertGData(robot_footprint);
    robot_footcontours_ = DataConverter::LocalConvertGData(robot_footprint);

    roborts_costmap::RobotPose robot_pose;
    local_cost_.lock()->GetRobotPose(robot_pose);
    auto temp_pose = DataConverter::LocalConvertRMData(robot_pose);
    robot_pose_ = DataBase(temp_pose.first, temp_pose.second);
    /*robot_footprint_ = DataConverter::LocalConvertGData(robot_footprint);
    for(int i=0; i<robot_footprint_.size(); i++ ){
      robot_footprint_[i].coeffRef(0) /= 2;
      robot_footprint_[i].coeffRef(1) /= 2;
    }*/
    robot_footprint_.push_back(robot_pose_.GetPosition());
    last_robot_pose_ = robot_pose_;

    RobotPositionCost::CalculateMinAndMaxDistances(robot_footprint_,
                                                   robot_inscribed_radius_,
                                                   robot_circumscribed_radius);
    odom_info_.SetTopic(param_config_.opt_frame().odom_frame());

    is_initialized_ = true;
    ROS_INFO("local algorithm initialize ok");
    return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
  }

  return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
}

void TebLocalPlanner::RegisterErrorCallBack(ErrorInfoCallback error_callback) {
  error_callback_ = error_callback;
}

bool TebLocalPlanner::CutAndTransformGlobalPlan(int *current_goal_idx) {
  if (!transformed_plan_.empty()) {
    transformed_plan_.clear();
  }

}

bool TebLocalPlanner::SetPlanOrientation() {
  if (global_plan_.poses.size() < 2) {
    ROS_WARN("can not compute the orientation because the global plan size is: %d", (int)global_plan_.poses.size());
    return false;
  } else {
    //auto goal = DataConverter::LocalConvertGData(global_plan_.poses.back().pose);
    //auto line_vector = (robot_pose_.GetPosition() - goal.first);
    //auto  orientation = GetOrientation(line_vector);
    for (int i = 0; i < global_plan_.poses.size() - 1; ++i) {
      auto pose = DataConverter::LocalConvertGData(global_plan_.poses[i].pose);
      auto next_pose = DataConverter::LocalConvertGData(global_plan_.poses[i+1].pose);
      double x = global_plan_.poses[i+1].pose.position.x - global_plan_.poses[i].pose.position.x;
      double y = global_plan_.poses[i+1].pose.position.y - global_plan_.poses[i].pose.position.y;
      double angle = atan2(y, x);
      auto quaternion = EulerToQuaternion(0, 0, angle);
      global_plan_.poses[i].pose.orientation.w = quaternion[0];
      global_plan_.poses[i].pose.orientation.x = quaternion[1];
      global_plan_.poses[i].pose.orientation.y = quaternion[2];
      global_plan_.poses[i].pose.orientation.z = quaternion[3];
    }
  }
}

void TebLocalPlanner::RobotGetBack(roborts_msgs::TwistAccel &cmd_vel)
{
  int obstacl_info = optimal_ -> GetBackVel(robot_cost_.get(), robot_footprint_, check_back_size_);
  ROS_INFO("obstacl_info %d", obstacl_info);

  //if(obstacl_info == 8){
    cmd_vel.twist.linear.x = 0;
    cmd_vel.twist.linear.y = 0;
    cmd_vel.twist.angular.z = 0;

    cmd_vel.accel.linear.x = 0;
    cmd_vel.accel.linear.y = 0;
    cmd_vel.accel.angular.z = 0;
  //}

}
} // namespace roborts_local_planner
