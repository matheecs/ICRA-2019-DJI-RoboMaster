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
#include "dwa/dwa_local_planner.h"

namespace roborts_local_planner {
DWALocalPlanner::DWALocalPlanner() : dwa_(NULL){

}
DWALocalPlanner::~DWALocalPlanner(){
  if(dwa_ != NULL)
      delete dwa_;
}
void DWALocalPlanner::DirectComputeGlobalVelocity(roborts_msgs::TwistAccel &cmd_vel){}
roborts_common::ErrorInfo DWALocalPlanner::Initialize (
    std::shared_ptr<roborts_costmap::CostmapInterface> local_cost,
    std::shared_ptr<tf::TransformListener> tf, 
    LocalVisualizationPtr visual) {

  if (!is_initialized_) {
    oscillation_ = std::chrono::system_clock::now();
    tf_ = tf;
    local_cost_ = local_cost;

    std::string full_path = ros::package::getPath("roborts_planning") + \
      "/local_planner/dwa/config/dwa.prototxt";
    roborts_common::ReadProtoFromTextFile(full_path.c_str(), &param_config_);
    if (&param_config_ == nullptr) {
      ROS_ERROR("error occur when loading config file");
      roborts_common::ErrorInfo read_file_error(roborts_common::ErrorCode::LP_ALGORITHM_INITILIZATION_ERROR,
                                              "load algorithm param file failed");
      is_initialized_ = false;
      ROS_ERROR("%s", read_file_error.error_msg().c_str());
      return read_file_error;
    }
    max_vel_x_          = param_config_.kinematics_opt().max_vel_x();
    max_vel_y_          = param_config_.kinematics_opt().max_vel_y();
    max_vel_theta_      = param_config_.kinematics_opt().max_vel_theta();
    min_vel_x_          = param_config_.kinematics_opt().min_vel_x();
    min_vel_y_          = param_config_.kinematics_opt().min_vel_y();
    min_vel_theta_      = param_config_.kinematics_opt().min_vel_theta();
    min_in_place_vel_th_= param_config_.kinematics_opt().min_in_place_vel_th();
    backup_vel_         = param_config_.kinematics_opt().max_vel_x_backwards();
    acc_lim_x_          = param_config_.kinematics_opt().acc_lim_x();
    acc_lim_y_          = param_config_.kinematics_opt().acc_lim_y();
    acc_lim_theta_      = param_config_.kinematics_opt().acc_lim_theta(); 

    xy_goal_tolerance_  = param_config_.tolerance_opt().xy_goal_tolerance();
    yaw_goal_tolerance_ = param_config_.tolerance_opt().yaw_goal_tolerance();

    cut_lookahead_dist_       = param_config_.trajectory_opt().max_global_plan_lookahead_dist();
    fesiable_step_look_ahead_ = param_config_.trajectory_opt().feasibility_check_no_poses();
    sim_period_               = param_config_.optimize_info().sim_period();

    osbtacle_behind_robot_dist_ = param_config_.obstacles_opt().costmap_obstacles_behind_robot_dist();

    global_plan_overwrite_orientation_ = param_config_.trajectory_opt().global_plan_overwrite_orientation();

    global_frame_ = local_cost_.lock()->GetGlobalFrameID();
    visual_ = visual;
    costmap_ = local_cost_.lock()->GetLayeredCostmap()->GetCostMap();
    robot_cost_ = std::make_shared<roborts_local_planner::RobotPositionCost>(*costmap_);
    
    std::vector<geometry_msgs::Point> robot_footprint;
    robot_footprint = local_cost_.lock()->GetRobotFootprint();
    std::vector<Eigen::Vector2d> footprint_spec;
    for(int i=0; i<robot_footprint.size(); i++){
      Eigen::Vector2d tmp(robot_footprint[i].x, robot_footprint[i].y);
      footprint_spec.push_back(tmp);
    }
    // robot_footprint_ = DataConverter::LocalConvertGData(robot_footprint);
    roborts_costmap::RobotPose robot_pose;
    local_cost_.lock()->GetRobotPose(robot_pose);
    auto temp_pose = DataConverter::LocalConvertRMData(robot_pose);
    robot_pose_ = DataBase(temp_pose.first, temp_pose.second);
    robot_footprint_.push_back(robot_pose_.GetPosition());
    last_robot_pose_ = robot_pose_;

    RobotPositionCost::CalculateMinAndMaxDistances(robot_footprint_,
                                                   robot_inscribed_radius_,
                                                   robot_circumscribed_radius_);
    odom_info_.SetTopic(param_config_.opt_frame().odom_frame());

    is_initialized_ = true;


    dwa_ = new DWAAlg(footprint_spec, robot_cost_, costmap_, robot_inscribed_radius_,
          robot_circumscribed_radius_, max_vel_x_, max_vel_y_, max_vel_theta_, min_vel_x_, min_vel_y_,
          min_vel_theta_, acc_lim_x_, acc_lim_y_, acc_lim_theta_, min_in_place_vel_th_,
          backup_vel_, param_config_.optimize_info().heading_lookahead(), param_config_.optimize_info().dwa(), 
          param_config_.optimize_info().heading_scoring(), param_config_.optimize_info().simple_attractor(),
          param_config_.optimize_info().holonomic_robot(), param_config_.optimize_info().vx_samples(),
          param_config_.optimize_info().vy_samples(), param_config_.optimize_info().vtheta_samples(),
          param_config_.optimize_info().sim_time(), param_config_.optimize_info().sim_granularity(), 
          param_config_.optimize_info().angular_sim_granularity(), param_config_.optimize_info().heading_scoring_timestep(),
          param_config_.optimize_info().pdist_scale(), param_config_.optimize_info().gdist_scale(), 
          param_config_.optimize_info().occdist_scale(), param_config_.optimize_info().sim_period(),
          param_config_.optimize_info().escape_reset_dist(), param_config_.optimize_info().escape_reset_theta(),
          param_config_.optimize_info().oscillation_reset_dist());
    ROS_INFO("local algorithm initialize ok");
    return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
  }

  return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
}

roborts_common::ErrorInfo DWALocalPlanner::ComputeVelocityCommands(roborts_msgs::TwistAccel &cmd_vel) 
{
  if (!is_initialized_) {
    ROS_ERROR("dwa algorithm doesn't be initialized");
    roborts_common::ErrorInfo algorithm_init_error(roborts_common::LP_ALGORITHM_INITILIZATION_ERROR,
                                                 "dwa initialize failed");
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
    rotating_to_goal_ = false;
    xy_tolerance_latch_ = false;
    return algorithm_ok;
  }
  
  PruneGlobalPlan();

  int goal_idx; //规划路径上的目标点个数
  if (!TransformGlobalPlan(&goal_idx)) {
    roborts_common::ErrorInfo PlanTransformError(roborts_common::LP_PLANTRANSFORM_ERROR, "plan transform error");
    ROS_ERROR("%s", PlanTransformError.error_msg().c_str());
    return PlanTransformError;
  }
  if (transformed_plan_.path_.empty()) {
    roborts_common::ErrorInfo PlanTransformError(roborts_common::LP_PLANTRANSFORM_ERROR, "transformed plan is empty");
    ROS_ERROR("transformed plan is empty");
    return PlanTransformError;
  }
  if (global_plan_overwrite_orientation_) {
    transformed_plan_.path_.back().SetTheta(EstimateLocalGoalOrientation(transformed_plan_.path_.back(), goal_idx));
  }

  if (transformed_plan_.path_.size()==1) {// plan only contains the goal
    transformed_plan_.path_.insert(transformed_plan_.path_.begin(), robot_pose_); // insert start (not yet initialized)
  } else {
    transformed_plan_.path_.front() = robot_pose_;// update start;
  }

  robot_goal_ = transformed_plan_.path_.back();
  

  // we've reached the goal position
  double distance, angle;
  getGoalPosOriDiff(distance, angle);
  ROS_INFO("compute velocity begin");
  if (xy_tolerance_latch_ || distance <= xy_goal_tolerance_) {
    ROS_INFO("compute velocity near");
    //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
    //just rotate in place
    if (latch_xy_goal_tolerance_) {
      xy_tolerance_latch_ = true;
    }
    // the goal orientation has been reached
    // 但这种情况已经在isgoalreach中讨论过
    if (fabs(angle) <= yaw_goal_tolerance_) 
    {
      rotating_to_goal_ = false;
      xy_tolerance_latch_ = false;
      ROS_INFO("compute velocity succeed");
      return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
    }
    else{
      dwa_->updatePlan(transformed_plan_);
      TrajectoryDataBase local_path;
      geometry_msgs::Twist drive_velocities;
      dwa_->findBestPath(robot_pose_, robot_current_vel_, local_path, drive_velocities);
      cmd_vel.twist = drive_velocities;
      ROS_INFO("81");
  
      //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
      if ( !rotating_to_goal_ && !velSmallStopped()) {
        if ( !stopWithAccLimits(robot_pose_, robot_current_vel_, cmd_vel)) {
          roborts_common::ErrorInfo velocity_error(roborts_common::LP_VELOCITY_ERROR, "stopWithAccLimits velocity is not vaild");
          return velocity_error;
        }
      }
      //if we're stopped... then we want to rotate to goal
      else{
        //set this so that we know its OK to be moving
        rotating_to_goal_ = true;
        if(!rotateToGoal(robot_pose_, robot_current_vel_, robot_goal_.GetTheta(), cmd_vel)) {
          roborts_common::ErrorInfo velocity_error(roborts_common::LP_VELOCITY_ERROR, "rotateToGoal velocity is not vaild");
          return velocity_error;
        }
      }
    }
    ROS_INFO("compute velocity succeed");
    return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
  }  
  ROS_INFO("compute velocity far");
  dwa_->updatePlan(transformed_plan_);
  TrajectoryDataBase local_path;
  geometry_msgs::Twist drive_velocities;
  dwa_->findBestPath(robot_pose_, robot_current_vel_, local_path, drive_velocities);
  ROS_INFO("compute velocity 3");
  cmd_vel.twist = drive_velocities;
  ROS_INFO("411");
  
  if(local_path.cost_ < 0)
  {
    ROS_ERROR("trajectory is not feasible");
    roborts_common::ErrorInfo trajectory_error(roborts_common::LP_ALGORITHM_TRAJECTORY_ERROR, "trajectory is not feasible");
    return trajectory_error;
  }
  else{
    ROS_INFO("compute velocity succeed");
    return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
  }
}

bool DWALocalPlanner::SetPlan(const nav_msgs::Path& plan, const geometry_msgs::PoseStamped& goal)
{
  if (plan_mutex_.try_lock()) {
    ROS_INFO("set plan");
    if (plan.poses.empty()) {
      temp_plan_.poses.push_back(goal);
    } else {
      temp_plan_ = plan;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    xy_tolerance_latch_ = false;
    plan_mutex_.unlock();
  }
}
bool DWALocalPlanner::SetMaxVel(float maxvelx, float maxvely){}
bool DWALocalPlanner::GetPlan(const nav_msgs::Path& plan)
{
  if (plan_mutex_.try_lock()) {
    global_plan_ = plan;
    plan_mutex_.unlock();
  }
}

bool DWALocalPlanner::IsGoalReached ()
{
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


void DWALocalPlanner::UpdateRobotPose() {
  local_cost_.lock()->GetRobotPose(robot_tf_pose_);
  Eigen::Vector2d position;
  position.coeffRef(0) = robot_tf_pose_.getOrigin().getX();
  position.coeffRef(1) = robot_tf_pose_.getOrigin().getY();
  robot_pose_ = DataBase(position, tf::getYaw(robot_tf_pose_.getRotation()));
}

void DWALocalPlanner::UpdateRobotVel() {
  tf::Stamped<tf::Pose> robot_vel_tf;
  odom_info_.GetVel(robot_vel_tf);
  robot_current_vel_.linear.x = robot_vel_tf.getOrigin().getX();
  robot_current_vel_.linear.y = robot_vel_tf.getOrigin().getY();
  robot_current_vel_.angular.z = tf::getYaw(robot_vel_tf.getRotation());
}

void DWALocalPlanner::UpdateGlobalToPlanTranform() {
  tf_.lock()->waitForTransform(global_frame_, ros::Time::now(),
                               global_plan_.poses.front().header.frame_id, global_plan_.poses.front().header.stamp,
                               global_plan_.poses.front().header.frame_id, ros::Duration(0.5));
  tf_.lock()->lookupTransform(global_frame_, ros::Time(),
                              global_plan_.poses.front().header.frame_id, global_plan_.poses.front().header.stamp,
                              global_plan_.poses.front().header.frame_id, plan_to_global_transform_);
}

bool DWALocalPlanner::PruneGlobalPlan() {
  if (global_plan_.poses.empty()) {
    return true;
  }
  try {
    tf::StampedTransform global_to_plan_transform;
    tf_.lock()->lookupTransform(global_plan_.poses.front().header.frame_id,
                                robot_tf_pose_.frame_id_, ros::Time(0),
                                global_to_plan_transform);
    tf::Stamped<tf::Pose> robot;
    // 机器人位置在local's global frame 中的位置
    robot.setData( global_to_plan_transform * robot_tf_pose_ );

    // robot_to_goal 机器人到目标的距离
    Eigen::Vector2d robot_to_goal(robot.getOrigin().x() - global_plan_.poses.back().pose.position.x,
                                  robot.getOrigin().y() - global_plan_.poses.back().pose.position.y);

    for (auto iterator = global_plan_.poses.begin(); iterator != global_plan_.poses.end(); ++iterator) {
      Eigen::Vector2d temp_vector (robot.getOrigin().x() - iterator->pose.position.x,
                                   robot.getOrigin().y() - iterator->pose.position.y);
      if (temp_vector.norm() < 0.8) {
        if (iterator == global_plan_.poses.begin()) {
          break;
        }
        // 走回头路
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

bool DWALocalPlanner::TransformGlobalPlan(int *current_goal_idx) {
  transformed_plan_.path_.clear();

  try {
    if (global_plan_.poses.empty()) {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // 得到坐标系转换
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

      transformed_plan_.path_.push_back(data_pose);

      double x_diff = robot_pose.getOrigin().x() - global_plan_.poses[i].pose.position.x;
      double y_diff = robot_pose.getOrigin().y() - global_plan_.poses[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;

      if (i>0 && cut_lookahead_dist_>0) {
        plan_length += Distance(global_plan_.poses[i-1].pose.position.x, global_plan_.poses[i-1].pose.position.y,
                                global_plan_.poses[i].pose.position.x, global_plan_.poses[i].pose.position.y);
      }

      ++i;
    }

    if (transformed_plan_.path_.empty()) {
      tf::poseStampedMsgToTF(global_plan_.poses.back(), tf_pose);
      tf_pose.setData(plan_to_global_transform_ * tf_pose);
      tf_pose.stamp_ = plan_to_global_transform_.stamp_;
      tf_pose.frame_id_ = global_frame_;
      tf::poseStampedTFToMsg(tf_pose, newer_pose);

      auto temp = DataConverter::LocalConvertGData(newer_pose.pose);
      DataBase data_pose(temp.first, temp.second);

      transformed_plan_.path_.push_back(data_pose);

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

double DWALocalPlanner::EstimateLocalGoalOrientation(const DataBase& local_goal,
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


bool DWALocalPlanner::stopWithAccLimits(const DataBase& robot_pose, const geometry_msgs::Twist& robot_current_vel, roborts_msgs::TwistAccel &cmd_vel){
  //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
  //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
  double vx = Sign(robot_current_vel.linear.x) * std::max(0.0, (fabs(robot_current_vel.linear.x) - acc_lim_x_ * sim_period_));
  double vy = Sign(robot_current_vel.linear.y) * std::max(0.0, (fabs(robot_current_vel.linear.y) - acc_lim_y_ * sim_period_));

  double vel_yaw = robot_current_vel.angular.z;
  double vth = Sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim_theta_ * sim_period_));

  //we do want to check whether or not the command is valid
  double yaw = robot_pose.GetTheta();
  bool valid_cmd = dwa_->checkTrajectory(robot_pose.GetPosition().coeffRef(0), robot_pose.GetPosition().coeffRef(1), yaw, 
                      robot_current_vel.linear.x, robot_current_vel.linear.y, vel_yaw, vx, vy, vth);

  //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
  if(valid_cmd){
  // ROS_DEBUG("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
  cmd_vel.twist.linear.x = vx;
  cmd_vel.twist.linear.y = vy;
  cmd_vel.twist.angular.z = vth;
  return true;
  }

  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.linear.y = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  return false;
}

bool DWALocalPlanner::rotateToGoal(const DataBase& robot_pose, const geometry_msgs::Twist& robot_current_vel, double goal_th, roborts_msgs::TwistAccel &cmd_vel)
{
  double yaw = robot_pose.GetTheta();
  double vel_yaw = robot_current_vel.angular.z;
  cmd_vel.twist.linear.x = 0;
  cmd_vel.twist.linear.y = 0;
  double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

  double v_theta_samp = ang_diff > 0.0 ? std::min(max_vel_theta_, std::max(min_in_place_vel_th_, ang_diff)) : 
                                         std::max(min_vel_theta_, std::min(-1.0 * min_in_place_vel_th_, ang_diff));

  //take the acceleration limits of the robot into account
  double max_acc_vel = fabs(vel_yaw) + acc_lim_theta_ * sim_period_;
  double min_acc_vel = fabs(vel_yaw) - acc_lim_theta_ * sim_period_;

  v_theta_samp = Sign(v_theta_samp) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

  //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
  double max_speed_to_stop = sqrt(2 * acc_lim_theta_ * fabs(ang_diff)); 

  v_theta_samp = Sign(v_theta_samp) * std::min(max_speed_to_stop, fabs(v_theta_samp));

  //we still want to lay down the footprint of the robot and check if the action is legal
  bool valid_cmd = dwa_->checkTrajectory(robot_pose.GetPosition().coeffRef(0), robot_pose.GetPosition().coeffRef(1), yaw, 
                        robot_current_vel.linear.x, robot_current_vel.linear.y, vel_yaw, 0.0, 0.0, v_theta_samp);

  ROS_DEBUG("Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);

  if(valid_cmd){
    cmd_vel.twist.angular.z = v_theta_samp;
    return true;
  }

  cmd_vel.twist.angular.z = 0.0;
  return false;
}

} // namespace roborts_local_planner
