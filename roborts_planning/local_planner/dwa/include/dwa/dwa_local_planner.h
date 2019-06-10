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
#ifndef ROBORTS_PLANNING_DWA_LOCAL_PLANNER_H_
#define ROBORTS_PLANNING_DWA_LOCAL_PLANNER_H_

#include <mutex>

#include "io/io.h"
#include "state/error_code.h"
#include "alg_factory/algorithm_factory.h"

#include "costmap/costmap_interface.h"

#include "local_planner/local_planner_base.h"
#include "local_planner/robot_position_cost.h"
#include "local_planner/local_visualization.h"
#include "local_planner/utility_tool.h"
#include "local_planner/odom_info.h"
#include "local_planner/data_converter.h"
#include "local_planner/data_base.h"
#include "local_planner/obstacle.h"
#include "local_planner/robot_footprint_model.h"

#include "../proto/dwa.pb.h"
#include "dwa_alg.h"

namespace roborts_local_planner {
class DWALocalPlanner : public LocalPlannerBase {
 public:
   DWALocalPlanner();
   ~DWALocalPlanner();

   /**
    * @brief initialize local planner algorithm 
    * @param local_cost Local cost map
    * @param tf Tf listener
    * @param visual Visualize pointer
    * @return Error info
    */
   roborts_common::ErrorInfo Initialize (std::shared_ptr<roborts_costmap::CostmapInterface> local_cost,
                   std::shared_ptr<tf::TransformListener> tf, LocalVisualizationPtr visual) override;    

   /**
    * @brief  Given the current position, orientation, and velocity of the robot,
    *         compute velocity commands to send to the base
    * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
    * @return Error info
    */
   roborts_common::ErrorInfo ComputeVelocityCommands(roborts_msgs::TwistAccel &cmd_vel) override;
   void DirectComputeGlobalVelocity(roborts_msgs::TwistAccel &cmd_vel) override;
   /**
    * @brief Set global plan's result to local planner or set a goal to local planner
    * @param plan Result of global planner
    * @param goal Goal of local planner
    * @return If true success, else fail
    */
   bool SetPlan(const nav_msgs::Path& plan, const geometry_msgs::PoseStamped& goal) override ;
   bool SetMaxVel(float maxvelx, float maxvely);
   bool GetPlan(const nav_msgs::Path& plan);

   /**
    * @brief  Check if the goal pose (both distance and orientation)has been achieved
    * @return True if achieved, false otherwise
    */
   bool IsGoalReached () override;

   /**
    * @brief Register error callback function
    * @param error_callback Callback function
    */
   void RegisterErrorCallBack(ErrorInfoCallback error_callback) {error_callback_ = error_callback;}

   bool isInitialized() { return is_initialized_; }

 protected:
   void UpdateRobotPose();
   void UpdateRobotVel();
   void UpdateGlobalToPlanTranform();
   void UpdateObstacleWithCostmap(Eigen::Vector2d local_goal);
   // 机器人的全局路径如果走回头路就清除
   bool PruneGlobalPlan();
   bool TransformGlobalPlan(int *current_goal_idx = NULL);
   double EstimateLocalGoalOrientation(const DataBase& local_goal,
                                      int current_goal_idx, 
                                      int moving_average_length=3) const;

 private:
   /**
    * @brief Callback to update the local planner's parameters based on dynamic reconfigure
    */
    //  void reconfigureCB(DWAPlannerConfig &config, uint32_t level);

    //  void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

    //  void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);


   DWAAlg* dwa_;
   
   //! Tf listener
   std::weak_ptr<tf::TransformListener> tf_;
   //! Local cost map
   std::weak_ptr<roborts_costmap::CostmapInterface> local_cost_;
   //! Local planner costmap 2d
   roborts_costmap::Costmap2D *costmap_;
   //! *Local* planner frame(local planner will do optimal in this frame), different with global planner frame
   std::string global_frame_;
   //! Tf transform from global planner frame to optimal(local global) frame
   tf::StampedTransform plan_to_global_transform_;
   
   //! Global planner's solve
   nav_msgs::Path global_plan_, temp_plan_;
   //! Robot goal
   DataBase robot_goal_;
   //! When no global planner give the global plan, use local goal express robot end point
   tf::Stamped<tf::Pose> local_goal_;
   //! Way point after tf transform
   TrajectoryDataBase transformed_plan_;

   //! Robot footprint cost
   std::shared_ptr<roborts_local_planner::RobotPositionCost> robot_cost_;
   //! Robot inscribed radius
   double robot_inscribed_radius_;
   //! Robot circumscribed radius
   double robot_circumscribed_radius_; 
   //! Robot footprint
   std::vector<Eigen::Vector2d> robot_footprint_;
   //! Robot odom info
   OdomInfo odom_info_;
   //! Robot current pose
   DataBase robot_pose_;
   //! Robot last position
   DataBase last_robot_pose_;
   //! Robot current pose
   tf::Stamped<tf::Pose> robot_tf_pose_;
   //! Robot current velocity
   geometry_msgs::Twist robot_current_vel_;
   //! Last velocity
   roborts_msgs::TwistAccel last_cmd_;

   //! Plan mutex
   std::mutex plan_mutex_;
   //! Visualize ptr use to visualize trajectory after optimize
   LocalVisualizationPtr visual_;

   //! Error info when running dwa local planner algorithm
   roborts_common::ErrorInfo dwa_error_info_;
   //! Call back function use to return error info
   ErrorInfoCallback error_callback_;
   //! Time begin when robot oscillation at a position
   std::chrono::system_clock::time_point oscillation_;
   //! Time allow robot oscillation at a position
   double oscillation_time_;


   //! Check if the algorithm is initialized
   bool is_initialized_= false;

   //! Optimal param
   DWAConfig param_config_;
   bool  free_goal_vel_;
   bool  global_plan_overwrite_orientation_;
   double cut_lookahead_dist_;
   long  fesiable_step_look_ahead_;
   double sim_period_;

   double max_vel_x_, max_vel_y_, max_vel_theta_;
   double min_vel_x_, min_vel_y_, min_vel_theta_;
   double acc_lim_x_, acc_lim_y_, acc_lim_theta_; 
   double min_in_place_vel_th_;
   double backup_vel_; 

   double xy_goal_tolerance_, yaw_goal_tolerance_;
   double osbtacle_behind_robot_dist_;

   double rot_stopped_velocity_, trans_stopped_velocity_;
   double inflation_radius_;
   bool prune_plan_;
   bool rotating_to_goal_;
   bool latch_xy_goal_tolerance_, xy_tolerance_latch_;

   double getGoalPosOriDiff(double& distance, double& delta_orient) {
     tf::Stamped<tf::Pose> global_goal;
     tf::poseStampedMsgToTF(global_plan_.poses.back(), global_goal);
     global_goal.setData( plan_to_global_transform_ * global_goal );
     auto goal = DataConverter::LocalConvertTFData(global_goal);

     distance = (goal.first - robot_pose_.GetPosition()).norm();
     delta_orient = g2o::normalize_theta( goal.second - robot_pose_.GetTheta());
   }

   bool velSmallStopped(){
     tf::Stamped<tf::Pose> robot_vel_tf;
     odom_info_.GetVel(robot_vel_tf);
     return fabs(tf::getYaw(robot_vel_tf.getRotation())) <= rot_stopped_velocity_
            && fabs(robot_vel_tf.getOrigin().getX()) <= trans_stopped_velocity_
            && fabs(robot_vel_tf.getOrigin().getY()) <= trans_stopped_velocity_;
   }

   bool stopWithAccLimits(const DataBase& robot_pose, const geometry_msgs::Twist& robot_current_vel, roborts_msgs::TwistAccel &cmd_vel);
  /**
   * @brief Once a goal position is reached... rotate to the goal orientation
   * @param  global_pose The pose of the robot in the global frame
   * @param  robot_vel The velocity of the robot
   * @param  goal_th The desired th value for the goal
   * @param  cmd_vel The velocity commands to be filled
   * @return  True if a valid trajectory was found, false otherwise
   */
  bool rotateToGoal(const DataBase& robot_pose, const geometry_msgs::Twist& robot_current_vel, double goal_th, roborts_msgs::TwistAccel &cmd_vel);
  
  int Sign(double x) {
    return x > 0 ? 1 : -1;
  }
  };// class roborts_local_planner

  /**
 * Register the algorithm to algorithm factory
 */
roborts_common::REGISTER_ALGORITHM(LocalPlannerBase, "dwa", DWALocalPlanner);
}
#endif //ROBORTS_PLANNING_DWA_LOCAL_PLANNER_H_
