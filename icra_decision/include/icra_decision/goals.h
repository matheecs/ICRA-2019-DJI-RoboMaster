#ifndef MODULE_DECISION_ICRA_GOAL_FACORY_H
#define MODULE_DECISION_ICRA_GOAL_FACORY_H

#include <Eigen/Core>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "icra_decision/GlobalPlannerAction.h"
#include "icra_decision/LocalPlannerAction.h"
#include "icra_decision/SupplyPidAction.h"
#include "icra_decision/TurnAngleAction.h"
// CostMap
#include "costmap/costmap_interface.h"
#include "blackboard.h"

namespace decision {

enum class SyncStateType {
  FREE        = 0,
  SUPPLY_BUSY = 1,
  BUFF_BUSY   = 2
};

class GoalFactory {
public:
  typedef std::shared_ptr<GoalFactory> GoalFactoryPtr;
  typedef actionlib::SimpleActionClient<icra_decision::LocalPlannerAction> LocalActionClient;
  typedef actionlib::SimpleActionClient<icra_decision::GlobalPlannerAction> GlobalActionClient;
  typedef icra_decision::GlobalPlannerFeedbackConstPtr GlobalFeedback;
  typedef icra_decision::LocalPlannerGoal LocalGoal;
  typedef icra_decision::GlobalPlannerGoal GlobalGoal;
  // CostMap
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;

  GoalFactory(const Blackboard::Ptr &blackboard_ptr):
    blackboard_ptr_(blackboard_ptr),
    patrol_count_(0),
    search_count_(0),
    lost_enemy_(true),
    bullet_supply_actionlib_client_("supplypid", true),
    /*****leonard add******/
    turn_angle_actionlib_client_("turnangle", true),
    global_planner_actionlib_client_("global_planner_node_action", true),
    local_planner_actionlib_client_("local_planner_node_action", true) {
    
    angle_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    // CostMap
    ROS_INFO("Create CostMap Start...");
    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
    std::string map_path = ros::package::getPath("roborts_costmap") + "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_, map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();
    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();
    ROS_INFO("Adding CostMap...Done.");

    // Load the configurations:
    nh.getParam("point_size", point_size_);
    ROS_INFO("point_size = %d", point_size_);
    patrol_goals_.resize(point_size_);
    for (int i = 0; i < point_size_; i++) {
      patrol_goals_[i].header.frame_id = "map";
      nh.getParam("point"+std::to_string(i)+"/x", patrol_goals_[i].pose.position.x);
      nh.getParam("point"+std::to_string(i)+"/y", patrol_goals_[i].pose.position.y);
      nh.getParam("point"+std::to_string(i)+"/z", patrol_goals_[i].pose.position.z);

      double tmp_roll, tmp_pitch, tmp_yaw;
      nh.getParam("point"+std::to_string(i)+"/roll", tmp_roll);
      nh.getParam("point"+std::to_string(i)+"/pitch", tmp_pitch);
      nh.getParam("point"+std::to_string(i)+"/yaw", tmp_yaw);
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(tmp_roll, tmp_pitch, tmp_yaw);
      patrol_goals_[i].pose.orientation.x = quaternion.x();
      patrol_goals_[i].pose.orientation.y = quaternion.y();
      patrol_goals_[i].pose.orientation.z = quaternion.z();
      patrol_goals_[i].pose.orientation.w = quaternion.w();
    }

    // Load Escape Parameters:
    nh.getParam("escape/left_x_limit", left_x_limit_);
    nh.getParam("escape/right_x_limit", right_x_limit_);
    nh.getParam("escape/robot_x_limit", robot_x_limit_);
    nh.getParam("escape/left_random_min_x", left_random_min_x_);
    nh.getParam("escape/left_random_max_x", left_random_max_x_);
    nh.getParam("escape/right_random_min_x", right_random_min_x_);
    nh.getParam("escape/right_random_max_x", right_random_max_x_);
    nh.getParam("whirl_vel/angle_x_vel", whirl_vel_.angular.x);
    nh.getParam("whirl_vel/angle_y_vel", whirl_vel_.angular.y);
    nh.getParam("whirl_vel/angle_z_vel", whirl_vel_.angular.z);

     // Load Enemy Field Parameters:
    nh.getParam("enemy_field_supplier/x_min", enemy_field_supplier_x_);
    nh.getParam("enemy_field_supplier/y_min", enemy_field_supplier_y_);
    nh.getParam("enemy_field_buff/x_min", enemy_field_buff_x_);
    nh.getParam("enemy_field_buff/y_min", enemy_field_buff_y_);


    // Load search parameters:
    // Only three points in any search region!
    search_region_.resize(3);
    for (int i = 1; i <= 3; i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      nh.getParam("search_region_1_" + std::to_string(i) + "/x", search_point.pose.position.x);
      nh.getParam("search_region_1_" + std::to_string(i) + "/y", search_point.pose.position.y);
      nh.getParam("search_region_1_" + std::to_string(i) + "/z", search_point.pose.position.z);

      double tmp_roll, tmp_pitch, tmp_yaw;
      nh.getParam("search_region_1_" + std::to_string(i) + "/roll", tmp_roll);
      nh.getParam("search_region_1_" + std::to_string(i) + "/pitch", tmp_pitch);
      nh.getParam("search_region_1_" + std::to_string(i) + "/yaw", tmp_yaw);
      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(tmp_roll, tmp_pitch, tmp_yaw);
      search_point.pose.orientation = quaternion;
      search_region_1_.push_back(search_point);
    }

    for (int i = 1; i <= 3; i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      nh.getParam("search_region_2_" + std::to_string(i) + "/x", search_point.pose.position.x);
      nh.getParam("search_region_2_" + std::to_string(i) + "/y", search_point.pose.position.y);
      nh.getParam("search_region_2_" + std::to_string(i) + "/z", search_point.pose.position.z);

      double tmp_roll, tmp_pitch, tmp_yaw;
      nh.getParam("search_region_2_" + std::to_string(i) + "/roll", tmp_roll);
      nh.getParam("search_region_2_" + std::to_string(i) + "/pitch", tmp_pitch);
      nh.getParam("search_region_2_" + std::to_string(i) + "/yaw", tmp_yaw);
      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(tmp_roll, tmp_pitch, tmp_yaw);
      search_point.pose.orientation = quaternion;
      search_region_2_.push_back(search_point);
    }

    for (int i = 1; i <= 3; i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      nh.getParam("search_region_3_" + std::to_string(i) + "/x", search_point.pose.position.x);
      nh.getParam("search_region_3_" + std::to_string(i) + "/y", search_point.pose.position.y);
      nh.getParam("search_region_3_" + std::to_string(i) + "/z", search_point.pose.position.z);

      double tmp_roll, tmp_pitch, tmp_yaw;
      nh.getParam("search_region_3_" + std::to_string(i) + "/roll", tmp_roll);
      nh.getParam("search_region_3_" + std::to_string(i) + "/pitch", tmp_pitch);
      nh.getParam("search_region_3_" + std::to_string(i) + "/yaw", tmp_yaw);
      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(tmp_roll, tmp_pitch, tmp_yaw);
      search_point.pose.orientation = quaternion;
      search_region_3_.push_back(search_point);
    }

    for (int i = 1; i <= 3; i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      nh.getParam("search_region_4_" + std::to_string(i) + "/x", search_point.pose.position.x);
      nh.getParam("search_region_4_" + std::to_string(i) + "/y", search_point.pose.position.y);
      nh.getParam("search_region_4_" + std::to_string(i) + "/z", search_point.pose.position.z);

      double tmp_roll, tmp_pitch, tmp_yaw;
      nh.getParam("search_region_4_" + std::to_string(i) + "/roll", tmp_roll);
      nh.getParam("search_region_4_" + std::to_string(i) + "/pitch", tmp_pitch);
      nh.getParam("search_region_4_" + std::to_string(i) + "/yaw", tmp_yaw);
      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(tmp_roll, tmp_pitch, tmp_yaw);
      search_point.pose.orientation = quaternion;
      search_region_4_.push_back(search_point);
    }

    // Read the supply and buff location:
    supply_location_.resize(1);
    {
      supply_location_[0].header.frame_id = "map";
      nh.getParam("supplier_location/x", supply_location_[0].pose.position.x);
      nh.getParam("supplier_location/y", supply_location_[0].pose.position.y);
      nh.getParam("supplier_location/z", supply_location_[0].pose.position.z);

      double tmp_roll, tmp_pitch, tmp_yaw;
      nh.getParam("supplier_location/roll", tmp_roll);
      nh.getParam("supplier_location/pitch", tmp_pitch);
      nh.getParam("supplier_location/yaw", tmp_yaw);
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(tmp_roll, tmp_pitch, tmp_yaw);
      supply_location_[0].pose.orientation.x = quaternion.x();
      supply_location_[0].pose.orientation.y = quaternion.y();
      supply_location_[0].pose.orientation.z = quaternion.z();
      supply_location_[0].pose.orientation.w = quaternion.w();
    }
    // Read the supply location:
    buff_location_.resize(1);
    {
      buff_location_[0].header.frame_id = "map";
      nh.getParam("buff_location/x", buff_location_[0].pose.position.x);
      nh.getParam("buff_location/y", buff_location_[0].pose.position.y);
      nh.getParam("buff_location/z", buff_location_[0].pose.position.z);

      double tmp_roll, tmp_pitch, tmp_yaw;
      nh.getParam("buff_location/roll", tmp_roll);
      nh.getParam("buff_location/pitch", tmp_pitch);
      nh.getParam("buff_location/yaw", tmp_yaw);
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(tmp_roll, tmp_pitch, tmp_yaw);
      buff_location_[0].pose.orientation.x = quaternion.x();
      buff_location_[0].pose.orientation.y = quaternion.y();
      buff_location_[0].pose.orientation.z = quaternion.z();
      buff_location_[0].pose.orientation.w = quaternion.w();
    }

    // Read the near enemy supply and buff location:
    near_enemy_supplier_location_.resize(1);
    {
      near_enemy_supplier_location_[0].header.frame_id = "map";
      nh.getParam("near_enemy_supplier_location/x", near_enemy_supplier_location_[0].pose.position.x);
      nh.getParam("near_enemy_supplier_location/y", near_enemy_supplier_location_[0].pose.position.y);
      nh.getParam("near_enemy_supplier_location/z", near_enemy_supplier_location_[0].pose.position.z);

      double tmp_roll, tmp_pitch, tmp_yaw;
      nh.getParam("near_enemy_supplier_location/roll", tmp_roll);
      nh.getParam("near_enemy_supplier_location/pitch", tmp_pitch);
      nh.getParam("near_enemy_supplier_location/yaw", tmp_yaw);
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(tmp_roll, tmp_pitch, tmp_yaw);
      near_enemy_supplier_location_[0].pose.orientation.x = quaternion.x();
      near_enemy_supplier_location_[0].pose.orientation.y = quaternion.y();
      near_enemy_supplier_location_[0].pose.orientation.z = quaternion.z();
      near_enemy_supplier_location_[0].pose.orientation.w = quaternion.w();
    }
    // Read the supply location:
    near_enemy_buff_location_.resize(1);
    {
      near_enemy_buff_location_[0].header.frame_id = "map";
      nh.getParam("near_enemy_buff_location/x", near_enemy_buff_location_[0].pose.position.x);
      nh.getParam("near_enemy_buff_location/y", near_enemy_buff_location_[0].pose.position.y);
      nh.getParam("near_enemy_buff_location/z", near_enemy_buff_location_[0].pose.position.z);

      double tmp_roll, tmp_pitch, tmp_yaw;
      nh.getParam("near_enemy_buff_location/roll", tmp_roll);
      nh.getParam("near_enemy_buff_location/pitch", tmp_pitch);
      nh.getParam("near_enemy_buff_location/yaw", tmp_yaw);
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(tmp_roll, tmp_pitch, tmp_yaw);
      near_enemy_buff_location_[0].pose.orientation.x = quaternion.x();
      near_enemy_buff_location_[0].pose.orientation.y = quaternion.y();
      near_enemy_buff_location_[0].pose.orientation.z = quaternion.z();
      near_enemy_buff_location_[0].pose.orientation.w = quaternion.w();
    }

    global_planner_actionlib_client_.waitForServer();
    ROS_INFO("Global planer server start!");
    bullet_supply_actionlib_client_.waitForServer();
    ROS_INFO("Bullet supply server start!");
    local_planner_actionlib_client_.waitForServer();
    ROS_INFO("Local planer server start!");
    /******leonard add******/
    turn_angle_actionlib_client_.waitForServer();
    ROS_INFO("Turn back server start!");

    // master-or-wing:
    last_goal_.header.frame_id = "map";
    last_goal_.pose.position.x = 0;
    last_goal_.pose.position.y = 0;
    nh.getParam("is_master", is_master_);
    ROS_INFO("create send goal client");
    if (is_master_) {
      goal_pub_ = nh.advertise<icra_decision::GoalTask>("/master/goal_task", 2);
      sync_state_pub_ = nh.advertise<std_msgs::Int32>("/master/sync_state", 2);
      ROS_INFO("create some special master sync topics");
    } else {
      goal_pub_ = nh.advertise<icra_decision::GoalTask>("/wing/goal_task", 2);
      sync_state_pub_ = nh.advertise<std_msgs::Int32>("/wing/sync_state", 2);
      ROS_INFO("create some special wing sync topics");
    }
  }
  ~GoalFactory() = default;

  void CancelGoal() {
    ROS_INFO("Cancel Goal!");
    global_planner_actionlib_client_.cancelGoal();
    local_planner_actionlib_client_.cancelGoal();
    action_state_ = BehaviorState::IDLE;
    blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
  }

  void CancelBulletGoal() {
    ROS_INFO("Cancel Supply!");
    bullet_supply_actionlib_client_.cancelGoal();
    action_state_ = BehaviorState::IDLE;
    blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
  }

  BehaviorState Whirl() {
    blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
    blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELAX);
    angle_vel_pub_.publish(whirl_vel_);
    return BehaviorState::RUNNING;
  }

  void CancelWhirl() {
    ROS_INFO("Cancel Whirl!");
    geometry_msgs::Twist zero_angle_vel;
    zero_angle_vel.linear.x = 0;
    zero_angle_vel.linear.y = 0;
    zero_angle_vel.angular.z = 0;
    angle_vel_pub_.publish(zero_angle_vel);
  }

  void PatrolGoal() {
    if (action_state_ != BehaviorState::RUNNING) {
      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      if (patrol_goals_.empty()) {
        ROS_ERROR("Patrol goal is empty");
        return;
      }
      SendGoal(patrol_goals_[patrol_count_]);
      patrol_count_ = ++patrol_count_ % point_size_;
      SendGoalTask(patrol_goals_[patrol_count_]);
      patrol_count_ = ++patrol_count_ % point_size_;
    }
  }

  void SendGoal(geometry_msgs::PoseStamped goal) {
    global_planner_goal_.goal = goal;
    global_planner_actionlib_client_.sendGoal(global_planner_goal_,
                                              GlobalActionClient::SimpleDoneCallback(),
                                              GlobalActionClient::SimpleActiveCallback(),
                                              boost::bind(&GoalFactory::GlobalPlannerFeedbackCallback, this, _1));
  }

  void GlobalPlannerFeedbackCallback(const GlobalFeedback& feedback){
    if (!feedback->path.poses.empty()) {
      local_planner_goal_.route = feedback->path;
      // Change the local planner's velocity:
      local_planner_goal_.maxvelx = 1.5;
      local_planner_goal_.maxvely = 1.5;
      local_planner_goal_.delta = 0.03;
      local_planner_actionlib_client_.sendGoal(local_planner_goal_);
    }
  }

  void UpdateActionState(){
    auto state = global_planner_actionlib_client_.getState();
    if (state == actionlib::SimpleClientGoalState::ACTIVE){
      action_state_ = BehaviorState::RUNNING;
    } else if (state == actionlib::SimpleClientGoalState::PENDING) {
      action_state_ = BehaviorState::RUNNING;
    } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      action_state_ = BehaviorState::SUCCESS;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      action_state_ = BehaviorState::FAILURE;
    } else {
      action_state_ = BehaviorState::FAILURE;
    }
  }

  // Bullet Supply Action State
  void UpdateBulletState(){
    auto state = bullet_supply_actionlib_client_.getState();
    if (state == actionlib::SimpleClientGoalState::ACTIVE){
      action_state_ = BehaviorState::RUNNING;
    } else if (state == actionlib::SimpleClientGoalState::PENDING) {
      action_state_ = BehaviorState::RUNNING;
    } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      action_state_ = BehaviorState::SUCCESS;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      action_state_ = BehaviorState::FAILURE;
    } else {
      action_state_ = BehaviorState::FAILURE;
    }
  }

  BehaviorState GetActionState() {
    return action_state_;
  }

  bool SearchValid() {
    return (search_count_ > 0);
  }
  void CancelSearch() {
    search_count_ = 0;
  }
  void SearchGoal () {
    double yaw;
    double x_diff;
    double y_diff;
    if (action_state_ != BehaviorState::RUNNING) {
      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      if (search_count_ == 5) {
        x_diff = enemy_buffer_.pose.position.x - robot_map_pose_.pose.position.x;
        y_diff = enemy_buffer_.pose.position.y - robot_map_pose_.pose.position.y;
        auto enemy_x = enemy_buffer_.pose.position.x;
        auto enemy_y = enemy_buffer_.pose.position.y;
        if (enemy_x < 4.2 && enemy_y < 2.75) {
          search_region_ = search_region_1_;
        } else if (enemy_x > 4.2 && enemy_y < 2.75) {
          search_region_ = search_region_2_;
        } else if (enemy_x < 4.2 && enemy_y > 2.75) {
          search_region_ = search_region_3_;
        } else {
          search_region_ = search_region_4_;
        }
        double search_min_dist = 99999;
        // Find the nearest search point to enemy
        for (int i = 0; i < search_region_.size(); ++i) {
          auto dist_sq = std::pow(search_region_[i].pose.position.x - enemy_x, 2)
              + std::pow(search_region_[i].pose.position.y - enemy_y, 2);
          if (dist_sq < search_min_dist) {
            search_min_dist = dist_sq;
            search_index_ = i;
          }
        }
        yaw = std::atan2(y_diff, x_diff);
        auto orientation = tf::createQuaternionMsgFromYaw(yaw);
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position = enemy_buffer_.pose.position;
        goal.pose.orientation = orientation;
        SendGoalTask(goal);
        SendGoal(goal);
        lost_enemy_ = true;
        search_count_--;
      } else if (search_count_ > 0) {
        auto search_goal = search_region_[(search_index_++)];
        SendGoal(search_goal);
        search_index_ = search_index_% search_region_.size();
        search_count_--;

        search_goal = search_region_[(search_index_++)];
        SendGoalTask(search_goal);
        search_index_ = search_index_% search_region_.size();
        search_count_--;
      }
    }
  }

  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();

    robot_tf_pose.frame_id_ = "base_link";
    robot_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }

  bool CheckRobotInEnemyField() {
    UpdateRobotPose();
    auto x_curr = robot_map_pose_.pose.position.x;
    auto y_curr = robot_map_pose_.pose.position.y;
    // 处于敌人补给区域
    if ((x_curr > enemy_field_supplier_x_ && x_curr < (enemy_field_supplier_x_ + 1.5)) &&
        (y_curr > enemy_field_supplier_y_ && y_curr < (enemy_field_supplier_y_ + 1.5))) {
          return true;
        }
    // 处于敌人BUFF区域
    if ((x_curr > (enemy_field_buff_x_ - 0.3) && x_curr < (enemy_field_buff_x_ + 1.3)) &&
        (y_curr > (enemy_field_buff_y_ - 0.3) && y_curr < (enemy_field_buff_y_ + 1.3))) {
          return true;
        }
    return false;
  }

  void ChaseGoal() {
    if (action_state_ != BehaviorState::RUNNING) {
      enemy_buffer_ = blackboard_ptr_->GetFrontEnemyPose();
      if (lost_enemy_) {
        lost_enemy_ = false;
        search_count_ = 5;
      }
      UpdateRobotPose();
      auto dx = enemy_buffer_.pose.position.x - robot_map_pose_.pose.position.x;
      auto dy = enemy_buffer_.pose.position.y - robot_map_pose_.pose.position.y;
      auto yaw = std::atan2(dy, dx);

      // Call Wing Robot for Cooperation:
      if (is_master_) {
        auto wing_bot_goal = blackboard_ptr_->GetFrontEnemyPose();
        auto wing_bot_goal_x = wing_bot_goal.pose.position.x;
        auto wing_bot_goal_y = wing_bot_goal.pose.position.y;
        for (int i = 20; i < 340; i += 5) {
          auto theta = (i / 180.) * M_PI;
          auto x = 1 * cos(theta); // TODO: 2m->1m
          auto y = 1 * sin(theta);
          auto world_x = wing_bot_goal_x + x * cos(yaw) - y * sin(yaw);
          auto world_y = wing_bot_goal_y + x * sin(yaw) + y * cos(yaw);
          unsigned int wing_cell_x;
          unsigned int wing_cell_y;
          auto get_wing_cell = costmap_2d_->World2Map(world_x, world_y, wing_cell_x, wing_cell_y);
          if (!get_wing_cell)
            continue;
          auto index = costmap_2d_->GetIndex(wing_cell_x, wing_cell_x);
          if (charmap_[index]< 253) {
            wing_bot_goal.pose.position.x = world_x;
            wing_bot_goal.pose.position.y = world_y;
            wing_bot_goal.pose.orientation = tf::createQuaternionMsgFromYaw(-theta + yaw);
            break;
          }
        } // end for loop to find a point 
        SendGoalTask(wing_bot_goal);
        // SendGoal(blackboard_ptr_->GetFrontEnemyPose());
      } // end for Cooperation.
      
      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) >= 1.0 && std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2.0) {
        // Enough Close to Enemy, range: (1m ~ 2m)
        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
        blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);
        if (switch_mode_) {
          // what the function of "switch_mode": true(far away); false(clost)?
          global_planner_actionlib_client_.cancelGoal();
          local_planner_actionlib_client_.cancelGoal();
          switch_mode_ = false;
        }
        action_state_ = BehaviorState::SUCCESS;
        return;
      } else {
        // If too close to or far away from ememy, find a new goal:
        auto orientation = tf::createQuaternionMsgFromYaw(yaw);
        geometry_msgs::PoseStamped reduce_goal;
        reduce_goal.pose.orientation = robot_map_pose_.pose.orientation;
        reduce_goal.header.frame_id = "map";
        reduce_goal.header.stamp = ros::Time::now();
        // Keep a appropriate distance from the enemy:
        // What happen if very closed to enemy and can not find it ?!
        reduce_goal.pose.position.x = enemy_buffer_.pose.position.x - 1.5 * cosf(yaw);
        reduce_goal.pose.position.y = enemy_buffer_.pose.position.y - 1.5 * sinf(yaw);
        auto enemy_x = reduce_goal.pose.position.x;
        auto enemy_y = reduce_goal.pose.position.y;
        reduce_goal.pose.position.z = 1;
        unsigned int goal_cell_x, goal_cell_y;
        auto get_enemy_cell = costmap_ptr_->GetCostMap()->World2Map(enemy_x, enemy_y, goal_cell_x, goal_cell_y);
        if (!get_enemy_cell)
          return;
        auto robot_x = robot_map_pose_.pose.position.x;
        auto robot_y = robot_map_pose_.pose.position.y;
        unsigned int robot_cell_x, robot_cell_y;
        double goal_x, goal_y;
        costmap_ptr_->GetCostMap()->World2Map(robot_x, robot_y, robot_cell_x, robot_cell_y);
        if (costmap_2d_->GetCost(goal_cell_x, goal_cell_y) >= 253) {
          // It is a Obstacle:
          bool find_goal = false;
          // for loop:
          for(FastLineIterator line( goal_cell_x, goal_cell_y, robot_cell_x, robot_cell_x); line.IsValid(); line.Advance()) {
            auto point_cost = costmap_2d_->GetCost(line.GetX(), line.GetY()); //current point's cost
            if(point_cost >= 253){
              // Still a Obstacle! Try Next Loop
              continue;
            } else {
              // Find free space for Goal:
              find_goal = true;
              costmap_ptr_->GetCostMap()->Map2World(line.GetX(), line.GetY(), goal_x, goal_y);
              reduce_goal.pose.position.x = goal_x;
              reduce_goal.pose.position.y = goal_y;
              break; // end for loop
            }
          }
          if (find_goal) {
            // find a goal:
            switch_mode_ = true;
            blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
            blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
            SendGoal(reduce_goal);
          } else {
            // can not find a goal, still dodge mode and shoot
            blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
            blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);
            action_state_ = BehaviorState::SUCCESS;
            if (switch_mode_) {
              global_planner_actionlib_client_.cancelGoal();
              local_planner_actionlib_client_.cancelGoal();
              switch_mode_ = false;
            }
            return;
          }
        } else {
          // It is Free space, just go to the goal
          switch_mode_ = true;
          blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
          blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
          SendGoal(reduce_goal);
        }
      }
    }
    UpdateActionState();
  }

  void EscapeGoal() {
    if (action_state_ != BehaviorState::RUNNING) {
      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      if (blackboard_ptr_->GetFrontEnemyDetected()) {
        // If find the enemy:
        geometry_msgs::PoseStamped enemy;
        enemy = blackboard_ptr_->GetFrontEnemyPose();
        float goal_yaw, goal_x, goal_y;
        unsigned int goal_cell_x, goal_cell_y;
        unsigned int enemy_cell_x, enemy_cell_y;

        std::random_device rd;
        std::mt19937 gen(rd());
        UpdateRobotPose();
        float x_min, x_max;
        if (enemy.pose.position.x < left_x_limit_) {
          x_min = right_random_min_x_;
          x_max = right_random_max_x_;
        } else if (enemy.pose.position.x > right_x_limit_) {
          x_min = left_random_min_x_;
          x_max = left_random_max_x_;
        } else {
          if ((robot_x_limit_ - robot_map_pose_.pose.position.x) >= 0) {
            x_min = left_random_min_x_;
            x_max = left_random_max_x_;
          } else {
            x_min = right_random_min_x_;
            x_max = right_random_max_x_;
          }
        }
        std::uniform_real_distribution<float> x_uni_dis(x_min, x_max);
        std::uniform_real_distribution<float> y_uni_dis(0, 5);
        //std::uniform_real_distribution<float> yaw_uni_dis(-M_PI, M_PI);
        auto get_enemy_cell = costmap_ptr_->GetCostMap()->World2Map(enemy.pose.position.x, enemy.pose.position.y, enemy_cell_x, enemy_cell_y);
        if (!get_enemy_cell) {
          angle_vel_pub_.publish(whirl_vel_);
          action_state_ = BehaviorState::SUCCESS;
          return;
        }
        while (true) {
          // Try to find a goal for escape action:
          // (find a goal: FREE) <----------and some obstacles---------Robot@
          goal_x = x_uni_dis(gen);
          goal_y = y_uni_dis(gen);
          auto get_goal_cell = costmap_ptr_->GetCostMap()->World2Map(goal_x, goal_y, goal_cell_x, goal_cell_y);
          if (!get_goal_cell)
            continue;
          auto index = costmap_2d_->GetIndex(goal_cell_x, goal_cell_y);
          if (charmap_[index] >= 253) {
            // obstacle:
            continue;
          }
          unsigned int obstacle_count = 0;
          for(FastLineIterator line( goal_cell_x, goal_cell_y, enemy_cell_x, enemy_cell_y); line.IsValid(); line.Advance()) {
            auto point_cost = costmap_2d_->GetCost(line.GetX(), line.GetY()); //current point's cost
            if(point_cost > 253){
              // Obstacle Counter:
              obstacle_count++;
            }
          }
          if (obstacle_count > 5) {
            // find enough obstacles, and jump out of the while loop:
            break;
          }
        }
        Eigen::Vector2d pose_to_enemy(enemy.pose.position.x - robot_map_pose_.pose.position.x,
                                      enemy.pose.position.y - robot_map_pose_.pose.position.y);
        goal_yaw = static_cast<float>(std::atan2(pose_to_enemy.coeffRef(1), pose_to_enemy.coeffRef(0)));
        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(0,0,goal_yaw);
        geometry_msgs::PoseStamped escape_goal;
        escape_goal.header.frame_id = "map";
        escape_goal.header.stamp = ros::Time::now();
        escape_goal.pose.position.x = goal_x;
        escape_goal.pose.position.y = goal_y;
        escape_goal.pose.orientation = quaternion;
        SendGoal(escape_goal);
      } else {
        // not find the enemy:
        angle_vel_pub_.publish(whirl_vel_);
        action_state_ = BehaviorState::SUCCESS;
        return;
      }
    }
    UpdateActionState();
  }

  void GotoEnemyBuffGoal() {
    if (action_state_ != BehaviorState::RUNNING) {
      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELAX);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      if (near_enemy_buff_location_.empty()) {
        ROS_ERROR("Enemy Buff goal is empty");
        return;
      }
      SendGoal(near_enemy_buff_location_[0]);
    }
  }

  void GotoEnemySupplierGoal() {
    if (action_state_ != BehaviorState::RUNNING) {
      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELAX);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      if (near_enemy_supplier_location_.empty()) {
        ROS_ERROR("Enemy Supplier goal is empty");
        return;
      }
      SendGoal(near_enemy_supplier_location_[0]);
    }
  }

  void RunAwayEnemyFields() {
    if (action_state_ != BehaviorState::RUNNING) {
      // 逃离时也可以射击敌人，并通知另外一台机器人
      if (blackboard_ptr_->GetFrontEnemyDetected()){
        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
        blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
        SendGoalTask(blackboard_ptr_->GetFrontEnemyPose());
      } else {
        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
        blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      }
      // 逃离危险区域
      SendGoal(patrol_goals_[1]);
    }
  }

  void GotoSupplierGoal() {
    if (action_state_ != BehaviorState::RUNNING) {
      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELAX);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      if (supply_location_.empty()) {
        ROS_ERROR("Supplier goal is empty");
        return;
      }
      SendGoal(supply_location_[0]);
      // TODO
      // 一机器人去补给站，让另一机器人在旁边掩护
      SendGoalTask(near_enemy_buff_location_[0]);
    }
  }

  void GotoBuffGoal() {
    if (action_state_ != BehaviorState::RUNNING) {
      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELAX);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      if (buff_location_.empty()) {
        ROS_ERROR("Buff goal is empty");
        return;
      }
      SendGoal(buff_location_[0]);
    }
  }

  void TurnBack()
  {
    if (action_state_ != BehaviorState::RUNNING)
    {
      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      double yaw;
      // Turn to BACK:
      yaw = M_PI;

      /*****leonard modify***/
      // geometry_msgs::PoseStamped new_pose;
      // auto quaternion = tf::createQuaternionMsgFromYaw(yaw);
      // new_pose.header.frame_id = "base_link";
      // new_pose.header.stamp = ros::Time::now();
      // new_pose.pose.orientation = quaternion;
      // SendGoal(new_pose);

      icra_decision::TurnAngleGoal goal;
      goal.angle = yaw;
      turn_angle_actionlib_client_.sendGoal(goal);
    }
  }

  /*********leonard add**********/
  void CancelTurnAngleGoal()
  {
    turn_angle_actionlib_client_.cancelGoal();
    action_state_ = BehaviorState::IDLE;
  }

  /*********leonard add**********/
  void UpdateTurnAngleState()
  {
    auto state = turn_angle_actionlib_client_.getState();
    if (state == actionlib::SimpleClientGoalState::ACTIVE) {
      action_state_ = BehaviorState::RUNNING;
    } else if (state == actionlib::SimpleClientGoalState::PENDING) {
      action_state_ = BehaviorState::RUNNING;
    } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      action_state_ = BehaviorState::SUCCESS;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      action_state_ = BehaviorState::FAILURE;
    } else{
      action_state_ = BehaviorState::FAILURE;
    }
  }

  // Bullet Supply Using PID & AprilTag
  void AddBullet() {
    if (action_state_ != BehaviorState::RUNNING) {
      // 补弹时也可以射击敌人，并通知另外一台机器人
      if (blackboard_ptr_->GetFrontEnemyDetected()){
        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
        blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
        SendGoalTask(blackboard_ptr_->GetFrontEnemyPose());
      } else {
        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
        blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      }

      icra_decision::SupplyPidGoal goal;
      goal.command = 1;
      bullet_supply_actionlib_client_.sendGoal(goal);
    }
  }

  // 发送 Auxiliary 点
  void SendGoalTask(geometry_msgs::PoseStamped goal) {
    auto dx = goal.pose.position.x - last_goal_.pose.position.x;
    auto dy = goal.pose.position.y - last_goal_.pose.position.y;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(goal.pose.orientation, rot1);
    tf::quaternionMsgToTF(last_goal_.pose.orientation, rot2);
    auto d_yaw =  rot1.angleShortestPath(rot2);

    if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) < 0.8) {
      return;
    }
    icra_decision::GoalTask goal_task_msg;
    goal_task_msg.goal = goal;
    goal_pub_.publish(goal_task_msg);
    ROS_INFO("SendGoalTask: W/M <------> M/W");
    last_goal_ = goal;
  }
  /*发送同步消息:
      0: free
      1: supply busy!
      2: buff busy!*/
  void SendSyncState(const SyncStateType& sync_type) {
    std_msgs::Int32 sync_msg;
    sync_msg.data = static_cast<int32_t>(sync_type);
    sync_state_pub_.publish(sync_msg);
  }

  void GoAuxiliaryPosition() {
    if (action_state_ != BehaviorState::RUNNING) {
      if (blackboard_ptr_->GetFrontEnemyDetected()) {
        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
      } else {
        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
      }
      //blackboard_ptr_->SetArrive(false);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      SendGoal(blackboard_ptr_->GetAuxiliaryPosition());
    }
  }

  void TurnToWoundedArmor() {
    if (action_state_ != BehaviorState::RUNNING) {
      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      double yaw;
      switch (blackboard_ptr_->GetArmorAttacked()){
        case ArmorAttacked::FRONT:
          ROS_INFO("Hurt@FRONT");
          break;
        case ArmorAttacked::LEFT:
          yaw = M_PI/2.;
          ROS_INFO("Hurt@LEFT");
          break;
        case ArmorAttacked::BACK:
          yaw = M_PI;
          ROS_INFO("Hurt@BACK");
          break;
        case ArmorAttacked::RIGHT:
          yaw = -M_PI/2.;
          ROS_INFO("Hurt@RIGHT");
          break;
        default:
          return;
      }
      geometry_msgs::PoseStamped hurt_pose;
      auto quaternion = tf::createQuaternionMsgFromYaw(yaw);
      hurt_pose.header.frame_id="base_link";
      hurt_pose.header.stamp=ros::Time::now();
      hurt_pose.pose.orientation=quaternion;
      SendGoal(hurt_pose);
    }
  }
  
  // Definition of FastLineIterator
  class FastLineIterator {
  // this method is a modified version of base_local_planner/line_iterator.h
  public:
    FastLineIterator( int x0, int y0, int x1, int y1 )
      : x0_( x0 )
      , y0_( y0 )
      , x1_( x1 )
      , y1_( y1 )
      , x_( x0 )
      , y_( y0 )
      , deltax_(abs(x1 - x0))
      , deltay_(abs(y1 - y0))
      , curpixel_( 0 ) {
      xinc1_ = (x1 - x0) >0 ?1:-1;
      xinc2_ = (x1 - x0) >0 ?1:-1;
      yinc1_ = (y1 - y0) >0 ?1:-1;
      yinc2_ = (y1 - y0) >0 ?1:-1;

      if( deltax_ >= deltay_ ) {
        xinc1_ = 0;
        yinc2_ = 0;
        den_ = 2 * deltax_;
        num_ = deltax_;
        numadd_ = 2 * deltay_;
        numpixels_ = deltax_;
      } else {
        xinc2_ = 0;
        yinc1_ = 0;
        den_ = 2 * deltay_;
        num_ = deltay_;
        numadd_ = 2 * deltax_;
        numpixels_ = deltay_;
      }
    }
    ~FastLineIterator() = default;
    bool IsValid() const {
      return curpixel_ <= numpixels_;
    }

    void Advance() {
      num_ += numadd_;
      if( num_ >= den_ ) {
        num_ -= den_;
        x_ += xinc1_;
        y_ += yinc1_;
      }
      x_ += xinc2_;
      y_ += yinc2_;
      curpixel_++;
    }

    int GetX() const { return x_; }
    int GetY() const { return y_; }
    int GetX0() const { return x0_; }
    int GetY0() const { return y0_; }
    int GetX1() const { return x1_; }
    int GetY1() const { return y1_; }

  private:
    int x0_;
    int y0_;
    int x1_;
    int y1_;
    int x_;
    int y_;
    int deltax_;
    int deltay_;
    int curpixel_;
    int xinc1_, xinc2_, yinc1_, yinc2_;
    int den_, num_, numadd_, numpixels_;
  }; // end for class FastLineIterator

public:
  geometry_msgs::PoseStamped enemy_buffer_;
  bool lost_enemy_;
  geometry_msgs::PoseStamped robot_map_pose_;
  bool switch_mode_;

  // CostMap
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;
  // TF
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  ros::NodeHandle nh;
  ros::Publisher angle_vel_pub_;
  BehaviorState action_state_;
  Blackboard::Ptr blackboard_ptr_;

  GlobalGoal global_planner_goal_;
  LocalGoal local_planner_goal_;
  LocalActionClient local_planner_actionlib_client_;
  GlobalActionClient global_planner_actionlib_client_;
  // Add Bullet
  actionlib::SimpleActionClient<icra_decision::SupplyPidAction> bullet_supply_actionlib_client_;
  /********leonard add*********/
  actionlib::SimpleActionClient<icra_decision::TurnAngleAction> turn_angle_actionlib_client_;

  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  int point_size_;
  unsigned int patrol_count_;

  // Escape Configurations
  float left_x_limit_, right_x_limit_;
  float robot_x_limit_;
  float left_random_min_x_, left_random_max_x_;
  float right_random_min_x_, right_random_max_x_;
  geometry_msgs::Twist whirl_vel_;

  // Search Configurations
  unsigned int search_count_;
  std::vector<geometry_msgs::PoseStamped> search_region_1_;
  std::vector<geometry_msgs::PoseStamped> search_region_2_;
  std::vector<geometry_msgs::PoseStamped> search_region_3_;
  std::vector<geometry_msgs::PoseStamped> search_region_4_;
  std::vector<geometry_msgs::PoseStamped> search_region_;
  unsigned int search_index_;

  // Supplier location
  std::vector<geometry_msgs::PoseStamped> supply_location_;
  // Buff location
  std::vector<geometry_msgs::PoseStamped> buff_location_;
  // Near Enemhy Supplier Location
  std::vector<geometry_msgs::PoseStamped> near_enemy_supplier_location_;
  // Near Enemy Buff location
  std::vector<geometry_msgs::PoseStamped> near_enemy_buff_location_;

  bool is_master_;
  ros::Publisher goal_pub_;
  geometry_msgs::PoseStamped last_goal_;

  // 多机器人数据同步
  ros::Publisher sync_state_pub_;

  // Enemy Field (Left Corner)
  float enemy_field_supplier_x_;
  float enemy_field_supplier_y_;
  float enemy_field_buff_x_;
  float enemy_field_buff_y_;
};
} // namespace decision
#endif //MODULE_DECISION_ICRA_GOAL_FACORY_H
