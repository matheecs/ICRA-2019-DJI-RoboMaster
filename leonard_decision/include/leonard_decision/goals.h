#ifndef MODULE_DECISION_ICRA_GOAL_FACORY_H
#define MODULE_DECISION_ICRA_GOAL_FACORY_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "leonard_decision/GlobalPlannerAction.h"
#include "leonard_decision/LocalPlannerAction.h"
#include "leonard_decision/SupplyPidAction.h"
#include "leonard_decision/TurnAngleAction.h"
#include "leonard_decision/TwistAccel.h"
#include "types.h"
// CostMap
#include "costmap/costmap_interface.h"
#include "blackboard.h"

#include "log.h"

namespace decision {

    class GoalFactory {
    public:
        typedef std::shared_ptr<GoalFactory> GoalFactoryPtr;
        typedef actionlib::SimpleActionClient<leonard_decision::LocalPlannerAction> LocalActionClient;
        typedef actionlib::SimpleActionClient<leonard_decision::GlobalPlannerAction> GlobalActionClient;
        typedef leonard_decision::GlobalPlannerFeedbackConstPtr GlobalFeedback;
        typedef leonard_decision::LocalPlannerGoal LocalGoal;
        typedef leonard_decision::GlobalPlannerGoal GlobalGoal;
        // CostMap
        typedef roborts_costmap::CostmapInterface CostMap;
        typedef roborts_costmap::Costmap2D CostMap2D;

        GoalFactory(const Blackboard::Ptr &blackboard_ptr) :
                blackboard_ptr_(blackboard_ptr),
                patrol_count_(0),
                search_count_(0),
                lost_enemy_(true),
                aim_supply_actionlib_client_("supplypid", true),
                turn_angle_actionlib_client_("turnangle", true),
                global_planner_actionlib_client_("global_planner_node_action", true),
                local_planner_actionlib_client_("local_planner_node_action", true) {
            // ctor
            angle_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);

            // CostMap
            ROS_INFO("Create CostMap Start...");
            tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
            std::string map_path =
                    ros::package::getPath("roborts_costmap") + "/config/costmap_parameter_config_for_decision.prototxt";
            costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_, map_path);
            charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();
            costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();
            ROS_INFO("Adding CostMap...Done!");

            // Load the configurations:
            nh.getParam("point_size", point_size_);
            ROS_INFO("point_size_%d", point_size_);
            patrol_goals_.resize(point_size_);
            for (int i = 0; i < point_size_; i++) {
                patrol_goals_[i].header.frame_id = "map";
                nh.getParam("point" + std::to_string(i) + "/x", patrol_goals_[i].pose.position.x);
                nh.getParam("point" + std::to_string(i) + "/y", patrol_goals_[i].pose.position.y);
                nh.getParam("point" + std::to_string(i) + "/z", patrol_goals_[i].pose.position.z);

                double tmp_roll, tmp_pitch, tmp_yaw;
                nh.getParam("point" + std::to_string(i) + "/roll", tmp_roll);
                nh.getParam("point" + std::to_string(i) + "/pitch", tmp_pitch);
                nh.getParam("point" + std::to_string(i) + "/yaw", tmp_yaw);
                tf::Quaternion quaternion = tf::createQuaternionFromRPY(tmp_roll, tmp_pitch, tmp_yaw);
                patrol_goals_[i].pose.orientation.x = quaternion.x();
                patrol_goals_[i].pose.orientation.y = quaternion.y();
                patrol_goals_[i].pose.orientation.z = quaternion.z();
                patrol_goals_[i].pose.orientation.w = quaternion.w();
            }


            nh.getParam("inner_point_size", inner_point_size_);
            std::cout << "inner_point_size_" << inner_point_size_ << std::endl;
            inner_patrol_goals_.resize(inner_point_size_);
            for (int i = 0; i < inner_point_size_; i++) {
                inner_patrol_goals_[i].header.frame_id = "map";
                nh.getParam("inner_point" + std::to_string(i) + "/x", inner_patrol_goals_[i].pose.position.x);
                nh.getParam("inner_point" + std::to_string(i) + "/y", inner_patrol_goals_[i].pose.position.y);
                nh.getParam("inner_point" + std::to_string(i) + "/z", inner_patrol_goals_[i].pose.position.z);

                double tmp_roll, tmp_pitch, tmp_yaw;
                nh.getParam("inner_point" + std::to_string(i) + "/roll", tmp_roll);
                nh.getParam("inner_point" + std::to_string(i) + "/pitch", tmp_pitch);
                nh.getParam("inner_point" + std::to_string(i) + "/yaw", tmp_yaw);
                tf::Quaternion quaternion = tf::createQuaternionFromRPY(tmp_roll, tmp_pitch, tmp_yaw);
                inner_patrol_goals_[i].pose.orientation.x = quaternion.x();
                inner_patrol_goals_[i].pose.orientation.y = quaternion.y();
                inner_patrol_goals_[i].pose.orientation.z = quaternion.z();
                inner_patrol_goals_[i].pose.orientation.w = quaternion.w();
            }

            nh.getParam("wait_point_size", wait_point_size_);
            std::cout << "wait_point_size: " << wait_point_size_ << std::endl;
            wait_patrol_goals_.resize(wait_point_size_);
            for (int i = 0; i < wait_point_size_; i++) {
                wait_patrol_goals_[i].header.frame_id = "map";
                nh.getParam("wait_point" + std::to_string(i) + "/x", wait_patrol_goals_[i].pose.position.x);
                nh.getParam("wait_point" + std::to_string(i) + "/y", wait_patrol_goals_[i].pose.position.y);
                nh.getParam("wait_point" + std::to_string(i) + "/z", wait_patrol_goals_[i].pose.position.z);

                double tmp_roll, tmp_pitch, tmp_yaw;
                nh.getParam("wait_point" + std::to_string(i) + "/roll", tmp_roll);
                nh.getParam("wait_point" + std::to_string(i) + "/pitch", tmp_pitch);
                nh.getParam("wait_point" + std::to_string(i) + "/yaw", tmp_yaw);
                tf::Quaternion quaternion = tf::createQuaternionFromRPY(tmp_roll, tmp_pitch, tmp_yaw);
                wait_patrol_goals_[i].pose.orientation.x = quaternion.x();
                wait_patrol_goals_[i].pose.orientation.y = quaternion.y();
                wait_patrol_goals_[i].pose.orientation.z = quaternion.z();
                wait_patrol_goals_[i].pose.orientation.w = quaternion.w();
            }


            // Load escape parameters:
            nh.getParam("escape/left_x_limit", left_x_limit_);
            nh.getParam("escape/right_x_limit", right_x_limit_);
            nh.getParam("escape/robot_x_limit", robot_x_limit_);
            nh.getParam("escape/left_random_min_x", left_random_min_x_);
            nh.getParam("escape/left_random_max_x", left_random_max_x_);
            nh.getParam("escape/right_random_min_x", right_random_min_x_);
            nh.getParam("escape/right_random_max_x", right_random_max_x_);

            std::cout << "escape done" << std::endl;

            nh.getParam("whirl_vel/angle_x_vel", whirl_vel_.angular.x);
            nh.getParam("whirl_vel/angle_y_vel", whirl_vel_.angular.y);
            nh.getParam("whirl_vel/angle_z_vel", whirl_vel_.angular.z);

            std::cout << "whirl_vel done" << std::endl;

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

            std::cout << "search region done" << std::endl;

            // Read the supply location:
            supply_location_.header.frame_id = "map";
            nh.getParam("supply_location/x", supply_location_.pose.position.x);
            nh.getParam("supply_location/y", supply_location_.pose.position.y);
            supply_location_.pose.position.z = 0;
            double tmp_roll = 0, tmp_pitch = 0, tmp_yaw;
            nh.getParam("supply_location/yaw", tmp_yaw);
            tf::Quaternion quaternion = tf::createQuaternionFromRPY(tmp_roll, tmp_pitch, tmp_yaw);
            supply_location_.pose.orientation.x = quaternion.x();
            supply_location_.pose.orientation.y = quaternion.y();
            supply_location_.pose.orientation.z = quaternion.z();
            supply_location_.pose.orientation.w = quaternion.w();

            std::cout << "supply location done" << std::endl;


            global_planner_actionlib_client_.waitForServer();
            ROS_INFO("Global planer server start!");
            aim_supply_actionlib_client_.waitForServer();
            ROS_INFO("Bullet supply server start!");
            local_planner_actionlib_client_.waitForServer();
            ROS_INFO("Local planer server start!");
            /******leonard add******/
            turn_angle_actionlib_client_.waitForServer();
            ROS_INFO("Turn back server start!");

            nh.getParam("is_master", is_master_);


            planner_state_pub_ = nh.advertise<std_msgs::Bool>("planner_state", 30);
            sub_cmd_vel_acc_ = nh.subscribe("cmd_vel_acc", 1, &GoalFactory::ChassisSpeedAccRecordCallback, this);

        }

        ~GoalFactory() {
            speed_record_.clear();
            speed_record_.shrink_to_fit();
        }


        void ChassisSpeedAccRecordCallback(const leonard_decision::TwistAccel::ConstPtr &vel_acc) {
            Vec3d speed(vel_acc->twist.linear.x, vel_acc->twist.linear.y, vel_acc->twist.angular.z);
            if (speed_record_.size() == 8) {
                speed_record_.erase(speed_record_.begin());
            }
            speed_record_.push_back(speed);
        }

        /***************! planner !*******************/
        void SendPlannerGoal(geometry_msgs::PoseStamped goal, float maxvel = 1.5) {
//            std::cout << "SendPlannerGoal" << goal.pose.position.x << "," << goal.pose.position.y << std::endl;
            ROS_INFO("SendPlannerGoal:%f,%f", goal.pose.position.x, goal.pose.position.y);
            SetLocalPlannerMaxVel(maxvel);
            global_planner_goal_.goal = goal;
            global_planner_actionlib_client_.sendGoal(global_planner_goal_,
                                                      GlobalActionClient::SimpleDoneCallback(),
                                                      GlobalActionClient::SimpleActiveCallback(),
                                                      boost::bind(&GoalFactory::GlobalPlannerFeedbackCallback, this,
                                                                  _1));
        }

        void UpdatePlannerState() {
            auto state = global_planner_actionlib_client_.getState();
            if (state == actionlib::SimpleClientGoalState::ACTIVE) {
                SetPlannerState(BehaviorState::RUNNING);
            } else if (state == actionlib::SimpleClientGoalState::PENDING) {
                SetPlannerState(BehaviorState::RUNNING);
            } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                SetPlannerState(BehaviorState::SUCCESS);
            } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                SetPlannerState(BehaviorState::FAILURE);
            } else {
                SetPlannerState(BehaviorState::FAILURE);
            }
        }

        void CancelPlannerGoal() {
            //  std::cout<<"Cancel Goal!");
            if (planner_state_ == BehaviorState::RUNNING) {
                global_planner_actionlib_client_.cancelGoal();
                local_planner_actionlib_client_.cancelGoal();
                SetPlannerState(BehaviorState::IDLE);
            }
        }

        BehaviorState GetPlannerState() {
            return planner_state_;
        }

        void GlobalPlannerFeedbackCallback(const GlobalFeedback &feedback) {
            if (!feedback->path.poses.empty()) {
                local_planner_goal_.route = feedback->path;
                // Change the local planner's velocity:
//                local_planner_goal_.maxvelx = 1.5;
//                local_planner_goal_.maxvely = 1.5;
//                local_planner_goal_.delta = 0.05;
                local_planner_actionlib_client_.sendGoal(local_planner_goal_);
            }
        }


        void SetLocalPlannerMaxVel(float max = 1.5) {
            local_planner_goal_.maxvelx = max;
            local_planner_goal_.maxvely = max;
        }


        void SetPlannerState(BehaviorState state) {
            planner_state_ = state;
            // std_msgs::Bool msg;
            // if (state == BehaviorState::RUNNING) {
            //     msg.data = true;
            // } else {
            //     msg.data = false;
            // }
            // planner_state_pub_.publish(msg);
        }


        /***************! Aim supply !*******************/
        void UpdateAimSupplyState() {
            auto state = aim_supply_actionlib_client_.getState();
            if (state == actionlib::SimpleClientGoalState::ACTIVE) {
                aim_supply_state_ = BehaviorState::RUNNING;
            } else if (state == actionlib::SimpleClientGoalState::PENDING) {
                aim_supply_state_ = BehaviorState::RUNNING;
            } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                aim_supply_state_ = BehaviorState::SUCCESS;
            } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                aim_supply_state_ = BehaviorState::FAILURE;
            } else {
                aim_supply_state_ = BehaviorState::FAILURE;
            }
        }

        void CancelAimSupplyGoal() {
            if (aim_supply_state_ == BehaviorState::RUNNING) {
                //std::cout << "Cancel AimSupply Goal!" << std::endl;
                ROS_INFO("Cancel AimSupply Goal!");
                aim_supply_actionlib_client_.cancelGoal();
                aim_supply_state_ = BehaviorState::IDLE;
            }
        }

        BehaviorState GetAimSupplyState() {
            return aim_supply_state_;
        }

        /********* ! Turn Angle !**********/

        void TurnAngleTask(double yaw = M_PI) {
            if (turn_angle_state_ != BehaviorState::RUNNING) {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
                blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

                leonard_decision::TurnAngleGoal goal;
                goal.angle = yaw;
                turn_angle_actionlib_client_.sendGoal(goal);
            }
        }

        void UpdateTurnAngleState() {
            auto state = turn_angle_actionlib_client_.getState();
            if (state == actionlib::SimpleClientGoalState::ACTIVE) {
                turn_angle_state_ = BehaviorState::RUNNING;

            } else if (state == actionlib::SimpleClientGoalState::PENDING) {
                turn_angle_state_ = BehaviorState::RUNNING;

            } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                turn_angle_state_ = BehaviorState::SUCCESS;

            } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                turn_angle_state_ = BehaviorState::FAILURE;

            } else {
                turn_angle_state_ = BehaviorState::FAILURE;

            }
        }

        void CancelTurnAngleGoal() {
            if (turn_angle_state_ == BehaviorState::RUNNING) {
                turn_angle_actionlib_client_.cancelGoal();
                turn_angle_state_ = BehaviorState::IDLE;
            }
        }

        BehaviorState GetTurnAngleState() {
            return turn_angle_state_;
        }
        /************ !!!!!!**************/

        // Bullet Supply using PID:
        void AddBullet() {

            if (blackboard_ptr_->GetEnemyDetected()) {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
            } else {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
            }


            if (aim_supply_state_ != BehaviorState::RUNNING) {
                leonard_decision::SupplyPidGoal goal;
                goal.command = blackboard_ptr_->DecisionSupplierCnt();
                aim_supply_actionlib_client_.sendGoal(goal);
                blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
            }


            static ros::Time tick;
            if (ros::Time().now() - tick > ros::Duration(0.1)) {
                //呼唤同伴
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = 2.2;
                pose.pose.position.y = 4.4;
                tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, -1.11);
                pose.pose.orientation.x = quaternion.x();
                pose.pose.orientation.y = quaternion.y();
                pose.pose.orientation.z = quaternion.z();
                pose.pose.orientation.w = quaternion.w();

                SendGoalTask(pose);
                tick = ros::Time().now();
            }
        }

        void GoBulletGoal(float maxvel = 3.0) {

            if (blackboard_ptr_->GetEnemyDetected()) {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
            } else {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
            }


            if (planner_state_ != BehaviorState::RUNNING) {
                blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
                SendPlannerGoal(supply_location_, maxvel);
            }
        }


        BehaviorState Whirl() {
            angle_vel_pub_.publish(whirl_vel_);
            return BehaviorState::RUNNING;
        }

        void CancelWhirl() {
            // std::cout<<"Cancel Whirl!"<<std::endl;
            geometry_msgs::Twist zero_angle_vel;
            zero_angle_vel.linear.x = 0;
            zero_angle_vel.linear.y = 0;
            zero_angle_vel.angular.z = 0;
            angle_vel_pub_.publish(zero_angle_vel);
        }


        void WingPatrolGoal(float maxvel = 1.5) {
            if (planner_state_ != BehaviorState::RUNNING) {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
                blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
                if (patrol_goals_.empty()) {
                    ROS_ERROR("Patrol goal is empty");
                    return;
                }
                SendPlannerGoal(patrol_goals_[patrol_count_], maxvel);
                patrol_count_ = ++patrol_count_ % point_size_;
            }
        }


        void MasterPatrolGoal(float maxvel = 1.5) {
            if (planner_state_ != BehaviorState::RUNNING) {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
                blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
                if (inner_patrol_goals_.empty()) {
                    ROS_ERROR("Patrol goal is empty");
                    return;
                }
                SendPlannerGoal(inner_patrol_goals_[inner_patrol_count_], maxvel);
                inner_patrol_count_ = ++inner_patrol_count_ % inner_point_size_;
            }
        }


        void WaitPatrolGoal(float maxvel = 1.5) {
            // 原版是两个点绕来绕去
//            if (planner_state_ != BehaviorState::RUNNING) {
//                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
//                blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
//                if (wait_patrol_goals_.empty()) {
//                    ROS_ERROR("Patrol goal is empty");
//                    return;
//                }
//                SendPlannerGoal(wait_patrol_goals_[wait_patrol_count_], maxvel);
//                wait_patrol_count_ = ++wait_patrol_count_ % wait_point_size_;
//            }

            //到一个固定点,再转
            if (planner_state_ != BehaviorState::RUNNING) {
                if (wait_patrol_count_ == 0) {
                    blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
                    blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
                    SendPlannerGoal(wait_patrol_goals_[0], maxvel);
                    wait_patrol_count_ = 1;
                } else {
                    angle_vel_pub_.publish(whirl_vel_);
                    SetPlannerState(BehaviorState::SUCCESS);
                    return;
                }
            }
            UpdatePlannerState();
        }


        void SetWaitPatrolCountZero() {
            wait_patrol_count_ = 0;
        }


        bool SearchValid() {
            return search_count_ > 0;
        }

        void CancelSearch() {
            search_count_ = 0;
        }

        void GoAuxiliaryPosition(float maxvel = 2.5) {
            if (planner_state_ != BehaviorState::RUNNING) {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
                blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

                geometry_msgs::PoseStamped goal = blackboard_ptr_->GetAuxiliaryPosition();
                SendPlannerGoal(goal, maxvel);
                last_goal_ = goal;

                blackboard_ptr_->SetAuxiliaryIsUsed(true);

            } else {
                //说明辅助点换了
                if (blackboard_ptr_->GetAuxiliaryIsUsed() == false) {
                    //如果辅助点变化较大
                    geometry_msgs::PoseStamped goal = blackboard_ptr_->GetAuxiliaryPosition();

                    if (
                            (EularDistance(last_goal_.pose.position.x, goal.pose.position.x,
                                           last_goal_.pose.position.y, goal.pose.position.y) > 1) ||
                            (std::fabs(
                                    angle_diff<double>(GetYawFromGeometryMsgs(last_goal_),
                                                       GetYawFromGeometryMsgs(goal))) > M_PI_2)
                            ) {

                        global_planner_actionlib_client_.cancelGoal();
                        local_planner_actionlib_client_.cancelGoal();
                        SendPlannerGoal(goal, maxvel);
                        last_goal_ = goal;
                    }
                    blackboard_ptr_->SetAuxiliaryIsUsed(true);
                }
            }
        }

        void SearchGoal(float maxvel = 2.5) {
            double yaw;
            double x_diff;
            double y_diff;
            if (planner_state_ != BehaviorState::RUNNING) {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
                blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
                if (search_count_ == 5) {
                    x_diff = enemy_buffer_.pose.position.x - robot_map_pose_.pose.position.x;
                    y_diff = enemy_buffer_.pose.position.y - robot_map_pose_.pose.position.y;

                    auto enemy_x = enemy_buffer_.pose.position.x;
                    auto enemy_y = enemy_buffer_.pose.position.y;

                    if (enemy_x < 4 && enemy_y < 2.5) {
                        search_region_ = search_region_1_;
                    } else if (enemy_x > 4 && enemy_y < 2.5) {
                        search_region_ = search_region_2_;
                    } else if (enemy_x < 4 && enemy_y > 2.5) {
                        search_region_ = search_region_3_;
                    } else {
                        search_region_ = search_region_4_;
                    }

                    double search_min_dist = 99999;
                    // Find the minimal search_index_:
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
                    SendPlannerGoal(goal, maxvel);

                    lost_enemy_ = true;
                    search_count_--;
                } else if (search_count_ > 0) {
                    auto search_goal = search_region_[(search_index_)];
                    SendPlannerGoal(search_goal, maxvel);
                    search_index_ = (++search_index_) % search_region_.size();
                    search_count_--;

//                    search_goal = search_region_[(search_index_)];
//                    SendGoalTask(search_goal);
//                    search_index_ = (++search_index_) % search_region_.size();
//                    search_count_--;
                }
            }
        }

//        void ChaseGoal() {
//            if (planner_state_ != BehaviorState::RUNNING) {
//                enemy_buffer_ = blackboard_ptr_->GetEnemyPose();
//                if (lost_enemy_) {
//                    lost_enemy_ = false;
//                    search_count_ = 5;
//                }
//                UpdateRobotPose();
//
//                auto dx = enemy_buffer_.pose.position.x - robot_map_pose_.pose.position.x;
//                auto dy = enemy_buffer_.pose.position.y - robot_map_pose_.pose.position.y;
//                auto yaw = std::atan2(dy, dx);
//
//                // Wing Robot's Goal:
//                // TODO change 2m to 1m?
////                if (is_master_) {
//                auto wing_bot_goal = blackboard_ptr_->GetEnemyPose();
//                auto wing_bot_goal_x = wing_bot_goal.pose.position.x;
//                auto wing_bot_goal_y = wing_bot_goal.pose.position.y;
//                for (int i = 20; i < 340; i += 5) {
//                    auto theta = (i / 180.) * M_PI;
//                    auto x = 1 * cos(theta); // TODO 2->1
//                    auto y = 1 * sin(theta);
//                    auto world_x = wing_bot_goal_x + x * cos(yaw) - y * sin(yaw);
//                    auto world_y = wing_bot_goal_y + x * sin(yaw) + y * cos(yaw);
//                    unsigned int wing_cell_x;
//                    unsigned int wing_cell_y;
//                    auto get_wing_cell = costmap_2d_->World2Map(world_x, world_y, wing_cell_x, wing_cell_y);
//                    if (!get_wing_cell)
//                        continue;
//                    auto index = costmap_2d_->GetIndex(wing_cell_x, wing_cell_y);
//                    if (charmap_[index] < 253) {
//                        wing_bot_goal.pose.position.x = world_x;
//                        wing_bot_goal.pose.position.y = world_y;
//                        wing_bot_goal.pose.position.z = 1;
//                        wing_bot_goal.pose.orientation = tf::createQuaternionMsgFromYaw(-theta + yaw);
//                        break;
//                    }
//                } // end for loop to find a point
//                SendGoalTask(wing_bot_goal);
//                // SendPlannerGoal(blackboard_ptr_->GetEnemyPose());
////                } // end for if(is_master_)
//
//                if (
//                        std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) >= 0.6 &&
//                        std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2.0) {
//                    // Enough Close to Enemy, range from 1m to 2m:
//                    if (switch_mode_) {
//                        // what the function of "switch_mode": true(far away); false(clost)
//                        global_planner_actionlib_client_.cancelGoal();
//                        local_planner_actionlib_client_.cancelGoal();
//                        switch_mode_ = false;
//                    }
//                    blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
//                    blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);
//                    SetPlannerState(BehaviorState::SUCCESS);
//                    return;
//                } else {
//                    // Find a goal and try close to the enemy:
//                    auto orientation = tf::createQuaternionMsgFromYaw(yaw);
//                    geometry_msgs::PoseStamped reduce_goal;
//                    reduce_goal.pose.orientation = robot_map_pose_.pose.orientation;
//                    reduce_goal.header.frame_id = "map";
//                    reduce_goal.header.stamp = ros::Time::now();
//                    if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) < 0.6) {
//                        reduce_goal.pose.position.x = enemy_buffer_.pose.position.x - 0.8 * cosf(yaw);
//                        reduce_goal.pose.position.y = enemy_buffer_.pose.position.y - 0.8 * sinf(yaw);
//
//                    } else {
//                        reduce_goal.pose.position.x = enemy_buffer_.pose.position.x - 1.2 * cosf(yaw);
//                        reduce_goal.pose.position.y = enemy_buffer_.pose.position.y - 1.2 * sinf(yaw);
//
//
//                    }
//                    auto enemy_x = reduce_goal.pose.position.x;
//                    auto enemy_y = reduce_goal.pose.position.y;
//                    reduce_goal.pose.position.z = 1;
//                    unsigned int goal_cell_x, goal_cell_y;
//                    auto get_enemy_cell = costmap_ptr_->GetCostMap()->World2Map(enemy_x, enemy_y, goal_cell_x,
//                                                                                goal_cell_y);
//                    if (!get_enemy_cell) return;
//                    auto robot_x = robot_map_pose_.pose.position.x;
//                    auto robot_y = robot_map_pose_.pose.position.y;
//                    unsigned int robot_cell_x, robot_cell_y;
//                    double goal_x, goal_y;
//                    costmap_ptr_->GetCostMap()->World2Map(robot_x, robot_y, robot_cell_x, robot_cell_y);
//
//                    if (costmap_2d_->GetCost(goal_cell_x, goal_cell_y) >= 253) {
//                        // It is a Obstacle!
//                        bool find_goal = false;
//                        for (FastLineIterator line(goal_cell_x, goal_cell_y, robot_cell_x,
//                                                   robot_cell_y); line.IsValid(); line.Advance()) {
//                            auto point_cost = costmap_2d_->GetCost(line.GetX(), line.GetY()); //current point's cost
//                            if (point_cost >= 253) {
//                                // Still a Obstacle! Try Next Loop
//                                continue;
//                            } else {
//                                // Find free space for Goal:
//                                find_goal = true;
//                                costmap_ptr_->GetCostMap()->Map2World(line.GetX(), line.GetY(), goal_x, goal_y);
//                                reduce_goal.pose.position.x = goal_x;
//                                reduce_goal.pose.position.y = goal_y;
//                                break; // end for loop
//                            }
//                        }
//                        if (find_goal) {
//                            // find a goal:
//                            switch_mode_ = true;
//                            blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
//                            blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
//                            std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
//                            std::cout << robot_map_pose_.pose.position.x << std::endl;
//                            std::cout << robot_map_pose_.pose.position.y << std::endl;
//                            std::cout << reduce_goal.pose.position.x << std::endl;
//                            std::cout << reduce_goal.pose.position.y << std::endl;
//                            std::cout << enemy_buffer_.pose.position.x << std::endl;
//                            std::cout << enemy_buffer_.pose.position.y << std::endl;
//                            SendPlannerGoal(reduce_goal);
//                        } else {
//                            // can not find a goal, still dodge mode and shoot
//
//                            if (switch_mode_) {
//                                global_planner_actionlib_client_.cancelGoal();
//                                local_planner_actionlib_client_.cancelGoal();
//                                switch_mode_ = false;
//                            }
//
//                            blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
//                            blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);
//                            SetPlannerState(BehaviorState::SUCCESS);
//                            return;
//                        }
//                    } else {
//                        // It is Free space, just go to the goal
//                        switch_mode_ = true;
//                        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
//                        blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
//                        std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
//                        std::cout << robot_map_pose_.pose.position.x << std::endl;
//                        std::cout << robot_map_pose_.pose.position.y << std::endl;
//                        std::cout << reduce_goal.pose.position.x << std::endl;
//                        std::cout << reduce_goal.pose.position.y << std::endl;
//                        std::cout << enemy_buffer_.pose.position.x << std::endl;
//                        std::cout << enemy_buffer_.pose.position.y << std::endl;
//                        SendPlannerGoal(reduce_goal);
//                    }
//                }
//            }
//            UpdatePlannerState();
//        }


        void ChaseGoalCxn(float maxvel = 2.5, float minvel = 1.5) {

            float vel = minvel;

            static int distance_flag = 0;

            if (planner_state_ != BehaviorState::RUNNING) {
                enemy_buffer_ = blackboard_ptr_->GetEnemyPose();
                if (lost_enemy_) {
                    lost_enemy_ = false;
                    search_count_ = 5;
                }
                UpdateRobotPose();

                auto dx = enemy_buffer_.pose.position.x - robot_map_pose_.pose.position.x;
                auto dy = enemy_buffer_.pose.position.y - robot_map_pose_.pose.position.y;
                auto yaw = std::atan2(dy, dx);

                // Another Robot's Goal:
                auto another_bot_goal = blackboard_ptr_->GetEnemyPose();
                auto another_bot_goal_x = another_bot_goal.pose.position.x;
                auto another_bot_goal_y = another_bot_goal.pose.position.y;

                bool find_another_goal = false;
                for (int i = 30; i < 330; i += 5) {
                    auto theta = (i / 180.) * M_PI;
                    auto x = 1 * cos(theta);
                    auto y = 1 * sin(theta);
                    auto world_x = another_bot_goal_x + x * cos(yaw) - y * sin(yaw);
                    auto world_y = another_bot_goal_y + x * sin(yaw) + y * cos(yaw);
                    unsigned int another_cell_x, another_cell_y;
                    auto get_another_cell = costmap_2d_->World2Map(world_x, world_y, another_cell_x, another_cell_y);
                    if (!get_another_cell)
                        continue;
                    auto index = costmap_2d_->GetIndex(another_cell_x, another_cell_y);
                    if (charmap_[index] < 253) {
                        another_bot_goal.pose.position.x = world_x;
                        another_bot_goal.pose.position.y = world_y;
                        another_bot_goal.pose.position.z = 1;
                        another_bot_goal.pose.orientation = tf::createQuaternionMsgFromYaw(-theta + yaw);
                        find_another_goal = true;
                        break;
                    }
                } // end for loop to find a point

                if (!find_another_goal) {
                    unsigned int temp_cell_x, temp_cell_y;
                    auto get_another_enemy_cell = costmap_ptr_->GetCostMap()->World2Map(
                            another_bot_goal.pose.position.x, another_bot_goal.pose.position.y, temp_cell_x,
                            temp_cell_y);
                    if (get_another_enemy_cell) {

                        SendGoalTask(another_bot_goal);

                    }
                } else {
                    SendGoalTask(another_bot_goal);
                }



                // This Robot
                if (
                        std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) >= 0.6 &&
                        std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2.0) {
                    // Enough Close to Enemy, range from 1m to 2m:
                    if (switch_mode_) {
                        // what the function of "switch_mode": true(far away); false(clost)
                        global_planner_actionlib_client_.cancelGoal();
                        local_planner_actionlib_client_.cancelGoal();
                        switch_mode_ = false;
                    }
                    blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
                    blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);
                    SetPlannerState(BehaviorState::SUCCESS);
                    distance_flag = 0;
                    return;
                } else {
                    // Find a goal and try close to the enemy:
                    auto orientation = tf::createQuaternionMsgFromYaw(yaw);
                    geometry_msgs::PoseStamped reduce_goal;
                    reduce_goal.pose.orientation = robot_map_pose_.pose.orientation;
                    reduce_goal.header.frame_id = "map";
                    reduce_goal.header.stamp = ros::Time::now();
                    if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) < 0.6) {
                        distance_flag = 1;
                        reduce_goal.pose.position.x = enemy_buffer_.pose.position.x - 0.8 * cosf(yaw);
                        reduce_goal.pose.position.y = enemy_buffer_.pose.position.y - 0.8 * sinf(yaw);
                        vel = minvel;
                    } else {
                        distance_flag = 2;
                        reduce_goal.pose.position.x = enemy_buffer_.pose.position.x - 1.2 * cosf(yaw);
                        reduce_goal.pose.position.y = enemy_buffer_.pose.position.y - 1.2 * sinf(yaw);
                        vel = maxvel;
                    }
                    auto enemy_x = reduce_goal.pose.position.x;//敌人周围的点
                    auto enemy_y = reduce_goal.pose.position.y;
                    reduce_goal.pose.position.z = 1;
                    unsigned int goal_cell_x, goal_cell_y;
                    auto get_enemy_cell = costmap_ptr_->GetCostMap()->World2Map(enemy_x, enemy_y, goal_cell_x,
                                                                                goal_cell_y);

                    //没有在地图上找到了点
                    if (!get_enemy_cell) {
                        if (switch_mode_) {
                            global_planner_actionlib_client_.cancelGoal();
                            local_planner_actionlib_client_.cancelGoal();
                            switch_mode_ = false;
                        }

                        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
                        blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);
                        SetPlannerState(BehaviorState::SUCCESS);
                        distance_flag = 0;
                        return;
                    }
                    //找到了点
                    auto robot_x = robot_map_pose_.pose.position.x;
                    auto robot_y = robot_map_pose_.pose.position.y;
                    unsigned int robot_cell_x, robot_cell_y;
                    double goal_x, goal_y;
                    costmap_ptr_->GetCostMap()->World2Map(robot_x, robot_y, robot_cell_x, robot_cell_y);

                    //如果目标点在障碍物里
                    if (costmap_2d_->GetCost(goal_cell_x, goal_cell_y) >= 253) {
                        // It is a Obstacle!
                        bool find_goal = false;
                        for (FastLineIterator line(goal_cell_x, goal_cell_y, robot_cell_x,
                                                   robot_cell_y); line.IsValid(); line.Advance()) {
                            auto point_cost = costmap_2d_->GetCost(line.GetX(), line.GetY()); //current point's cost
                            if (point_cost >= 253) {
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
//                            std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
//                            std::cout << robot_map_pose_.pose.position.x << std::endl;
//                            std::cout << robot_map_pose_.pose.position.y << std::endl;
//                            std::cout << reduce_goal.pose.position.x << std::endl;
//                            std::cout << reduce_goal.pose.position.y << std::endl;
//                            std::cout << enemy_buffer_.pose.position.x << std::endl;
//                            std::cout << enemy_buffer_.pose.position.y << std::endl;

                            ROS_INFO("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                            ROS_INFO("%f", robot_map_pose_.pose.position.x);
                            ROS_INFO("%f", robot_map_pose_.pose.position.y);
                            ROS_INFO("%f", reduce_goal.pose.position.x);
                            ROS_INFO("%f", reduce_goal.pose.position.y);
                            ROS_INFO("%f", enemy_buffer_.pose.position.x);
                            ROS_INFO("%f", enemy_buffer_.pose.position.y);

                            SendPlannerGoal(reduce_goal, maxvel);
                        } else {
                            // can not find a goal, still dodge mode and shoot

                            if (switch_mode_) {
                                global_planner_actionlib_client_.cancelGoal();
                                local_planner_actionlib_client_.cancelGoal();
                                switch_mode_ = false;
                            }

                            blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
                            blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);
                            SetPlannerState(BehaviorState::SUCCESS);
                            distance_flag = 0;
                            return;
                        }
                    } else {
                        // It is Free space, just go to the goal
                        switch_mode_ = true;
                        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
                        blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
//                        std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
//                        std::cout << robot_map_pose_.pose.position.x << std::endl;
//                        std::cout << robot_map_pose_.pose.position.y << std::endl;
//                        std::cout << reduce_goal.pose.position.x << std::endl;
//                        std::cout << reduce_goal.pose.position.y << std::endl;
//                        std::cout << enemy_buffer_.pose.position.x << std::endl;
//                        std::cout << enemy_buffer_.pose.position.y << std::endl;

                        ROS_INFO("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                        ROS_INFO("%f", robot_map_pose_.pose.position.x);
                        ROS_INFO("%f", robot_map_pose_.pose.position.y);
                        ROS_INFO("%f", reduce_goal.pose.position.x);
                        ROS_INFO("%f", reduce_goal.pose.position.y);
                        ROS_INFO("%f", enemy_buffer_.pose.position.x);
                        ROS_INFO("%f", enemy_buffer_.pose.position.y);


                        SendPlannerGoal(reduce_goal, vel);
                    }
                }
            } else {
                enemy_buffer_ = blackboard_ptr_->GetEnemyPose();
                UpdateRobotPose();

                auto dx = enemy_buffer_.pose.position.x - robot_map_pose_.pose.position.x;
                auto dy = enemy_buffer_.pose.position.y - robot_map_pose_.pose.position.y;
                auto yaw = std::atan2(dy, dx);

                if (
                        (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) >= 0.65 && distance_flag == 1) ||
                        (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 1.5 && distance_flag == 2)
                        ) {
                    // Enough Close to Enemy, range from 1m to 2m:
                    if (switch_mode_) {
                        // what the function of "switch_mode": true(far away); false(clost)
                        global_planner_actionlib_client_.cancelGoal();
                        local_planner_actionlib_client_.cancelGoal();
                        switch_mode_ = false;
                    }
                    blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
                    blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);
                    SetPlannerState(BehaviorState::SUCCESS);
                    distance_flag = 0;
                    return;
                }
            }

            UpdatePlannerState();
        }


        void EscapeGoal(float maxvel = 2.0) {
            if (planner_state_ != BehaviorState::RUNNING) {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
                blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

                if (blackboard_ptr_->GetEnemyDetected()) {
                    // find the enemy:
                    geometry_msgs::PoseStamped enemy;
                    enemy = blackboard_ptr_->GetEnemyPose();
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
                    auto get_enemy_cell = costmap_ptr_->GetCostMap()->World2Map(enemy.pose.position.x,
                                                                                enemy.pose.position.y, enemy_cell_x,
                                                                                enemy_cell_y);
                    if (!get_enemy_cell) {
                        angle_vel_pub_.publish(whirl_vel_);
                        SetPlannerState(BehaviorState::SUCCESS);
                        return;
                    }

                    while (true) {
                        // Try to find a goal for escape action:
                        // (find a goal: FREE) <----------and some obstacles---------Robot@
                        goal_x = x_uni_dis(gen);
                        goal_y = y_uni_dis(gen);
                        auto get_goal_cell = costmap_ptr_->GetCostMap()->World2Map(goal_x, goal_y, goal_cell_x,
                                                                                   goal_cell_y);
                        if (!get_goal_cell)
                            continue;

                        auto index = costmap_2d_->GetIndex(goal_cell_x, goal_cell_y);
                        if (charmap_[index] >= 253) {
                            // obstacle:
                            continue;
                        }

                        unsigned int obstacle_count = 0;
                        for (FastLineIterator line(goal_cell_x, goal_cell_y, enemy_cell_x,
                                                   enemy_cell_y); line.IsValid(); line.Advance()) {
                            auto point_cost = costmap_2d_->GetCost(line.GetX(), line.GetY()); //current point's cost
                            if (point_cost > 253) {
                                // Obstacle Counter:
                                obstacle_count++;
                            }
                        }
                        if (obstacle_count > 5) {
                            // find enough obstacles, and jump out of the while loop:
                            break;
                        }
                    }

                    Vec2d pose_to_enemy(enemy.pose.position.x - robot_map_pose_.pose.position.x,
                                        enemy.pose.position.y - robot_map_pose_.pose.position.y);
                    goal_yaw = static_cast<float>(std::atan2(pose_to_enemy.coeffRef(1), pose_to_enemy.coeffRef(0)));
                    auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, goal_yaw);

                    geometry_msgs::PoseStamped escape_goal;
                    escape_goal.header.frame_id = "map";
                    escape_goal.header.stamp = ros::Time::now();
                    escape_goal.pose.position.x = goal_x;
                    escape_goal.pose.position.y = goal_y;
                    escape_goal.pose.position.z = 1;
                    escape_goal.pose.orientation = quaternion;
                    SendPlannerGoal(escape_goal, maxvel);
                } else {
                    // not find the enemy:
                    angle_vel_pub_.publish(whirl_vel_);
                    SetPlannerState(BehaviorState::SUCCESS);
                    return;
                }
            }
            UpdatePlannerState();
        }


        void OutSupplierGoal(float maxvel = 3.0) {

            if (blackboard_ptr_->GetEnemyDetected()) {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
            } else {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
            }


            if (planner_state_ != BehaviorState::RUNNING) {

                blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

                float goal_yaw, goal_x, goal_y;
                UpdateRobotPose();

                if (blackboard_ptr_->GetEnemeyDetectedValid(0.5)) {
                    // find the enemy:
                    geometry_msgs::PoseStamped enemy;
                    enemy = blackboard_ptr_->GetEnemyPose();
                    if (enemy.pose.position.y > (enemy.pose.position.x + 0.5)) {
                        goal_x = 4.6;
                        goal_y = 3.2;
                        goal_yaw = std::atan2(goal_y - robot_map_pose_.pose.position.y,
                                              goal_x - robot_map_pose_.pose.position.x);

                    } else {
                        goal_x = 3.0;
                        goal_y = 3.6;
                        goal_yaw = std::atan2(goal_y - robot_map_pose_.pose.position.y,
                                              goal_x - robot_map_pose_.pose.position.x);
                    }

                } else {
                    goal_x = 4.6;
                    goal_y = 3.2;
                    goal_yaw = std::atan2(goal_y - robot_map_pose_.pose.position.y,
                                          goal_x - robot_map_pose_.pose.position.x);
                }

                geometry_msgs::PoseStamped out_supplier_goal;
                out_supplier_goal.header.frame_id = "map";
                out_supplier_goal.header.stamp = ros::Time::now();
                out_supplier_goal.pose.position.x = goal_x;
                out_supplier_goal.pose.position.y = goal_y;
                out_supplier_goal.pose.position.z = 1;
                auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, goal_yaw);
                out_supplier_goal.pose.orientation = quaternion;
                SendPlannerGoal(out_supplier_goal, maxvel);
            }
        }


        void OutForbiddenGoal(float maxvel = 1.8) {

            if (blackboard_ptr_->GetEnemyDetected()) {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
            } else {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
            }


            if (planner_state_ != BehaviorState::RUNNING) {
                blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
                UpdateRobotPose();
                geometry_msgs::PoseStamped out_forbidden_goal;
                out_forbidden_goal.header.frame_id = "map";
                out_forbidden_goal.header.stamp = ros::Time::now();

                std::vector<Vec2d> array;
                double goal_x;
                double goal_y;
                double bench_goal_x;
                double bench_goal_y;
                int isSetValue = 0;


                // std::cout << "Forbidden warn number:" << std::endl;
                ROS_INFO("Forbidden warn number:");
                if (blackboard_ptr_->GetForbiddenData().occ_eb_warn) {
                    ROS_INFO("1");
                    array = blackboard_ptr_->GetEnemyBuffRadiuPointArr();
                    bench_goal_x = blackboard_ptr_->GetForbiddenData().buff_center[1](0) + 1.4;
                    bench_goal_y = blackboard_ptr_->GetForbiddenData().buff_center[1](1);
                } else if (blackboard_ptr_->GetForbiddenData().occ_es_warn) {
                    ROS_INFO("2");
                    array = blackboard_ptr_->GetEnemySupplierRadiuPointArr();
                    bench_goal_x = blackboard_ptr_->GetForbiddenData().supplier_center[1](0);
                    bench_goal_y = blackboard_ptr_->GetForbiddenData().supplier_center[1](1) + 1.4;
                } else if (blackboard_ptr_->GetForbiddenData().occ_ob_warn) {
                    ROS_INFO("3");
                    array = blackboard_ptr_->GetOurBuffRadiuPointArr();
                    bench_goal_x = blackboard_ptr_->GetForbiddenData().buff_center[0](0) - 1.4;
                    bench_goal_y = blackboard_ptr_->GetForbiddenData().buff_center[0](1);
                } else if (blackboard_ptr_->GetForbiddenData().occ_os_warn) {
                    ROS_INFO("4");
                    array = blackboard_ptr_->GetOurSupplierRadiuPointArr();
                    bench_goal_x = blackboard_ptr_->GetForbiddenData().supplier_center[0](0);
                    bench_goal_y = blackboard_ptr_->GetForbiddenData().supplier_center[0](1) - 1.4;

                    if (!blackboard_ptr_->IsDangerous()) {
                        maxvel = 1.0;
                    }

                }


                if (blackboard_ptr_->GetGimbalMode() == GimbalMode::GIMBAL_RELATIVE_MODE) {

                    UpdateGimbalPose();
                    out_forbidden_goal.pose.orientation = gimbal_map_pose_.pose.orientation;
                    double line1, line2, line3;
                    line1 = -std::tan(GetYawFromGeometryMsgs(gimbal_map_pose_) + M_PI_2);
                    line2 = 1;
                    line3 = 0 - line1 * robot_map_pose_.pose.position.x - line2 * robot_map_pose_.pose.position.y;

                    double min_distance = 8;

                    for (int i = 0; i < array.size(); i++) {
                        unsigned int cell_x;
                        unsigned int cell_y;
                        auto get_cell = costmap_2d_->World2Map(array[i](0), array[i](1), cell_x, cell_y);
                        if (!get_cell)
                            continue;
                        auto index = costmap_2d_->GetIndex(cell_x, cell_y);
                        if (charmap_[index] < 253) {
                            double distance = std::fabs(line1 * array[i](0) + line2 * array[i](1) +
                                                        line3) / std::sqrt(std::pow(line1, 2) + std::pow(line2, 2));
                            if (distance < min_distance) {
                                min_distance = distance;
                                goal_x = array[i](0);
                                goal_y = array[i](1);
                                isSetValue = 1;

                            }
                        }
                    }

                } else {
                    // 开到最近的点
//                    out_forbidden_goal.pose.orientation = robot_map_pose_.pose.orientation;
//                    double min_distance = 8;
//                    for (int i = 0; i < array.size(); i++) {
//                        unsigned int cell_x;
//                        unsigned int cell_y;
//                        auto get_cell = costmap_2d_->World2Map(array[i](0), array[i](1), cell_x, cell_y);
//                        if (!get_cell)
//                            continue;
//                        auto index = costmap_2d_->GetIndex(cell_x, cell_y);
//                        if (charmap_[index] < 253) {
//                            double distance = std::sqrt(std::pow(robot_map_pose_.pose.position.x - array[i](0), 2) +
//                                                        std::pow(robot_map_pose_.pose.position.y - array[i](1), 2));
//                            if (distance < min_distance) {
//                                min_distance = distance;
//                                goal_x = array[i](0);
//                                goal_y = array[i](1);
//                                isSetValue = 2;
//
//                            }
//                        }
//                    }


                    out_forbidden_goal.pose.orientation = robot_map_pose_.pose.orientation;
                    double line1, line2, line3;
                    line1 = -std::tan(GetYawFromGeometryMsgs(robot_map_pose_) + M_PI_2);
                    line2 = 1;
                    line3 = 0 - line1 * robot_map_pose_.pose.position.x - line2 * robot_map_pose_.pose.position.y;

                    double min_distance = 8;

                    for (int i = 0; i < array.size(); i++) {
                        unsigned int cell_x;
                        unsigned int cell_y;
                        auto get_cell = costmap_2d_->World2Map(array[i](0), array[i](1), cell_x, cell_y);
                        if (!get_cell)
                            continue;
                        auto index = costmap_2d_->GetIndex(cell_x, cell_y);
                        if (charmap_[index] < 253) {
                            double distance = std::fabs(line1 * array[i](0) + line2 * array[i](1) +
                                                        line3) / std::sqrt(std::pow(line1, 2) + std::pow(line2, 2));
                            if (distance < min_distance) {
                                min_distance = distance;
                                goal_x = array[i](0);
                                goal_y = array[i](1);
                                isSetValue = 2;

                            }
                        }
                    }


                }

                if (isSetValue == 0) {
                    goal_x = bench_goal_x;
                    goal_y = bench_goal_y;
//                    std::cout << "origin:" << robot_map_pose_.pose.position.x << "," << robot_map_pose_.pose.position.y
//                              << std::endl;
                    ROS_INFO("origin:%f,%f", robot_map_pose_.pose.position.x, robot_map_pose_.pose.position.y);
//                    std::cout << "bench_goal" << std::endl;
                    ROS_INFO("bench_goal");
                } else if (isSetValue == 1) {
//                    std::cout << "origin:" << robot_map_pose_.pose.position.x << "," << robot_map_pose_.pose.position.y
//                              << std::endl;
//                    std::cout << "GIMBAL_RELATIVE_MODE_goal" << std::endl;

                    ROS_INFO("origin:%f,%f", robot_map_pose_.pose.position.x, robot_map_pose_.pose.position.y);
                    ROS_INFO("GIMBAL_RELATIVE_MODE_goal");

                } else if (isSetValue == 2) {
                    ROS_INFO("origin:%f,%f", robot_map_pose_.pose.position.x, robot_map_pose_.pose.position.y);
                    ROS_INFO("GIMBAL_PATROL_MODE_goal");
                }


                out_forbidden_goal.pose.position.x = goal_x;
                out_forbidden_goal.pose.position.y = goal_y;
                out_forbidden_goal.pose.position.z = 1;
                SendPlannerGoal(out_forbidden_goal, maxvel);
            }
        }


        void GoEnemyBuffGoal(float maxvel = 3.0) {
            static Vec3d shoot_pose[3] = {Vec3d(1, 3.15, 0), Vec3d(2.4, 3.15, M_PI),
                                          Vec3d(2.05, 2.55, M_PI * 2 / 3)};

            if (blackboard_ptr_->GetEnemyDetected()) {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
            } else {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
            }

            if (planner_state_ != BehaviorState::RUNNING) {

                blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

                UpdateRobotPose();
                uint8_t i[2];
                double min_distance = 8;

                if (blackboard_ptr_->GetRoleState()) {
                    //如果另外一台车还通讯着,在下半场,视另一台车而定;否则找最近的
                    if (blackboard_ptr_->GetAnotherData().accessible) {
                        if (robot_map_pose_.pose.position.y <= 3.25 &&
                            blackboard_ptr_->GetAnotherData().pose_y <= robot_map_pose_.pose.position.y) {
                            //wing choose first
                            int j;
                            for (j = 0; j < 3; j++) {
                                double distance = EularDistance(blackboard_ptr_->GetAnotherData().pose_x,
                                                                shoot_pose[j](0),
                                                                blackboard_ptr_->GetAnotherData().pose_y,
                                                                shoot_pose[j](1));
                                if (distance < min_distance) {
                                    i[1] = j;
                                    min_distance = distance;
                                }
                            }

                            //master choose second
                            min_distance = 8;
                            for (j = 0; j < 3; j++) {
                                if (j != i[1]) {
                                    double distance = EularDistance(robot_map_pose_.pose.position.x, shoot_pose[j](0),
                                                                    robot_map_pose_.pose.position.y, shoot_pose[j](1));
                                    if (distance < min_distance) {
                                        i[0] = j;
                                        min_distance = distance;
                                    }
                                }
                            }
                        } else {
                            // master choose first
                            int j;
                            for (j = 0; j < 3; j++) {
                                double distance = EularDistance(robot_map_pose_.pose.position.x,
                                                                shoot_pose[j](0),
                                                                robot_map_pose_.pose.position.y,
                                                                shoot_pose[j](1));
                                if (distance < min_distance) {
                                    i[0] = j;
                                    min_distance = distance;
                                }
                            }

                            //wing choose second
                            min_distance = 8;
                            for (j = 0; j < 3; j++) {
                                if (j != i[0]) {
                                    double distance = EularDistance(blackboard_ptr_->GetAnotherData().pose_x,
                                                                    shoot_pose[j](0),
                                                                    blackboard_ptr_->GetAnotherData().pose_y,
                                                                    shoot_pose[j](1));
                                    if (distance < min_distance) {
                                        i[1] = j;
                                        min_distance = distance;
                                    }
                                }
                            }
                        }
                    }
                        //没通,去最近的
                    else {
                        int j;
                        for (j = 0; j < 3; j++) {
                            double distance = EularDistance(robot_map_pose_.pose.position.x, shoot_pose[j](0),
                                                            robot_map_pose_.pose.position.y, shoot_pose[j](1));
                            if (distance < min_distance) {
                                i[0] = j;
                                min_distance = distance;
                            }
                        }
                        i[1] = (j + 1) % 3;
                    }

                } else {
                    //wing   wing去最近的,master挑次近的
                    int j;
                    for (j = 0; j < 3; j++) {
                        double distance = EularDistance(robot_map_pose_.pose.position.x, shoot_pose[j](0),
                                                        robot_map_pose_.pose.position.y, shoot_pose[j](1));
                        if (distance < min_distance) {
                            i[1] = j;
                            min_distance = distance;
                        }
                    }


                    //master choose second
                    min_distance = 8;
                    for (j = 0; j < 3; j++) {
                        if (j != i[1]) {
                            double distance = EularDistance(robot_map_pose_.pose.position.x, shoot_pose[j](0),
                                                            robot_map_pose_.pose.position.y, shoot_pose[j](1));
                            if (distance < min_distance) {
                                i[0] = j;
                                min_distance = distance;
                            }
                        }
                    }
                }

                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = shoot_pose[i[0]](0);
                pose.pose.position.y = shoot_pose[i[0]](1);
                tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, shoot_pose[i[0]](2));
                pose.pose.orientation.x = quaternion.x();
                pose.pose.orientation.y = quaternion.y();
                pose.pose.orientation.z = quaternion.z();
                pose.pose.orientation.w = quaternion.w();
                SendPlannerGoal(pose, maxvel);

                pose.pose.position.x = shoot_pose[i[1]](0);
                pose.pose.position.y = shoot_pose[i[1]](1);
                quaternion = tf::createQuaternionFromRPY(0, 0, shoot_pose[i[1]](2));
                pose.pose.orientation.x = quaternion.x();
                pose.pose.orientation.y = quaternion.y();
                pose.pose.orientation.z = quaternion.z();
                pose.pose.orientation.w = quaternion.w();

                SendGoalTask(pose);

//                std::cout << "GoEnemyBuffGoal:" << (int) i[0] << "," << (int) i[1] << std::endl;
                ROS_INFO("GoEnemyBuffGoal:%d,%d", (int) i[0], (int) i[1]);
            }
        }


        void GoEnemySupplierGoal(float maxvel = 3.0) {
            static Vec3d shoot_pose[2] = {Vec3d(4, 1.4, -M_PI_2), Vec3d(4.9, 0.65, M_PI)};

            if (blackboard_ptr_->GetEnemyDetected()) {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
            } else {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
            }


            if (planner_state_ != BehaviorState::RUNNING) {

                blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

                UpdateRobotPose();
                uint8_t i = 0;

                if (blackboard_ptr_->GetRoleState()) {
                    //如果另外一台车还通讯着,在左边半场,则去0区,右边半场,视另一台车而定
                    if (blackboard_ptr_->GetAnotherData().accessible) {
                        if (robot_map_pose_.pose.position.x <= 4) {
                            i = 0;
                        } else {
                            if (blackboard_ptr_->GetAnotherData().pose_x >= robot_map_pose_.pose.position.x) {
                                i = 0;
                            } else {
                                i = 1;
                            }
                        }
                    }
                        //没通去最近的
                    else {
                        i =
                                (EularDistance(robot_map_pose_.pose.position.x, shoot_pose[0](0),
                                               robot_map_pose_.pose.position.y, shoot_pose[0](1)) >
                                 EularDistance(robot_map_pose_.pose.position.x, shoot_pose[1](0),
                                               robot_map_pose_.pose.position.y, shoot_pose[1](1)));
                    }
                } else {
                    i =
                            (EularDistance(robot_map_pose_.pose.position.x, shoot_pose[0](0),
                                           robot_map_pose_.pose.position.y, shoot_pose[0](1)) >
                             EularDistance(robot_map_pose_.pose.position.x, shoot_pose[1](0),
                                           robot_map_pose_.pose.position.y, shoot_pose[1](1)));
                }

                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = shoot_pose[i](0);
                pose.pose.position.y = shoot_pose[i](1);
                tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, shoot_pose[i](2));
                pose.pose.orientation.x = quaternion.x();
                pose.pose.orientation.y = quaternion.y();
                pose.pose.orientation.z = quaternion.z();
                pose.pose.orientation.w = quaternion.w();
                SendPlannerGoal(pose, maxvel);

                pose.pose.position.x = shoot_pose[!(i & 1)](0);
                pose.pose.position.y = shoot_pose[!(i & 1)](1);
                quaternion = tf::createQuaternionFromRPY(0, 0, shoot_pose[!(i & 1)](2));
                pose.pose.orientation.x = quaternion.x();
                pose.pose.orientation.y = quaternion.y();
                pose.pose.orientation.z = quaternion.z();
                pose.pose.orientation.w = quaternion.w();

                SendGoalTask(pose);

//                std::cout << "GoEnemySupplierGoal:" << (int) i << "," << (int) (!(i & 1)) << std::endl;
                ROS_INFO("GoEnemySupplierGoal:%d,%d", (int) i, (int) (!(i & 1)));
            }
        }


//bonus_index_;
        int bonus_index_ = 0;

        BehaviorState GainBuffGoal(float maxvel = 3.0, float tunevel = 0.8) {

            if (blackboard_ptr_->GetEnemyDetected()) {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
            } else {
                blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
            }


            if (planner_state_ != BehaviorState::RUNNING) {
                blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
                geometry_msgs::PoseStamped pose;
                if (bonus_index_ == 0) {
                    pose.pose.position.x = blackboard_ptr_->GetForbiddenData().buff_center[0](0);
                    pose.pose.position.y = blackboard_ptr_->GetForbiddenData().buff_center[0](1);
                } else if (bonus_index_ == 1) {
                    pose.pose.position.x = blackboard_ptr_->GetForbiddenData().buff_center[0](0) + 0.2;
                    pose.pose.position.y = blackboard_ptr_->GetForbiddenData().buff_center[0](1) + 0.2;
                } else if (bonus_index_ == 2) {
                    pose.pose.position.x = blackboard_ptr_->GetForbiddenData().buff_center[0](0) - 0.2;
                    pose.pose.position.y = blackboard_ptr_->GetForbiddenData().buff_center[0](1);
                } else if (bonus_index_ == 3) {
                    pose.pose.position.x = blackboard_ptr_->GetForbiddenData().buff_center[0](0) + 0.2;
                    pose.pose.position.y = blackboard_ptr_->GetForbiddenData().buff_center[0](1);
                } else if (bonus_index_ == 4) {
                    pose.pose.position.x = blackboard_ptr_->GetForbiddenData().buff_center[0](0) - 0.2;
                    pose.pose.position.y = blackboard_ptr_->GetForbiddenData().buff_center[0](1) + 0.2;
                }
                pose.pose.position.z = 1;

                if (blackboard_ptr_->GetEnemyDetected()) {
                    UpdateGimbalPose();
                    pose.pose.orientation = gimbal_map_pose_.pose.orientation;
                } else {
                    double tmp_roll = 0, tmp_pitch = 0, tmp_yaw = M_PI_2 + M_PI_4;
                    tf::Quaternion quaternion = tf::createQuaternionFromRPY(tmp_roll, tmp_pitch, tmp_yaw);
                    pose.pose.orientation.x = quaternion.x();
                    pose.pose.orientation.y = quaternion.y();
                    pose.pose.orientation.z = quaternion.z();
                    pose.pose.orientation.w = quaternion.w();
                }

                if (bonus_index_ == 0)
                    SendPlannerGoal(pose, maxvel);
                else {
                    SendPlannerGoal(pose, tunevel);

//                    std::cout << "TuneVel because:" << (int) (blackboard_ptr_->GetRfidScannning()) << ","
//                              << (int) (blackboard_ptr_->GetBuffData().our_bonus_occupying)
//                              << std::endl;
                    ROS_INFO("TuneVel because:%d,%d", (int) (blackboard_ptr_->GetRfidScannning()),
                             (int) (blackboard_ptr_->GetBuffData().our_bonus_occupying));
                }
                bonus_index_ = (++bonus_index_) % 5;
            }

            UpdatePlannerState();

            if (planner_state_ == BehaviorState::SUCCESS) {
                return BehaviorState::RUNNING;
            }
            return planner_state_;
        }


        void OutJammingGoal() {
            Vec3d sum;
            for (int i = 0; i < speed_record_.size(); i++) {
                sum(0) += speed_record_[i](0);
                sum(1) += speed_record_[i](1);
                sum(2) += speed_record_[i](2);
            }
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = -sum(0) / speed_record_.size();
            cmd_vel.linear.y = -sum(1) / speed_record_.size();
            cmd_vel.angular.z = -sum(2) / speed_record_.size();
            angle_vel_pub_.publish(cmd_vel);
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


        void UpdateGimbalPose() {
            tf::Stamped<tf::Pose> gimbal_tf_pose;
            gimbal_tf_pose.setIdentity();

            gimbal_tf_pose.frame_id_ = "gimbal";
            gimbal_tf_pose.stamp_ = ros::Time();
            try {
                geometry_msgs::PoseStamped gimbal_pose;
                tf::poseStampedTFToMsg(gimbal_tf_pose, gimbal_pose);
                tf_ptr_->transformPose("map", gimbal_pose, gimbal_map_pose_);
            }
            catch (tf::LookupException &ex) {
                ROS_ERROR("Transform Error looking up gimbal pose: %s", ex.what());
            }

        }

        void SendGoalTask(geometry_msgs::PoseStamped goal) {
//            auto dx = goal.pose.position.x - last_goal_.pose.position.x;
//            auto dy = goal.pose.position.y - last_goal_.pose.position.y;
//            tf::Quaternion rot1, rot2;
//            tf::quaternionMsgToTF(goal.pose.orientation, rot1);
//            tf::quaternionMsgToTF(last_goal_.pose.orientation, rot2);
//            auto d_yaw = rot1.angleShortestPath(rot2);
//
//            if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) < 0.8) {
//                return;
//            }
            leonard_decision::GoalTask goal_task_msg;
            goal_task_msg.goal = goal;
            // goal_pub_.publish(goal_task_msg);
            blackboard_ptr_->PublishGoalTaskByGoal(goal_task_msg);
//            std::cout << "SendGoalTask to wing robot..../master/goal_task" << goal.pose.position.x << ","
//                      << goal.pose.position.y << std::endl;

            ROS_INFO("SendGoalTask to wing robot..../master/goal_task:%f,%f", goal.pose.position.x,
                     goal.pose.position.y);


//            last_goal_ = goal;
        }

// Definition of FastLineIterator
        class FastLineIterator {
            // this method is a modified version of base_local_planner/line_iterator.h
        public:
            FastLineIterator(int x0, int y0, int x1, int y1)
                    : x0_(x0), y0_(y0), x1_(x1), y1_(y1), x_(x0), y_(y0), deltax_(abs(x1 - x0)),
                      deltay_(abs(y1 - y0)),
                      curpixel_(0) {
                xinc1_ = (x1 - x0) > 0 ? 1 : -1;
                xinc2_ = (x1 - x0) > 0 ? 1 : -1;
                yinc1_ = (y1 - y0) > 0 ? 1 : -1;
                yinc2_ = (y1 - y0) > 0 ? 1 : -1;

                if (deltax_ >= deltay_) {
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
                if (num_ >= den_) {
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

    protected:
        geometry_msgs::PoseStamped enemy_buffer_;
        bool lost_enemy_;
        geometry_msgs::PoseStamped robot_map_pose_;
        geometry_msgs::PoseStamped gimbal_map_pose_;
        bool switch_mode_;


//! CostMap
        std::shared_ptr<CostMap> costmap_ptr_;
        CostMap2D *costmap_2d_;
        unsigned char *charmap_;
//! tf
        std::shared_ptr<tf::TransformListener> tf_ptr_;

        ros::NodeHandle nh;
        ros::Publisher angle_vel_pub_;
        BehaviorState planner_state_ = BehaviorState::IDLE;
        BehaviorState aim_supply_state_ = BehaviorState::IDLE;
        BehaviorState turn_angle_state_ = BehaviorState::IDLE;
        Blackboard::Ptr blackboard_ptr_;

        GlobalGoal global_planner_goal_;
        LocalGoal local_planner_goal_;
        LocalActionClient local_planner_actionlib_client_;
        GlobalActionClient global_planner_actionlib_client_;
// Add Bullet:
        actionlib::SimpleActionClient<leonard_decision::SupplyPidAction> aim_supply_actionlib_client_;
/********leonard add*********/
        actionlib::SimpleActionClient<leonard_decision::TurnAngleAction> turn_angle_actionlib_client_;

        std::vector<geometry_msgs::PoseStamped> patrol_goals_;

        std::vector<geometry_msgs::PoseStamped> inner_patrol_goals_;

        std::vector<geometry_msgs::PoseStamped> wait_patrol_goals_;

        int point_size_;
        unsigned int patrol_count_;

        int inner_point_size_;
        unsigned int inner_patrol_count_;

        int wait_point_size_;
        unsigned int wait_patrol_count_ = 0;


// escape configurations:
        float left_x_limit_, right_x_limit_;
        float robot_x_limit_;
        float left_random_min_x_, left_random_max_x_;
        float right_random_min_x_, right_random_max_x_;
        geometry_msgs::Twist whirl_vel_;


// Search:
        unsigned int search_count_;
        std::vector<geometry_msgs::PoseStamped> search_region_1_;
        std::vector<geometry_msgs::PoseStamped> search_region_2_;
        std::vector<geometry_msgs::PoseStamped> search_region_3_;
        std::vector<geometry_msgs::PoseStamped> search_region_4_;
        std::vector<geometry_msgs::PoseStamped> search_region_;
        unsigned int search_index_;

// Supply location:
        geometry_msgs::PoseStamped supply_location_;

// master-wing:
        bool is_master_;
        geometry_msgs::PoseStamped last_goal_;

        ros::Subscriber sub_cmd_vel_acc_;
        std::vector<Vec3d> speed_record_;
        ros::Publisher planner_state_pub_;


    private:

        template<typename T>
        static T normalize(T z) {
            return atan2(sin(z), cos(z));
        }

        template<typename T>
        static T angle_diff(double a, double b) {
            T d1, d2;
            a = normalize(a);
            b = normalize(b);
            d1 = a - b;
            d2 = 2 * M_PI - fabs(d1);
            if (d1 > 0)
                d2 *= -1.0;
            if (fabs(d1) < fabs(d2))
                return (d1);
            else
                return (d2);
        }

        inline double GetYawFromGeometryMsgs(geometry_msgs::PoseStamped &poseStamped) {
            geometry_msgs::Quaternion orientation = poseStamped.pose.orientation;
            tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
            double yaw, pitch, roll;
            mat.getEulerYPR(yaw, pitch, roll);
            return yaw;
        }

        inline double EularDistance(double x1, double x2, double y1, double y2) {
            return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
        }

    };
} // namespace decision
#endif //MODULE_DECISION_ICRA_GOAL_FACORY_H