#ifndef MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H
#define MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include "icra_decision/EnemyDetect.h"
#include "icra_decision/ChassisMode.h"
#include "icra_decision/GimbalMode.h"
#include "icra_decision/GoalTask.h"
#include "icra_decision/RobotDamage.h"
#include "icra_decision/RobotStatus.h"
#include "icra_decision/BonusStatus.h"
#include "icra_decision/GameResult.h"
#include "icra_decision/GameStatus.h"
#include "icra_decision/GameSurvivor.h"
#include "icra_decision/RobotHeat.h"
#include "icra_decision/RobotShoot.h"
#include "icra_decision/SupplierStatus.h"
#include "icra_decision/Bullet.h"

namespace decision {
// 云台模式
enum class GimbalMode {
  GIMBAL_RELAX         = 0,
  GIMBAL_PATROL_MODE   = 1,
  GIMBAL_RELATIVE_MODE = 2
};
// 底盘模式
enum class ChassisMode {
  AUTO_SEPARATE_GIMBAL = 0,
  DODGE_MODE           = 1
};
// 装甲伤害的方向
enum class ArmorAttacked {
  FRONT = 0,
  LEFT = 1,
  BACK = 2,
  RIGHT = 3,
  NONE = 4
};

class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  
  explicit Blackboard():
    // 状态初始化
    goto_supplier_finished_(false),
    goto_buff_finished_(false),
    minute_counter_(3),
    has_a_supply_chance_(true),
    sync_supply_action_busy_(false),
    sync_buff_action_busy_(false),
    is_blue_team_(true),
    find_the_front_enemy_(false),
    find_the_back_enemy_(false),
    red_bonus_(0),
    blue_bonus_(0),
    remain_bullet_num_(40),
    game_status_(0),
    remaining_time_(180),
    remain_hp_(2000),
    supply_projectile_id_(0),
    supply_robot_id_(0),
    supply_projectile_step_(0),
    armor_attacked_(ArmorAttacked::NONE),
    gimbal_mode_(GimbalMode::GIMBAL_RELAX),
    chassis_mode_(ChassisMode::AUTO_SEPARATE_GIMBAL) {
    ros::NodeHandle nh;
    // 订阅相机的话题
    front_enemy_sub_ = nh.subscribe("test_enemy_pose", 1, &Blackboard::FindEnemyFrontCallback, this);
    back_enemy_sub_ = nh.subscribe("enemy_back", 1, &Blackboard::FindEnemyBackCallback, this);
    // 订阅裁判系统的话题
    robot_damage_sub_ = nh.subscribe("robot_damage", 30, &Blackboard::RobotDamageCallback, this);
    robot_status_sub_ = nh.subscribe("robot_status", 30, &Blackboard::RobotStatusCallback, this);
    bonus_status_sub_ = nh.subscribe("field_bonus_status", 30, &Blackboard::BonusStatusCallback, this);
    game_result_sub_ = nh.subscribe("game_result", 30, &Blackboard::GameResultCallback, this);
    game_status_sub_ = nh.subscribe("game_status", 30, &Blackboard::GameStatusCallback, this);
    game_survivor_sub_ = nh.subscribe("game_survivor", 30, &Blackboard::GameSurvivorCallback, this);
    remain_bullet_num_sub_ = nh.subscribe("robot_bullet", 30, &Blackboard::RemainBulletNumCallback, this);
    supplier_status_sub_ = nh.subscribe("field_supplier_status", 30, &Blackboard::SupplierStatusCallback, this);
    // 底盘和云台工作模式的客户端
    chassis_mode_client_ = nh.serviceClient<icra_decision::ChassisMode>("set_chassis_mode");
    gimbal_mode_client_ = nh.serviceClient<icra_decision::GimbalMode>("set_gimbal_mode");
    // 多机器人数据同步
    nh.getParam("is_master", is_master_);
    if (is_master_) {
      goal_task_sub_ = nh.subscribe("/wing/goal_task", 2, &Blackboard::GoalTaskCallBack, this);
      sync_state_sub_ = nh.subscribe("/wing/sync_state", 2, &Blackboard::SyncStateCallBack, this);
      ROS_INFO("sub some special wing sync topics");
    } else {
      goal_task_sub_ = nh.subscribe("/master/goal_task", 2, &Blackboard::GoalTaskCallBack, this);
      sync_state_sub_ = nh.subscribe("/master/sync_state", 2, &Blackboard::SyncStateCallBack, this);
      ROS_INFO("sub some special master sync topics");
      // Wing 优先补弹
      has_a_buff_chance_ = false;
    }
  }
  ~Blackboard() = default;
  
  // 重置所有状态
  void ResetAllStatus () {
    remain_hp_ = 2000;
    armor_attacked_ = ArmorAttacked::NONE;
    // TODO
  }

  // 采用回调函数的方式更新系统状态
  void SupplierStatusCallback(const icra_decision::SupplierStatus::ConstPtr& supplier_status){
    supply_projectile_id_ = supplier_status->supply_projectile_id;
    supply_robot_id_ = supplier_status->supply_robot_id;
    supply_projectile_step_ = supplier_status->supply_projectile_step;
  }
  void RemainBulletNumCallback(const icra_decision::Bullet::ConstPtr& bullet_num){
    remain_bullet_num_ = bullet_num->bullet_num;
    // remain_bullet_num_ = 100;
  }
  void GameSurvivorCallback(const icra_decision::GameSurvivor::ConstPtr& game_survivor){
    red3_survived_ = game_survivor->red3;
    red4_survived_ = game_survivor->red4;
    blue13_survived_ = game_survivor->blue3;
    blue14_survived_ = game_survivor->blue4;
  }
  void GameStatusCallback(const icra_decision::GameStatus::ConstPtr& game_status){
    game_status_ = static_cast<unsigned int>(game_status->game_status);
    remaining_time_ = static_cast<unsigned int>(game_status->remaining_time);
  }
  void GameResultCallback(const icra_decision::GameResult::ConstPtr& game_result){
    result_ = static_cast<unsigned int>(game_result->result);
  }
  void BonusStatusCallback(const icra_decision::BonusStatus::ConstPtr& bonus_status){
    red_bonus_ = static_cast<unsigned int>(bonus_status->red_bonus);
    blue_bonus_ = static_cast<unsigned int>(bonus_status->blue_bonus);
  }
  void RobotDamageCallback(const icra_decision::RobotDamage::ConstPtr& robot_hurt_data){
    last_armor_attacked__time_ = ros::Time::now();
    if (robot_hurt_data->damage_type == 0) {
      switch(robot_hurt_data->damage_source) {
      case 0:
        armor_attacked_ = ArmorAttacked::FRONT;
        // ROS_INFO("BB--ArmorAttacked::FRONT");
        break;
      case 1:
        armor_attacked_ = ArmorAttacked::LEFT;
        // ROS_INFO("BB--ArmorAttacked::LEFT");
        break;
      case 2:
        armor_attacked_ = ArmorAttacked::BACK;
        // ROS_INFO("BB--ArmorAttacked::BACK");
        break;
      case 3:
        armor_attacked_ = ArmorAttacked::RIGHT;
        // ROS_INFO("BB--ArmorAttacked::RIGHT");
        break;
      default:
        armor_attacked_ = ArmorAttacked::NONE;
        return;
      }
    } else {
      armor_attacked_ = ArmorAttacked::NONE;
    }
  }
  void RobotStatusCallback(const icra_decision::RobotStatus::ConstPtr& robot_status){
    remain_hp_ = static_cast<unsigned int>(robot_status->remain_hp);
    local_id_ = static_cast<unsigned int>(robot_status->id);
    if (local_id_ == 13 || local_id_ == 14) {
      is_blue_team_ = true;
    }
    if (local_id_ ==  3 || local_id_ ==  4) {
      is_blue_team_ = false;
    }
  }
  void FindEnemyFrontCallback(const icra_decision::EnemyDetect::ConstPtr& enemy) {
    find_the_front_enemy_ = enemy->detected;
    pose_of_front_enemy_ = enemy->enemy_pos;
  }
  void FindEnemyBackCallback(const std_msgs::Bool& color_detect) {
    last_find_back_enemy_time_ = ros::Time::now();
    find_the_back_enemy_ = color_detect.data;
  }
  void GoalTaskCallBack(const icra_decision::GoalTask::ConstPtr& goal_task) {
    auxiliary_position_ = goal_task->goal;
    last_auxiliary_time_ = ros::Time::now();
    ROS_INFO("Received a Goal Task");
    auxiliary_ = true;
  }
  void SyncStateCallBack(const std_msgs::Int32& msg) {
    last_sync_state_time_ = ros::Time::now();
    int state_temp = 0;
    state_temp = msg.data;
    if (state_temp == 1) {
      sync_supply_action_busy_ = true;
      sync_buff_action_busy_ = false;
    } else if (state_temp == 2){
      sync_supply_action_busy_ = false;
      sync_buff_action_busy_ = true;
    } else {
      sync_supply_action_busy_ = false;
      sync_buff_action_busy_ = false;
    }
  }

  // 配置函数
  // 每分钟重置补弹和BUFF机会
  void ScheduledReset() {
    // master 和 wing 重置时间相差两秒
    // Reset all flags in every minutes
    if (is_master_) {
        if(minute_counter_ == 3) {
        has_a_supply_chance_ = true;
        // Master（已有40发子弹）先去抢BUFF，Wing先去补弹
        if (is_master_){
          has_a_buff_chance_ = true;
        } else {
          has_a_buff_chance_ = false;
        }
        goto_buff_finished_ = false;
        goto_supplier_finished_ = false;
        --minute_counter_;
      } else if(minute_counter_ == 2 && remaining_time_ <= 120) {
        ROS_INFO("Reset@120s......");
        has_a_supply_chance_ = true;
        has_a_buff_chance_ = true;
        goto_buff_finished_ = false;
        goto_supplier_finished_ = false;
        --minute_counter_;
      } else if(minute_counter_ == 1 && remaining_time_ <= 60) {
        ROS_INFO("Reset@60s......");
        has_a_supply_chance_ = true;
        has_a_buff_chance_ = true;
        goto_buff_finished_ = false;
        goto_supplier_finished_ = false;
        --minute_counter_;
      }
    } else {
        if(minute_counter_ == 3) {
        has_a_supply_chance_ = true;
        // Master（已有40发子弹）先去抢BUFF，Wing先去补弹
        if (is_master_){
          has_a_buff_chance_ = true;
        } else {
          has_a_buff_chance_ = false;
        }
        goto_buff_finished_ = false;
        goto_supplier_finished_ = false;
        --minute_counter_;
      } else if(minute_counter_ == 2 && remaining_time_ <= (120-2)) {
        has_a_supply_chance_ = true;
        has_a_buff_chance_ = true;
        goto_buff_finished_ = false;
        goto_supplier_finished_ = false;
        --minute_counter_;
      } else if(minute_counter_ == 1 && remaining_time_ <= (60-2)) {
        has_a_supply_chance_ = true;
        has_a_buff_chance_ = true;
        goto_buff_finished_ = false;
        goto_supplier_finished_ = false;
        --minute_counter_;
      }
    }
  }
  bool SetGimbalMode(const GimbalMode &gimbal_mode) {
    if(gimbal_mode == gimbal_mode_){
      return true;
    }
    icra_decision::GimbalMode gimbal_mode_msg;
    gimbal_mode_msg.request.gimbal_mode = static_cast<uint8_t>(gimbal_mode);
    if(gimbal_mode_client_.call(gimbal_mode_msg) && gimbal_mode_msg.response.received){
      gimbal_mode_ = gimbal_mode;
      ROS_INFO("Set gimbal mode to %d", static_cast<uint8_t>(gimbal_mode));
      return true;
    } else {
      ROS_ERROR("Set gimbal mode failed!");
      return false;
    }
  }
  bool SetChassisMode(const ChassisMode &chassis_mode) {
    if(chassis_mode == chassis_mode_){
      return true;
    }
    icra_decision::ChassisMode chassis_mode_msg;
    chassis_mode_msg.request.chassis_mode = static_cast<uint8_t >(chassis_mode);
    if(chassis_mode_client_.call(chassis_mode_msg) && chassis_mode_msg.response.received){
      chassis_mode_ = chassis_mode;
      ROS_INFO("Set chassis mode to %d", static_cast<int>(chassis_mode));
      return true;
    } else {
      ROS_ERROR("Set chassis mode failed!");
      return false;
    }
  }
  void UpdateRemainBulletNum() {
    remain_bullet_num_ += 50;
  }

  // 查询函数
  bool IsMaster() const {
    return is_master_;
  }
  bool MasterSurvived() const {
    if (is_master_) {
      if (local_id_ == 3)
        return red3_survived_;
      if (local_id_ == 4)
        return red4_survived_;
      if (local_id_ ==13)
        return blue13_survived_;
      if (local_id_ ==14)
        return blue14_survived_;
    } else {
      if (local_id_ == 3)
        return red4_survived_;
      if (local_id_ == 4)
        return red3_survived_;
      if (local_id_ ==13)
        return blue14_survived_;
      if (local_id_ == 3)
        return blue13_survived_;
    } 
  }
  bool WingSurvived() const {
    if (!is_master_) {
      if (local_id_ == 3)
        return red3_survived_;
      if (local_id_ == 4)
        return red4_survived_;
      if (local_id_ ==13)
        return blue13_survived_;
      if (local_id_ ==14)
        return blue14_survived_;
    } else {
      if (local_id_ == 3)
        return red4_survived_;
      if (local_id_ == 4)
        return red3_survived_;
      if (local_id_ ==13)
        return blue14_survived_;
      if (local_id_ == 3)
        return blue13_survived_;
    } 
  }
  bool GetFrontEnemyDetected() const {
    return find_the_front_enemy_;
  }
  geometry_msgs::PoseStamped GetFrontEnemyPose() const {
    return pose_of_front_enemy_;
  }
  bool IsSupplyBusy() {
    if(ros::Time::now() - last_sync_state_time_ > ros::Duration(15)) {
      sync_supply_action_busy_ = false;
    }
    return sync_supply_action_busy_;
  }
  bool IsBuffBusy() {
    if(ros::Time::now() - last_sync_state_time_ > ros::Duration(15)) {
      sync_buff_action_busy_ = false;
    }
    return sync_buff_action_busy_;
  }
  bool GetBackEnemyDetected() {
    // 在一定时间内（0.1s）判断状态是否有效
    if(ros::Time::now() - last_find_back_enemy_time_ > ros::Duration(0.5)){
      find_the_back_enemy_ = false;
    }
    return find_the_back_enemy_;
  }
  bool GetAuxiliaryState() {
    if(ros::Time::now() - last_auxiliary_time_ > ros::Duration(10)) {
      // ROS_INFO("GetAuxiliary Out of Time.");
      auxiliary_ = false;
    }
    return auxiliary_;
  }
  ArmorAttacked GetArmorAttacked() {
    if(ros::Time::now() - last_armor_attacked__time_ > ros::Duration(0.5)){
      armor_attacked_ = ArmorAttacked::NONE;
    }
    return armor_attacked_;
  }
  unsigned int GetRemainHp() const {
    return remain_hp_;
  }
  unsigned int GetBonusStatus() const {
    if (is_blue_team_)
      return blue_bonus_;
    else
      return red_bonus_;
  }
  unsigned int GetEnemyBonusStatus() const {
    if (is_blue_team_)
      return red_bonus_;
    else
      return blue_bonus_;
  }
  geometry_msgs::PoseStamped GetAuxiliaryPosition() const {
    return auxiliary_position_;
  }
  unsigned int GetGameStatus() const {
    return game_status_;
  }
  bool GameStopCondition() const {
    return (game_status_ != 4);
  }
  // 剩余子弹数量
  unsigned int GetRemainBulletNum() const {
    return remain_bullet_num_;
  }
  // 蹭敌人在补弹或抢BUFF时偷袭
  bool NeedGotoEnemySupplier() {
    if (supply_projectile_step_ != 0) {
      // 我方blue，敌方red
      if ( is_blue_team_ && (supply_robot_id_ == 3 || supply_robot_id_ == 4) && (supply_projectile_id_ == 0)) {
        return true;
      }
      // 我方red，敌方blue
      if (!is_blue_team_ && (supply_robot_id_ ==13 || supply_robot_id_ ==14) && (supply_projectile_id_ == 1)) {
        return true;
      }
    }
    return false;
  }
  bool NeedGotoEnemyBuff() {
    // TODO
    if ( is_blue_team_ && ( red_bonus_ == 1)) {
      return true;
    }
    if (!is_blue_team_ && (blue_bonus_ == 1)) {
      return true;
    }
    return false;
  }
  // 剩余时间 < 180s
  unsigned int GetRemainingTime() const {
    return remaining_time_;
  }
  


 public:
  // 云台和底盘模式配置的客户端
  GimbalMode gimbal_mode_;
  ros::ServiceClient gimbal_mode_client_;
  ChassisMode chassis_mode_;
  ros::ServiceClient chassis_mode_client_;
  
  // 前后相机目标检测
  ros::Subscriber front_enemy_sub_;
  ros::Subscriber back_enemy_sub_;
  bool find_the_front_enemy_;
  geometry_msgs::PoseStamped pose_of_front_enemy_;
  bool find_the_back_enemy_;
  ros::Time last_find_back_enemy_time_;

  // 多机协作
  bool is_master_;
  ros::Subscriber goal_task_sub_;
  geometry_msgs::PoseStamped auxiliary_position_;
  bool auxiliary_;
  ros::Time last_auxiliary_time_;
  // 多机数据同步
  ros::Subscriber sync_state_sub_;
  ros::Time last_sync_state_time_;
  bool sync_supply_action_busy_;
  bool sync_buff_action_busy_;

  // 裁判系统的话题
  ros::Subscriber robot_damage_sub_;
  ArmorAttacked armor_attacked_;
  ros::Time last_armor_attacked__time_;
  /*
  red  supply_projectile_id_: 0
  red  robot id:  3 or  4
  blue supply_projectile_id_: 1
  blue robot id: 13 or 14
  */
  ros::Subscriber robot_status_sub_;
  unsigned int remain_hp_;
  unsigned int local_id_;
  bool is_blue_team_;
  
  ros::Subscriber bonus_status_sub_;
  unsigned int red_bonus_;
  unsigned int blue_bonus_;
  
  ros::Subscriber game_result_sub_;
  unsigned int result_;
  
  ros::Subscriber game_status_sub_;
  unsigned int game_status_;
  unsigned int remaining_time_;
  
  ros::Subscriber game_survivor_sub_;
  bool red3_survived_;
  bool red4_survived_;
  bool blue13_survived_;
  bool blue14_survived_;
  
  ros::Subscriber remain_bullet_num_sub_;
  int remain_bullet_num_;
  
  ros::Subscriber supplier_status_sub_;
  unsigned int supply_projectile_id_;
  unsigned int supply_robot_id_;
  unsigned int supply_projectile_step_;


  // 每分钟重置补弹和Buff
  int minute_counter_;
  bool has_a_supply_chance_;
  bool has_a_buff_chance_;
  bool goto_buff_finished_;
  bool goto_supplier_finished_;
};
} //namespace decision
#endif //MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H