#include <ros/ros.h>
#include "icra_decision/behavior_tree.h"
#include "icra_decision/actions.h"

int main(int argc, char **argv){
  // ros::start() bring error Couldn't find an AF_INET address for []!
  ros::init(argc, argv, "decision_node");

  // make_shared<...>()
  auto blackboard_ptr = std::make_shared<decision::Blackboard>();
  auto goal_factory_ptr = std::make_shared<decision::GoalFactory>(blackboard_ptr);
 
  // Action 可重复使用
  auto stop_action = std::make_shared<decision::StopAction>(blackboard_ptr, goal_factory_ptr);
  auto turn_back = std::make_shared<decision::TurnBackAction>(blackboard_ptr, goal_factory_ptr);
  auto turn_armor = std::make_shared<decision::TurnArmorAction>(blackboard_ptr, goal_factory_ptr);
  auto shoot_action  = std::make_shared<decision::ShootAction >(blackboard_ptr, goal_factory_ptr);
  auto chase_action  = std::make_shared<decision::ChaseAction >(blackboard_ptr, goal_factory_ptr);
  auto search_action = std::make_shared<decision::SearchAction>(blackboard_ptr, goal_factory_ptr);
  auto patrol_action = std::make_shared<decision::PatrolAction>(blackboard_ptr, goal_factory_ptr);
  auto escape_action = std::make_shared<decision::EscapeAction>(blackboard_ptr, goal_factory_ptr);
  auto auxiliary_action = std::make_shared<decision::AuxiliaryAction>(blackboard_ptr, goal_factory_ptr);
  auto whirl_action = std::make_shared<decision::WhirlAction>(blackboard_ptr, goal_factory_ptr);

  auto goto_supplier = std::make_shared<decision::GotoSupplier>(blackboard_ptr, goal_factory_ptr);
  auto bullet_supply = std::make_shared<decision::SupplyAction>(blackboard_ptr, goal_factory_ptr);
  
  auto goto_buff = std::make_shared<decision::GotoBuff>(blackboard_ptr, goal_factory_ptr);
  auto wait_buff = std::make_shared<decision::WaitBuffAction>(blackboard_ptr, goal_factory_ptr);
  
  auto goto_enemy_supplier = std::make_shared<decision::GotoEnemySupplier>(blackboard_ptr, goal_factory_ptr);
  auto goto_enemy_buff = std::make_shared<decision::GotoEnemyBuff>(blackboard_ptr, goal_factory_ptr);
  auto enemy_field_action = std::make_shared<decision::RunAwayEnemyField>(blackboard_ptr, goal_factory_ptr);
  
  // Condition 不可重复使用
  auto game_status_selector = std::make_shared<decision::SelectorNode>("game_status_selector", blackboard_ptr);
  auto game_stop_condition = std::make_shared<decision::PreconditionNode>(
                                  "game_stop_condition",
                                  blackboard_ptr,
                                  stop_action,
                                  [&]() {
                                    // 比赛还未开始，勿动
                                    if (blackboard_ptr->GameStopCondition()) {
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);
  auto game_start_selector = std::make_shared<decision::SelectorNode>("game_start_selector", blackboard_ptr);
  
  // Wing
  auto wing_bot_selector = std::make_shared<decision::SelectorNode>("wing_bot_selector", blackboard_ptr);
  auto wing_bot_condition = std::make_shared<decision::PreconditionNode>(
                                  "wing_bot_condition",
                                  blackboard_ptr,
                                  wing_bot_selector,
                                  [&]() {
                                    // Wing 且 Master 还活着，才执行 Wing 分支；否则 Wing 执行 Master 角色
                                    if (!blackboard_ptr->IsMaster() && blackboard_ptr->MasterSurvived()) {
                                      // ROS_INFO("This is Wing:  ");
                                      return true;
                                    } else {
                                      // ROS_INFO("This is Master:");
                                      // ROS_INFO("local_id_: %d", blackboard_ptr->local_id_);
                                      return false;
                                    }
                                  },
                                  decision::AbortType::NONE);
  auto wing_enemy_field_condition = std::make_shared<decision::PreconditionNode>(
                                  "wing_enemy_field_condition",
                                  blackboard_ptr,
                                  enemy_field_action,
                                  [&]() {
                                    if (goal_factory_ptr->CheckRobotInEnemyField()) {
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);
  // Wing BUFF动作
  auto wing_gain_buff_selector = std::make_shared<decision::SelectorNode>("wing_gain_buff_selector", blackboard_ptr);
  auto wing_gain_buff_condition = std::make_shared<decision::PreconditionNode>(
                                  "wing_gain_buff_condition",
                                  blackboard_ptr,
                                  wing_gain_buff_selector,
                                  [&]() {
                                    blackboard_ptr->ScheduledReset();
                                    // 比赛开始的第一分钟让 master 抢 BUFF，以后每分钟抢需要满足 BUFF 失效且没有人去抢BUFF
                                    if (blackboard_ptr->has_a_buff_chance_ && blackboard_ptr->GetBonusStatus() != 2 && !blackboard_ptr->IsBuffBusy()) {
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);
  auto wing_goto_buff_condition = std::make_shared<decision::PreconditionNode>(
                                  "wing_goto_buff_condition",
                                  blackboard_ptr,
                                  goto_buff,
                                  [&]() {
                                    // 执行条件：没到达 BUFF 区
                                    if (!blackboard_ptr->goto_buff_finished_) {
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);
  auto wing_wait_buff_condition = std::make_shared<decision::PreconditionNode>(
                                  "wing_wait_buff_condition",
                                  blackboard_ptr,
                                  wait_buff,
                                  [&]() {
                                    // 已经到达 BUFF 区后等待 5s触发 BUFF
                                    if (blackboard_ptr->goto_buff_finished_) {
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);
  // Wing 补给站动作
  auto wing_bullet_supplier_selector = std::make_shared<decision::SelectorNode>("wing_bullet_supplier_selector", blackboard_ptr);
  auto wing_bullet_supplier_condition = std::make_shared<decision::PreconditionNode>(
                                  "wing_bullet_supplier_condition",
                                  blackboard_ptr,
                                  wing_bullet_supplier_selector,
                                  [&]() {
                                    blackboard_ptr->ScheduledReset();
                                    // 比赛开始由 Wing 先去补弹
                                    if (blackboard_ptr->has_a_supply_chance_ && !blackboard_ptr->IsSupplyBusy()) {
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);
  auto wing_goto_supplier_condition = std::make_shared<decision::PreconditionNode>(
                                  "wing_goto_supplier_condition",
                                  blackboard_ptr,
                                  goto_supplier,
                                  [&]() {
                                    if (!blackboard_ptr->goto_supplier_finished_) {
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);
  auto wing_bullet_supply_condition = std::make_shared<decision::PreconditionNode>(
                                  "wing_bullet_supply_condition",
                                  blackboard_ptr,
                                  bullet_supply,
                                  [&]() {
                                    if (blackboard_ptr->goto_supplier_finished_) {
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);        
  // Wing 常规动作
  auto wing_detect_front_condition = std::make_shared<decision::PreconditionNode>(
                                  "wing_detect_front_condition",
                                  blackboard_ptr,
                                  shoot_action,
                                  [&]() {
                                    if (blackboard_ptr->GetFrontEnemyDetected()) {
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);
  auto wing_detect_back_condition = std::make_shared<decision::PreconditionNode>(
                                  "wing_detect_back_condition",
                                  blackboard_ptr,
                                  turn_back,
                                  [&]() {
                                    return blackboard_ptr->GetBackEnemyDetected();
                                  },
                                  decision::AbortType::BOTH);
  auto wing_detect_armor_condition = std::make_shared<decision::PreconditionNode>(
                                  "wing_detect_armor_condition",
                                  blackboard_ptr,
                                  turn_armor,
                                  [&]() {
                                    if (blackboard_ptr->GetArmorAttacked() !=  decision::ArmorAttacked::NONE &&
                                        blackboard_ptr->GetArmorAttacked() !=  decision::ArmorAttacked::FRONT)
                                      return true;
                                    else
                                      return false;
                                  },
                                  decision::AbortType::BOTH);
  auto wing_auxiliary_condition = std::make_shared<decision::PreconditionNode>(
                                  "wing_auxiliary_condition",
                                  blackboard_ptr,
                                  auxiliary_action,
                                  [&]() {
                                    if (blackboard_ptr->GetAuxiliaryState())
                                      return true;
                                    else
                                      return false;
                                  },
                                  decision::AbortType::BOTH);
  // master
  auto master_bot_selector = std::make_shared<decision::SelectorNode>("master_bot_selector", blackboard_ptr);
  auto master_enemy_field_condition = std::make_shared<decision::PreconditionNode>(
                                  "master_enemy_field_condition",
                                  blackboard_ptr,
                                  enemy_field_action,
                                  [&]() {
                                    if (goal_factory_ptr->CheckRobotInEnemyField()) {
                                      ROS_INFO("WithinEnemyField: True!");
                                      return true;
                                    } else {
                                      ROS_INFO("WithinEnemyField: False");
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);
  // master BUFF动作
  auto master_gain_buff_selector = std::make_shared<decision::SelectorNode>("master_gain_buff_selector", blackboard_ptr);
  auto master_gain_buff_condition = std::make_shared<decision::PreconditionNode>(
                                  "master_gain_buff_condition",
                                  blackboard_ptr,
                                  master_gain_buff_selector,
                                  [&]() {
                                    blackboard_ptr->ScheduledReset();
                                    // 第一分钟由 master 抢BUFF
                                    if (blackboard_ptr->has_a_buff_chance_ && blackboard_ptr->GetBonusStatus() != 2 && !blackboard_ptr->IsBuffBusy()) {
                                      // ROS_INFO("master_gain_buff_condition: true");
                                      // ROS_INFO("bonus state: %d", static_cast<int>(blackboard_ptr->GetBonusStatus()));
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);
  auto master_goto_buff_condition = std::make_shared<decision::PreconditionNode>(
                                  "master_goto_buff_condition",
                                  blackboard_ptr,
                                  goto_buff,
                                  [&]() {
                                    if (!blackboard_ptr->goto_buff_finished_) {
                                      // ROS_INFO("master_goto_buff_condition: true");
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);
  auto master_wait_buff_condition = std::make_shared<decision::PreconditionNode>(
                                  "master_wait_buff_condition",
                                  blackboard_ptr,
                                  wait_buff,
                                  [&]() {
                                    if (blackboard_ptr->goto_buff_finished_) {
                                      // ROS_INFO("master_wait_buff_condition: true");
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);
  // master 补给站动作
  auto master_bullet_supplier_selector = std::make_shared<decision::SelectorNode>("master_bullet_supplier_selector", blackboard_ptr);
  auto master_bullet_supplier_condition = std::make_shared<decision::PreconditionNode>(
                                  "master_bullet_supplier_condition",
                                  blackboard_ptr,
                                  master_bullet_supplier_selector,
                                  [&]() {
                                    blackboard_ptr->ScheduledReset();
                                    // 比赛开始由 Wing 先去补弹
                                    if (blackboard_ptr->has_a_supply_chance_ && !blackboard_ptr->IsSupplyBusy()) {
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);
  auto master_goto_supplier_condition = std::make_shared<decision::PreconditionNode>(
                                  "master_goto_supplier_condition",
                                  blackboard_ptr,
                                  goto_supplier,
                                  [&]() {
                                    if (!blackboard_ptr->goto_supplier_finished_) {
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);
  auto master_bullet_supply_condition = std::make_shared<decision::PreconditionNode>(
                                  "master_bullet_supply_condition",
                                  blackboard_ptr,
                                  bullet_supply,
                                  [&]() {
                                    if (blackboard_ptr->goto_supplier_finished_) {
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);      
  // master 常规动作
  auto master_detect_front_condition = std::make_shared<decision::PreconditionNode>(
                                  "master_detect_front_condition",
                                  blackboard_ptr,
                                  chase_action,
                                  [&]() {
                                    if (blackboard_ptr->GetFrontEnemyDetected()) {
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  },
                                  decision::AbortType::BOTH);
  auto master_detect_back_condition = std::make_shared<decision::PreconditionNode>(
                                  "master_detect_back_condition",
                                  blackboard_ptr,
                                  turn_back,
                                  [&]() {
                                    return blackboard_ptr->GetBackEnemyDetected();
                                  },
                                  decision::AbortType::BOTH);
  auto master_detect_armor_condition = std::make_shared<decision::PreconditionNode>(
                                  "master_detect_armor_condition",
                                  blackboard_ptr,
                                  turn_armor,
                                  [&]() {
                                    if (blackboard_ptr->GetArmorAttacked() !=  decision::ArmorAttacked::NONE &&
                                        blackboard_ptr->GetArmorAttacked() !=  decision::ArmorAttacked::FRONT)
                                      return true;
                                    else
                                      return false;
                                  },
                                  decision::AbortType::BOTH);
  auto master_auxiliary_condition = std::make_shared<decision::PreconditionNode>(
                                  "master_auxiliary_condition",
                                  blackboard_ptr,
                                  auxiliary_action,
                                  [&]() {
                                    if (blackboard_ptr->GetAuxiliaryState())
                                      return true;
                                    else
                                      return false;
                                  },
                                  decision::AbortType::BOTH);



  // Build the Behavior Tree
  game_status_selector->AddChildren(game_stop_condition);
  game_status_selector->AddChildren(game_start_selector);
  game_start_selector->AddChildren(wing_bot_condition);
  game_start_selector->AddChildren(master_bot_selector);
  // wing
  wing_bot_selector->AddChildren(wing_enemy_field_condition);
  wing_bot_selector->AddChildren(wing_gain_buff_condition);
  wing_bot_selector->AddChildren(wing_bullet_supplier_condition);
  wing_gain_buff_selector->AddChildren(wing_goto_buff_condition);
  wing_gain_buff_selector->AddChildren(wing_wait_buff_condition);
  wing_bullet_supplier_selector->AddChildren(wing_goto_supplier_condition);
  wing_bullet_supplier_selector->AddChildren(wing_bullet_supply_condition);
  wing_bot_selector->AddChildren(wing_detect_front_condition);
  wing_bot_selector->AddChildren(wing_detect_back_condition);
  wing_bot_selector->AddChildren(wing_detect_armor_condition);
  wing_bot_selector->AddChildren(wing_auxiliary_condition);
  wing_bot_selector->AddChildren(whirl_action);
  // master
  master_bot_selector->AddChildren(master_enemy_field_condition);
  master_bot_selector->AddChildren(master_gain_buff_condition);
  master_bot_selector->AddChildren(master_bullet_supplier_condition);
  master_gain_buff_selector->AddChildren(master_goto_buff_condition);
  master_gain_buff_selector->AddChildren(master_wait_buff_condition);
  master_bullet_supplier_selector->AddChildren(master_goto_supplier_condition);
  master_bullet_supplier_selector->AddChildren(master_bullet_supply_condition);
  master_bot_selector->AddChildren(master_detect_front_condition);
  master_bot_selector->AddChildren(master_detect_back_condition);
  master_bot_selector->AddChildren(master_detect_armor_condition);
  master_bot_selector->AddChildren(master_auxiliary_condition);
  master_bot_selector->AddChildren(search_action);
  master_bot_selector->AddChildren(patrol_action);

  // root
  decision::BehaviorTree root(game_status_selector, 25);
  // std::cout can not work in ROS?
  root.Execute();
}