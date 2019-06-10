#ifndef MODULE_DECISION_ICRA_ACTION_BEHAVIOR_H
#define MODULE_DECISION_ICRA_ACTION_BEHAVIOR_H

#include <unistd.h>
#include <ros/ros.h>
#include "behavior_tree.h"
#include "blackboard.h"
#include "goals.h"

namespace decision {

class RunAwayEnemyField : public ActionNode {
 public:
  RunAwayEnemyField(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr):
      ActionNode::ActionNode("goto_enemy_supplier", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
      { }
  virtual ~RunAwayEnemyField() = default;

 private:
  virtual void OnInitialize() {
    ROS_INFO("RunAwayEnemyField OnInitialized");
  };

  virtual BehaviorState Update() {
    goal_factory_ptr_->RunAwayEnemyFields();
    goal_factory_ptr_->UpdateActionState();
    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        ROS_INFO("RunAwayEnemyField OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("RunAwayEnemyField OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("RunAwayEnemyField OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("RunAwayEnemyField OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
}; // class RunAwayEnemyField

class TurnArmorAction : public ActionNode {
 public:
  TurnArmorAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("turn_armor", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
      { }
  virtual ~TurnArmorAction() = default;

 private:
  virtual void OnInitialize() {
    ROS_INFO("TurnArmorAction OnInitialized");
  };

  virtual BehaviorState Update() {
    goal_factory_ptr_->TurnToWoundedArmor();
    goal_factory_ptr_->UpdateActionState();
    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelWhirl();
        ROS_INFO("TurnArmorAction OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("TurnArmorAction OnTerminate...SUCCESS");
        goal_factory_ptr_->CancelGoal();
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("TurnArmorAction OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("TurnArmorAction OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
}; // class TurnArmorAction

class WhirlAction : public ActionNode {
 public:
  WhirlAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr):
      ActionNode::ActionNode("whirl_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
      { }
  virtual ~WhirlAction() = default;
 private:
  virtual void OnInitialize() {
    ROS_INFO("WhirlAction OnInitialized");
  };

  virtual BehaviorState Update() {
    return goal_factory_ptr_->Whirl();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        ROS_INFO("WhirlAction OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("WhirlAction OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("WhirlAction OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("WhirlAction OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
}; // WhirlAction

class StopAction : public ActionNode {
 public:
  StopAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr):
      ActionNode::ActionNode("stop_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
      { }

  virtual ~StopAction() = default;
 private:
  virtual void OnInitialize() {
    ROS_INFO("StopAction OnInitialized");
  };

  virtual BehaviorState Update() {
    // Just Do Nothing
     return BehaviorState::RUNNING;
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        // goal_factory_ptr_->CancelGoal();
        ROS_INFO("StopAction OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("StopAction OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("StopAction OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("StopAction OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
}; // StopAction

class WaitBuffAction : public ActionNode {
 public:
  WaitBuffAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr):
      ActionNode::ActionNode("wait_buff", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
      { }
  virtual ~WaitBuffAction() = default;
 private:
  virtual void OnInitialize() {
    ROS_INFO("WaitBuffAction OnInitialized");
    goal_factory_ptr_->SendSyncState(SyncStateType::BUFF_BUSY);
  };

  virtual BehaviorState Update() {
    goal_factory_ptr_->SendSyncState(SyncStateType::BUFF_BUSY);
    // 触发 BUFF 等候时也能射击敌人，并通知另外一台机器人
    if (blackboard_ptr_->GetFrontEnemyDetected()){
      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
      blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);
      goal_factory_ptr_->SendGoalTask(blackboard_ptr_->GetFrontEnemyPose());
    } else {
      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
    }

    if (blackboard_ptr_->GetBonusStatus() == 2) {
      return BehaviorState::SUCCESS;
    } else {
      return BehaviorState::RUNNING;
    }
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->SendSyncState(SyncStateType::FREE);
        // 设置为 false 后下次可再执行 goto buff
        blackboard_ptr_->goto_buff_finished_ = false;
        // 这一分钟内的 BUFF 机会已经使用掉了
        blackboard_ptr_->has_a_buff_chance_ = false;
        ROS_INFO("WaitBuffAction OnTerminate...Reset");
        // TODO(Debug) GetBonusStatus() => 2 will Reset WaitBuffAction.
        break;
      case BehaviorState::SUCCESS:
        goal_factory_ptr_->SendSyncState(SyncStateType::FREE);
        // 设置为 false 后下次可再执行 goto buff
        blackboard_ptr_->goto_buff_finished_ = false;
        // 这一分钟内的 BUFF 机会已经使用掉了
        blackboard_ptr_->has_a_buff_chance_ = false;
        ROS_INFO("WaitBuffAction OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("WaitBuffAction OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("WaitBuffAction OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
}; // WaitBuffAction

class AuxiliaryAction : public ActionNode {
 public:
  AuxiliaryAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("auxiliary_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
      { }
  virtual ~AuxiliaryAction() = default;
 private:
  virtual void OnInitialize() {
    ROS_INFO("AuxiliaryAction OnInitialized");
  };

  virtual BehaviorState Update() {
    goal_factory_ptr_->GoAuxiliaryPosition();
    goal_factory_ptr_->UpdateActionState();
    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        ROS_INFO("AuxiliaryAction OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("AuxiliaryAction OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("AuxiliaryAction OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("AuxiliaryAction OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
}; // AuxiliaryAction

class SupplyAction : public ActionNode {
 public:
  SupplyAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr):
      ActionNode::ActionNode("bullet_supply", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
      { }
  virtual ~SupplyAction() = default;

 private:
  virtual void OnInitialize() {
    ROS_INFO("SupplyAction OnInitialized");
    goal_factory_ptr_->SendSyncState(SyncStateType::SUPPLY_BUSY);
  };

  virtual BehaviorState Update() {
    goal_factory_ptr_->SendSyncState(SyncStateType::SUPPLY_BUSY);
    goal_factory_ptr_->AddBullet();
    goal_factory_ptr_->UpdateBulletState();
    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelBulletGoal();
        goal_factory_ptr_->CancelGoal();
        ROS_INFO("SupplyAction OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        // 更新子弹数量
        // blackboard_ptr_->UpdateRemainBulletNum();
        goal_factory_ptr_->SendSyncState(SyncStateType::FREE);
        // 设置为 false 后下次可再执行 goto supplier
        blackboard_ptr_->goto_supplier_finished_ = false;
        // 这一分钟内的补弹机会已经使用掉了
        blackboard_ptr_->has_a_supply_chance_ = false;
        ROS_INFO("SupplyAction OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("SupplyAction OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("SupplyAction OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
}; // class SupplyAction


class TurnBackAction : public ActionNode {
 public:
  TurnBackAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("turn_back", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
      { }
  virtual ~TurnBackAction() = default;

 private:
  virtual void OnInitialize() {
    ROS_INFO("TurnBackAction OnInitialized");
    goal_factory_ptr_->CancelGoal(); // Do I need this function?
  };

  virtual BehaviorState Update() {
    goal_factory_ptr_->TurnBack();
    //leonard modify
    //goal_factory_ptr_->UpdateActionState();
    goal_factory_ptr_->UpdateTurnAngleState();
    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        /***leonard modify*****/
        // goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelTurnAngleGoal();
        goal_factory_ptr_->CancelWhirl();
        ROS_INFO("TurnBackAction OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("TurnBackAction OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("TurnBackAction OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("TurnBackAction OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class TurnBackAction

class GotoBuff : public ActionNode {
 public:
  GotoBuff(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr):
      failure_count_(0), ActionNode::ActionNode("goto_buff", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
      { }
  virtual ~GotoBuff() = default;

 private:
  virtual void OnInitialize() {
    // 避免两机器人同时去BUFF导致冲突，通知另一辆机器人暂时不要再来BUFF
    goal_factory_ptr_->SendSyncState(SyncStateType::BUFF_BUSY);
    // 统计失败次数提高鲁棒性，最多允许 3次 失败
    failure_count_ = 0;
    ROS_INFO("GotoBuff OnInitialized");
  };

  virtual BehaviorState Update() {
    goal_factory_ptr_->SendSyncState(SyncStateType::BUFF_BUSY);
    goal_factory_ptr_->GotoBuffGoal();
    goal_factory_ptr_->UpdateActionState();
    if (goal_factory_ptr_->GetActionState()==BehaviorState::FAILURE) {
      failure_count_++;
      if (failure_count_ == 3) {
        return BehaviorState::FAILURE;
      }
      return BehaviorState::RUNNING;
    } else {
      failure_count_ = 0;
    }
    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        ROS_INFO("GotoBuff OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        // 设置为 true 后将不再执行 goto buff
        blackboard_ptr_->goto_buff_finished_ = true;
        ROS_INFO("GotoBuff OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        // 如果去BUFF Goal失败，释放同步占用，把机会给另一机器人去抢BUFF
        goal_factory_ptr_->SendSyncState(SyncStateType::FREE);
        ROS_INFO("GotoBuff OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("GotoBuff OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
  unsigned int failure_count_;
}; // class GotoBuff

class GotoEnemySupplier : public ActionNode {
 public:
  GotoEnemySupplier(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr):
      ActionNode::ActionNode("goto_enemy_supplier", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
      { }
  virtual ~GotoEnemySupplier() = default;

 private:
  virtual void OnInitialize() {
    ROS_INFO("GotoEnemySupplier OnInitialized");
  };

  virtual BehaviorState Update() {
    goal_factory_ptr_->GotoEnemySupplierGoal();
    goal_factory_ptr_->UpdateActionState();
    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        ROS_INFO("GotoEnemySupplier OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("GotoEnemySupplier OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("GotoEnemySupplier OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("GotoEnemySupplier OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
}; // class GotoEnemySupplier

class GotoEnemyBuff : public ActionNode {
 public:
  GotoEnemyBuff(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr):
      ActionNode::ActionNode("goto_enemy_buff", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
      { }
  virtual ~GotoEnemyBuff() = default;

 private:
  virtual void OnInitialize() {
    ROS_INFO("GotoEnemyBuff OnInitialized");
  };

  virtual BehaviorState Update() {
    goal_factory_ptr_->GotoEnemyBuffGoal();
    goal_factory_ptr_->UpdateActionState();
    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        ROS_INFO("GotoEnemyBuff OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("GotoEnemyBuff OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("GotoEnemyBuff OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("GotoEnemyBuff OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
}; // class GotoEnemyBuff

class GotoSupplier : public ActionNode {
 public:
  GotoSupplier(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr):
      failure_count_(0), ActionNode::ActionNode("goto_supplier", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
      { }
  virtual ~GotoSupplier() = default;

 private:
  virtual void OnInitialize() {
    // 避免两机器人同时去补弹导致冲突，通知另一辆机器人暂时不要再来补弹
    goal_factory_ptr_->SendSyncState(SyncStateType::SUPPLY_BUSY);
    // 统计失败次数提高鲁棒性，最多允许 3次 失败
    failure_count_ = 0;
    ROS_INFO("GotoSupplier OnInitialized");
  };

  virtual BehaviorState Update() {
    goal_factory_ptr_->SendSyncState(SyncStateType::SUPPLY_BUSY);
    goal_factory_ptr_->GotoSupplierGoal();
    goal_factory_ptr_->UpdateActionState();
    if (goal_factory_ptr_->GetActionState()==BehaviorState::FAILURE) {
      failure_count_++;
      if (failure_count_ == 3) {
        return BehaviorState::FAILURE;
      }
      return BehaviorState::RUNNING;
    } else {
      failure_count_ = 0;
    }
    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        ROS_INFO("GotoSupplier OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        blackboard_ptr_->goto_supplier_finished_ = true;
        ROS_INFO("GotoSupplier OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        // 如果去补给站失败，释放同步占用，把机会给另一机器人去补弹
        goal_factory_ptr_->SendSyncState(SyncStateType::FREE);
        ROS_INFO("GotoSupplier OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("GotoSupplier OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
  unsigned int failure_count_;
}; // class GotoSupplier

class SearchAction : public ActionNode {
 public:
  SearchAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr):
     failure_count_(0), ActionNode::ActionNode("search_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
     { }
  virtual ~SearchAction() = default;

 private:
  virtual void OnInitialize() {
    failure_count_ = 0;
    ROS_INFO("SearchAction OnInitialized");
  };

  virtual BehaviorState Update() {
    if (!(goal_factory_ptr_->SearchValid())) {
      return BehaviorState::FAILURE;
    }
    goal_factory_ptr_->SearchGoal();
    goal_factory_ptr_->UpdateActionState();
    if (goal_factory_ptr_->GetActionState()==BehaviorState::FAILURE) {
      failure_count_++;
      if (failure_count_ == 3) {
        return BehaviorState::FAILURE;
      }
      return BehaviorState::RUNNING;
    } else {
      failure_count_ = 0;
    }
   return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelSearch();
        ROS_INFO("SearchAction OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("SearchAction OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("SearchAction OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("SearchAction OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
  unsigned int failure_count_;
}; // class SearchAction

class EscapeAction : public ActionNode {
 public:
  EscapeAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("escape_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) { }
  virtual ~EscapeAction() = default;

 private:
  virtual void OnInitialize() {
    ROS_INFO("EscapeAction OnInitialized");
  };

  virtual BehaviorState Update() {
    goal_factory_ptr_->EscapeGoal();
    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelWhirl();
        ROS_INFO("EscapeAction OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("EscapeAction OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("EscapeAction OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("EscapeAction OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
}; // class EscapeAction

class ChaseAction : public ActionNode {
 public:
  ChaseAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("chase_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) 
      { }
  virtual ~ChaseAction() = default;

 private:
  virtual void OnInitialize() {
    ROS_INFO("ChaseAction OnInitialized");
  };

  virtual BehaviorState Update() {
    goal_factory_ptr_->ChaseGoal();
    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelWhirl();
        ROS_INFO("ChaseAction OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("ChaseAction OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("ChaseAction OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("ChaseAction OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
}; // class ChaseAction

class ShootAction : public ActionNode {
 public:
  ShootAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("shoot_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) 
      { }
  virtual ~ShootAction() = default;

 private:
  virtual void OnInitialize() {
    ROS_INFO("ShootAction OnInitialized");
  };

  virtual BehaviorState Update() {
    blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
    blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);
    goal_factory_ptr_->SendGoalTask(blackboard_ptr_->GetFrontEnemyPose());
    return BehaviorState::RUNNING;
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelWhirl();
        ROS_INFO("ShootAction OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("ShootAction OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("ShootAction OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("ShootAction OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
}; // class ShootAction

class PatrolAction : public ActionNode {
 public:
  PatrolAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("patrol_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) 
      { }
  virtual ~PatrolAction() = default;

 private:
  virtual void OnInitialize() {
    ROS_INFO("PatrolAction OnInitialized");
  };

  virtual BehaviorState Update() {
    goal_factory_ptr_->PatrolGoal();
    goal_factory_ptr_->UpdateActionState();
    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        ROS_INFO("PatrolAction OnTerminate...Reset");
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("PatrolAction OnTerminate...SUCCESS");
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("PatrolAction OnTerminate...FAILURE");
        break;
      default:
        ROS_INFO("PatrolAction OnTerminate...ERROR");
        return;
    }
  }
  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
}; // class PatrolAction

}

#endif //MODULE_DECISION_ICRA_ACTION_BEHAVIOR_H
