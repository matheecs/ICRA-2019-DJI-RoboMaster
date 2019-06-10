#ifndef MODULE_DECISION_ICRA_ACTION_BEHAVIOR_H
#define MODULE_DECISION_ICRA_ACTION_BEHAVIOR_H

#include <unistd.h>
#include <ros/ros.h>

#include "behavior_tree.h"
#include "blackboard.h"
#include "goals.h"

namespace decision {

    class OutJammingAction : public ActionNode {
    public:
        OutJammingAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("WaitAction", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {}

        virtual ~OutJammingAction() = default;

    private:
        virtual void OnInitialize() {
            ROS_INFO("OutJammingAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            goal_factory_ptr_->OutJammingGoal();
            return BehaviorState::RUNNING;
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    ROS_INFO("OutJammingAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    ROS_INFO("OutJammingAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    ROS_INFO("OutJammingAction OnTerminate...FAILURE");
                    break;
                default:
                    ROS_INFO("OutJammingAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
    };


    class TwistAction : public ActionNode {
    public:
        TwistAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("TwistAction", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {}

        virtual ~TwistAction() = default;

    private:
        virtual void OnInitialize() {
            blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
            blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);
            ROS_INFO("TwistAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            return BehaviorState::RUNNING;
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelWhirl();
                    ROS_INFO("TwistAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    ROS_INFO("TwistAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    ROS_INFO("TwistAction OnTerminate...FAILURE");
                    break;
                default:
                    ROS_INFO("TwistAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
    };


    // 用来最开始停在启动区以及比赛停止停车
    class WaitAction : public ActionNode {
    public:
        WaitAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("WaitAction", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {}

        virtual ~WaitAction() = default;

    private:
        virtual void OnInitialize() {
            blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELAX);
            blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
            goal_factory_ptr_->CancelWhirl();
            //ROS_INFO("WaitAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            return BehaviorState::RUNNING;
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    //      ROS_INFO("WaitAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    //    ROS_INFO("WaitAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    //  ROS_INFO("WaitAction OnTerminate...FAILURE");
                    break;
                default:
                    //ROS_INFO("WaitAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
    }; // WaitAction

    // 用来test
    class TestAction : public ActionNode {
    public:
        TestAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("TestAction", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {}

        virtual ~TestAction() = default;

    private:
        virtual void OnInitialize() {
            blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
            blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
            goal_factory_ptr_->CancelWhirl();
            ROS_INFO("TestAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            return BehaviorState::RUNNING;
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    //      ROS_INFO("TestAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    //    ROS_INFO("TestAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    //  ROS_INFO("TestAction OnTerminate...FAILURE");
                    break;
                default:
                    //ROS_INFO("TestAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
    }; // TestAction



    class TurnToHurtAction : public ActionNode {
    public:
        TurnToHurtAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("turn_to_hurt_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {}

        virtual ~TurnToHurtAction() = default;

    private:
        virtual void OnInitialize() {

            ROS_INFO("TurnToHurtAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            double yaw;
            switch (blackboard_ptr_->GetArmorAttacked()) {
                case ArmorAttacked::LEFT:
                    yaw = M_PI / 2.;
//                    ROS_INFO("Hurt@LEFT");
                    break;
                case ArmorAttacked::BACK:
                    yaw = M_PI;
                    //ROS_INFO("Hurt@BACK");
                    break;
                case ArmorAttacked::RIGHT:
                    yaw = -M_PI / 2.;
                    //ROS_INFO("Hurt@RIGHT");
                    break;
                default:
                    break;
            }
            goal_factory_ptr_->TurnAngleTask(yaw);
            goal_factory_ptr_->UpdateTurnAngleState();
            return goal_factory_ptr_->GetTurnAngleState();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelTurnAngleGoal();
                    goal_factory_ptr_->CancelWhirl();
                    ROS_INFO("TurnToHurtAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    ROS_INFO("TurnToHurtAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    ROS_INFO("TurnToHurtAction OnTerminate...FAILURE");
                    break;
                default:
                    ROS_INFO("TurnToHurtAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
    }; // class TurnTOHurtArmorAction

    class TurnToBackAction : public ActionNode {
    public:
        TurnToBackAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("turn_to_back_direction", blackboard_ptr),
                goal_factory_ptr_(goal_factory_ptr) {}

        virtual ~TurnToBackAction() = default;

    private:
        virtual void OnInitialize() {

            ROS_INFO("TurnToBackAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            goal_factory_ptr_->TurnAngleTask(M_PI);
            goal_factory_ptr_->UpdateTurnAngleState();
            return goal_factory_ptr_->GetTurnAngleState();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelTurnAngleGoal();
                    goal_factory_ptr_->CancelWhirl();
                    ROS_INFO("TurnToBackAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    ROS_INFO("TurnToBackAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    ROS_INFO("TurnToBackAction OnTerminate...FAILURE");
                    break;
                default:
                    ROS_INFO("TurnToBackAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;

    }; // class TurnToBackAction


    class AimSupplyAction : public ActionNode {
    public:
        AimSupplyAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("aim_supply_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {}

        virtual ~AimSupplyAction() = default;

    private:
        virtual void OnInitialize() {
            ROS_INFO("AimSupplyAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            goal_factory_ptr_->AddBullet();
            goal_factory_ptr_->UpdateAimSupplyState();
            return goal_factory_ptr_->GetAimSupplyState();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelAimSupplyGoal();
                    blackboard_ptr_->SetSupplyDoing(false);
                    blackboard_ptr_->SetSupplyAimming(false);
                    ROS_INFO("AimSupplyAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    blackboard_ptr_->SetSupplyDoing(false);
                    blackboard_ptr_->SetSupplyAimming(false);
                    ROS_INFO("AimSupplyAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    blackboard_ptr_->SetSupplyDoing(false);
                    blackboard_ptr_->SetSupplyAimming(false);
                    blackboard_ptr_->SetCantseeFlagTrue();
                    ROS_INFO("AimSupplyAction OnTerminate...FAILURE");
                    break;
                default:
                    ROS_INFO("AimSupplyAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
    }; // class AimSupplyAction


    class GoSupplyAction : public ActionNode {
    public:
        GoSupplyAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("go_supply_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {}

        virtual ~GoSupplyAction() = default;

    private:
        virtual void OnInitialize() {

            blackboard_ptr_->SetSupplyDoing(true);
            ROS_INFO("GoSupplyAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {

            goal_factory_ptr_->GoBulletGoal();
            goal_factory_ptr_->UpdatePlannerState();
            return goal_factory_ptr_->GetPlannerState();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelPlannerGoal();
                    blackboard_ptr_->SetSupplyDoing(false);
                    ROS_INFO("GoSupplyAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    blackboard_ptr_->SetSupplyAimming(true);
                    ROS_INFO("GoSupplyAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    blackboard_ptr_->SetSupplyDoing(false);
                    ROS_INFO("GoSupplyAction OnTerminate...FAILURE");
                    break;
                default:
                    blackboard_ptr_->SetSupplyDoing(false);
                    ROS_INFO("GoSupplyAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
    }; // class GoSupplyAction


    class OutSupplierAction : public ActionNode {
    public:
        OutSupplierAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("out_supplier_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {}

        virtual ~OutSupplierAction() = default;

    private:
        virtual void OnInitialize() {
            ROS_INFO("OutSupplierAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {

            goal_factory_ptr_->OutSupplierGoal();
            goal_factory_ptr_->UpdatePlannerState();
            return goal_factory_ptr_->GetPlannerState();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelPlannerGoal();
                    goal_factory_ptr_->CancelWhirl();
                    blackboard_ptr_->SetOutSpplierFlag(false);
                    ROS_INFO("OutSupplierAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    blackboard_ptr_->SetOutSpplierFlag(false);
                    ROS_INFO("OutSupplierAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    blackboard_ptr_->SetOutSpplierFlag(false);
                    ROS_INFO("OutSupplierAction OnTerminate...FAILURE");
                    break;
                default:
                    blackboard_ptr_->SetOutSpplierFlag(false);
                    ROS_INFO("OutSupplierAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
    }; // class OutSupplierAction


    class OutForbiddenAction : public ActionNode {
    public:
        OutForbiddenAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("out_forbidden_action", blackboard_ptr),
                goal_factory_ptr_(goal_factory_ptr) {}

        virtual ~OutForbiddenAction() = default;

    private:
        virtual void OnInitialize() {

            ROS_INFO("OutForbiddenAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {

            goal_factory_ptr_->OutForbiddenGoal();
            goal_factory_ptr_->UpdatePlannerState();
            return goal_factory_ptr_->GetPlannerState();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelPlannerGoal();
                    goal_factory_ptr_->CancelWhirl();
                    ROS_INFO("OutForbiddenAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    ROS_INFO("OutForbiddenAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    ROS_INFO("OutForbiddenAction OnTerminate...FAILURE");
                    break;
                default:
                    ROS_INFO("OutForbiddenAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
    }; // class OutForbiddenAction



    class GainBuffAction : public ActionNode {
    public:
        GainBuffAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("gain_buff_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

        }

        virtual ~GainBuffAction() = default;

    private:
        virtual void OnInitialize() {
            blackboard_ptr_->SetBonusGoing(true);
            goal_factory_ptr_->bonus_index_ = 0;
            ROS_INFO("GainBuffAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            return goal_factory_ptr_->GainBuffGoal();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelPlannerGoal();
                    goal_factory_ptr_->CancelWhirl();
                    blackboard_ptr_->SetBonusGoing(false);
                    ROS_INFO("GainBuffAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    ROS_INFO("GainBuffAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    blackboard_ptr_->SetBonusGoing(false);
                    ROS_INFO("GainBuffAction OnTerminate...FAILURE");
                    break;
                default:
                    blackboard_ptr_->SetBonusGoing(false);
                    ROS_INFO("GainBuffAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;

    }; // class GainBuffAction



    class GoEnemyBuffAction : public ActionNode {
    public:
        GoEnemyBuffAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("go_enemy_buff_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

        }

        virtual ~GoEnemyBuffAction() = default;

    private:
        virtual void OnInitialize() {

            ROS_INFO("GoEnemyBuffAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            goal_factory_ptr_->GoEnemyBuffGoal();
            goal_factory_ptr_->UpdatePlannerState();
            return goal_factory_ptr_->GetPlannerState();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelPlannerGoal();
                    goal_factory_ptr_->CancelWhirl();
                    ROS_INFO("GoEnemyBuffAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    ROS_INFO("GoEnemyBuffAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    ROS_INFO("GoEnemyBuffAction OnTerminate...FAILURE");
                    break;
                default:
                    ROS_INFO("GoEnemyBuffAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;

    }; // class GoEnemyBuffAction


    class GoEnemySupplierAction : public ActionNode {
    public:
        GoEnemySupplierAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("go_enemysupplier_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

        }

        virtual ~GoEnemySupplierAction() = default;

    private:
        virtual void OnInitialize() {

            ROS_INFO("GoEnemySupplierAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            goal_factory_ptr_->GoEnemySupplierGoal();
            goal_factory_ptr_->UpdatePlannerState();
            return goal_factory_ptr_->GetPlannerState();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelPlannerGoal();
                    goal_factory_ptr_->CancelWhirl();
                    ROS_INFO("GoEnemySupplierAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    ROS_INFO("GoEnemySupplierAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    ROS_INFO("GoEnemySupplierAction OnTerminate...FAILURE");
                    break;
                default:
                    ROS_INFO("GoEnemySupplierAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;

    }; // class GoEnemySupplierAction




    class WhirlAction : public ActionNode {
    public:
        WhirlAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("whirl_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

        }

        virtual ~WhirlAction() = default;

    private:
        virtual void OnInitialize() {
            blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
            blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELAX);
            ROS_INFO("WhirlAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {

            return goal_factory_ptr_->Whirl();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelWhirl();
                    ROS_INFO("WhirlAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:

                    break;
                case BehaviorState::FAILURE:

                    break;
                default:

                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;

    }; // class WhirlAction

    class SearchAction : public ActionNode {
    public:
        SearchAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                failure_count_(0), ActionNode::ActionNode("search_action", blackboard_ptr),
                goal_factory_ptr_(goal_factory_ptr) {}

        virtual ~SearchAction() = default;

    private:
        virtual void OnInitialize() {
            failure_count_ = 0;
            ROS_INFO("SearchAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            if (!(goal_factory_ptr_->SearchValid())) {
                //ROS_INFO("No SearchValid!");
                return BehaviorState::FAILURE;
            }
            goal_factory_ptr_->SearchGoal(1.5);
            goal_factory_ptr_->UpdatePlannerState();
            if (goal_factory_ptr_->GetPlannerState() == BehaviorState::FAILURE) {
                failure_count_++;
                if (failure_count_ == 3) {
                    return BehaviorState::FAILURE;
                }
                return BehaviorState::RUNNING;
            } else {
                failure_count_ = 0;
            }
            return goal_factory_ptr_->GetPlannerState();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelPlannerGoal();
                    goal_factory_ptr_->CancelSearch();
                    ROS_INFO("SearchAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    ROS_INFO("SearchAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    //ROS_INFO("SearchAction OnTerminate...FAILURE");
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
                ActionNode::ActionNode("escape_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {}

        virtual ~EscapeAction() = default;

    private:
        virtual void OnInitialize() {
            ROS_INFO("EscapeAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            goal_factory_ptr_->EscapeGoal();
            return goal_factory_ptr_->GetPlannerState();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelPlannerGoal();
                    goal_factory_ptr_->CancelWhirl();
                    ROS_INFO("EscapeAction OnTerminate...IDLE");
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
                ActionNode::ActionNode("chase_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {
        }

        virtual ~ChaseAction() = default;

    private:
        virtual void OnInitialize() {
            ROS_INFO("ChaseAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            goal_factory_ptr_->ChaseGoalCxn(2.0, 0.8);
            return goal_factory_ptr_->GetPlannerState();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelPlannerGoal();
                    goal_factory_ptr_->CancelWhirl();
                    ROS_INFO("ChaseAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
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
                ActionNode::ActionNode("shoot_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {
        }

        virtual ~ShootAction() = default;

    private:
        virtual void OnInitialize() {
            blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
            blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);
            ROS_INFO("ShootAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            return BehaviorState::RUNNING;
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelWhirl();
                    ROS_INFO("ShootAction OnTerminate...IDLE");
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

    class WingPatrolAction : public ActionNode {
    public:
        WingPatrolAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("patrol_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {
        }

        virtual ~WingPatrolAction() = default;

    private:
        virtual void OnInitialize() {
            ROS_INFO("WingPatrolAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            goal_factory_ptr_->WingPatrolGoal();
            goal_factory_ptr_->UpdatePlannerState();
            return goal_factory_ptr_->GetPlannerState();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelPlannerGoal();
                    ROS_INFO("WingPatrolAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    ROS_INFO("WingPatrolAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    ROS_INFO("WingPatrolAction OnTerminate...FAILURE");
                    break;
                default:
                    ROS_INFO("WingPatrolAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
    }; // class WingPatrolAction




    class MasterPatrolAction : public ActionNode {
    public:
        MasterPatrolAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("patrol_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {
        }

        virtual ~MasterPatrolAction() = default;

    private:
        virtual void OnInitialize() {
            ROS_INFO("MasterPatrolAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            goal_factory_ptr_->MasterPatrolGoal();
            goal_factory_ptr_->UpdatePlannerState();
            return goal_factory_ptr_->GetPlannerState();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelPlannerGoal();
                    ROS_INFO("MasterPatrolAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    ROS_INFO("MasterPatrolAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    ROS_INFO("MasterPatrolAction OnTerminate...FAILURE");
                    break;
                default:
                    ROS_INFO("MasterPatrolAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
    }; // class MasterPatrolAction


    class WaitPatrolAction : public ActionNode {
    public:
        WaitPatrolAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("patrol_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {
        }

        virtual ~WaitPatrolAction() = default;

    private:
        virtual void OnInitialize() {
            blackboard_ptr_->wait_flag_ = true;
            ROS_INFO("WaitPatrolAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            goal_factory_ptr_->WaitPatrolGoal();
            return goal_factory_ptr_->GetPlannerState();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    blackboard_ptr_->wait_flag_ = false;
                    goal_factory_ptr_->CancelPlannerGoal();
                    goal_factory_ptr_->CancelWhirl();
                    goal_factory_ptr_->SetWaitPatrolCountZero();
                    ROS_INFO("WaitPatrolAction OnTerminate...IDLE");
                    break;
                case BehaviorState::SUCCESS:
                    blackboard_ptr_->wait_flag_ = false;
                    ROS_INFO("WaitPatrolAction OnTerminate...SUCCESS");
                    break;
                case BehaviorState::FAILURE:
                    ROS_INFO("WaitPatrolAction OnTerminate...FAILURE");
                    break;
                default:
                    ROS_INFO("WaitPatrolAction OnTerminate...ERROR");
                    return;
            }
        }

        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
    }; // class WaitPatrolAction


    class AuxiliaryAction : public ActionNode {
    public:
        AuxiliaryAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("auxiliary_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {}

        virtual ~AuxiliaryAction() = default;

    private:
        virtual void OnInitialize() {
            ROS_INFO("AuxiliaryAction OnInitialize...Done");
        };

        virtual BehaviorState Update() {
            goal_factory_ptr_->GoAuxiliaryPosition(2.0);
            goal_factory_ptr_->UpdatePlannerState();
            return goal_factory_ptr_->GetPlannerState();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state) {
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelPlannerGoal();
                    goal_factory_ptr_->CancelWhirl();
                    ROS_INFO("AuxiliaryAction OnTerminate...IDLE");
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

}

#endif //MODULE_DECISION_ICRA_ACTION_BEHAVIOR_H
