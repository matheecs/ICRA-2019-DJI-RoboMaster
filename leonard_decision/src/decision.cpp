#include <ros/ros.h>

#include "leonard_decision/behavior_tree.h"
#include "leonard_decision/actions.h"
#include "leonard_decision/log.h"

int main(int argc, char **argv) {
    // ros::start() bring error Couldn't find an AF_INET address for []!
    decision::GLogWrapper gLogWrapper(argv[0]);
    ros::init(argc, argv, "decision_node");

    // make_shared<...>()
    auto blackboard_ptr = std::make_shared<decision::Blackboard>();
    auto goal_factory = std::make_shared<decision::GoalFactory>(blackboard_ptr);

    // action
    auto search_action = std::make_shared<decision::SearchAction>(blackboard_ptr, goal_factory);
    auto chase_action = std::make_shared<decision::ChaseAction>(blackboard_ptr, goal_factory);
    auto escape_action = std::make_shared<decision::EscapeAction>(blackboard_ptr, goal_factory);
    auto wing_patrol_action = std::make_shared<decision::WingPatrolAction>(blackboard_ptr, goal_factory);
    auto wait_action = std::make_shared<decision::WaitAction>(blackboard_ptr, goal_factory);
    auto turn_to_hurt_action = std::make_shared<decision::TurnToHurtAction>(blackboard_ptr, goal_factory);
    auto turn_to_back_action = std::make_shared<decision::TurnToBackAction>(blackboard_ptr, goal_factory);
    auto shoot_action = std::make_shared<decision::ShootAction>(blackboard_ptr, goal_factory);
    auto out_forbidden_action = std::make_shared<decision::OutForbiddenAction>(blackboard_ptr, goal_factory);
    auto go_supply_action = std::make_shared<decision::GoSupplyAction>(blackboard_ptr, goal_factory);
    auto aim_supply_action = std::make_shared<decision::AimSupplyAction>(blackboard_ptr, goal_factory);
    auto out_supplier_action = std::make_shared<decision::OutSupplierAction>(blackboard_ptr, goal_factory);
    auto whirl_action = std::make_shared<decision::WhirlAction>(blackboard_ptr, goal_factory);
    auto gain_buff_action = std::make_shared<decision::GainBuffAction>(blackboard_ptr, goal_factory);
    auto go_enemybuff_action = std::make_shared<decision::GoEnemyBuffAction>(blackboard_ptr, goal_factory);
    auto go_enemysupplier_action = std::make_shared<decision::GoEnemySupplierAction>(blackboard_ptr, goal_factory);
    auto auxiliary_action = std::make_shared<decision::AuxiliaryAction>(blackboard_ptr, goal_factory);
    auto test_action = std::make_shared<decision::TestAction>(blackboard_ptr, goal_factory);
    auto master_patrol_action = std::make_shared<decision::MasterPatrolAction>(blackboard_ptr, goal_factory);
    auto wait_patrol_action = std::make_shared<decision::WaitPatrolAction>(blackboard_ptr, goal_factory);

//    auto out_jamming_action = std::make_shared<decision::OutJammingAction>(blackboard_ptr, goal_factory);
    auto twist_action = std::make_shared<decision::TwistAction>(blackboard_ptr, goal_factory);


    auto bot_selector = std::make_shared<decision::SelectorNode>("bot selector", blackboard_ptr);
    auto bullet_lack_selector = std::make_shared<decision::SelectorNode>("bullet lack selector", blackboard_ptr);
    auto idle_supply_selector = std::make_shared<decision::SelectorNode>("has chance supplier selector",
                                                                         blackboard_ptr);
    auto without_buff_selector = std::make_shared<decision::SelectorNode>("without buff selector", blackboard_ptr);
    auto obtain_buff_selector = std::make_shared<decision::SelectorNode>("obtain_buff_selector", blackboard_ptr);
    auto wing_obtain_buff_selector = std::make_shared<decision::SelectorNode>("obtain_buff_selector", blackboard_ptr);
    auto without_rfid_selector = std::make_shared<decision::SelectorNode>("without rfid selector", blackboard_ptr);
    auto wing_to_master_selector = std::make_shared<decision::SelectorNode>("wing to master selector", blackboard_ptr);
    auto wing_bot_selector = std::make_shared<decision::SelectorNode>("wing bot selector", blackboard_ptr);


    auto master_receive_condition = std::make_shared<decision::PreconditionNode>(
            "master receive condition",
            blackboard_ptr,
            auxiliary_action,
            [&]() {
                if (blackboard_ptr->GetAuxiliaryState()) {
                    ROS_INFO("master receive condition true");
                    return true;
                } else {
                    return false;
                }
            },
            decision::AbortType::LOW_PRIORITY);


    auto wing_receive_condition = std::make_shared<decision::PreconditionNode>(
            "wing receive condition",
            blackboard_ptr,
            auxiliary_action,
            [&]() {
                if (blackboard_ptr->GetAuxiliaryState()) {
                    ROS_INFO("wing receive condition true");
                    return true;
                } else {
                    return false;
                }
            },
            decision::AbortType::LOW_PRIORITY);


    auto go_supply_condition = std::make_shared<decision::PreconditionNode>(
            "go supply condition",
            blackboard_ptr,
            go_supply_action,
            [&]() {
                if (blackboard_ptr->GetGoSupply()) {
                    ROS_INFO("go supply condition true");
                    return true;
                } else {
                    // ROS_INFO("go supply condition false"<<std::endl;
                    return false;
                }
            },
            decision::AbortType::BOTH);

    auto aim_supply_condition = std::make_shared<decision::PreconditionNode>(
            "aim supply condition",
            blackboard_ptr,
            aim_supply_action,
            [&]() {
                if (blackboard_ptr->GetAimSupply()) {
                    ROS_INFO("aim_supply_condition true");
                    return true;
                } else {
                    // ROS_INFO("aim_supply_condition false"<<std::endl;
                    return false;
                }
            },
            decision::AbortType::BOTH);

    auto out_supplier_condition = std::make_shared<decision::PreconditionNode>(
            "out supplier condition",
            blackboard_ptr,
            out_supplier_action,
            [&]() {
                if (blackboard_ptr->GetOutSupplier()) {
                    ROS_INFO("out_supplier_condition true");
                    return true;
                } else {
                    ROS_INFO("out_supplier_condition false");
                    return false;
                }
            },
            decision::AbortType::BOTH);


    /*******************armor detected*************************/
    auto bullet_armor_detect_condition = std::make_shared<decision::PreconditionNode>(
            "bullet_armor_detect_for_condition",
            blackboard_ptr,
            turn_to_hurt_action,
            [&]() {
                if (blackboard_ptr->GetArmorAttacked() != decision::ArmorAttacked::NONE
                    && blackboard_ptr->GetArmorAttacked() != decision::ArmorAttacked::FRONT) {
                    return true;
                } else {
                    return false;
                }
            }, decision::AbortType::LOW_PRIORITY);


    auto armor_detect_condition = std::make_shared<decision::PreconditionNode>(
            "armor detect condition",
            blackboard_ptr,
            turn_to_hurt_action,
            [&]() {
                if (blackboard_ptr->GetArmorAttacked() != decision::ArmorAttacked::NONE
                    && blackboard_ptr->GetArmorAttacked() != decision::ArmorAttacked::FRONT)
                    return true;
                else
                    return false;
            }, decision::AbortType::LOW_PRIORITY);

    auto wing_armor_detect_condition = std::make_shared<decision::PreconditionNode>(
            "wing armor detect condition",
            blackboard_ptr,
            turn_to_hurt_action,
            [&]() {
                if (blackboard_ptr->GetArmorAttacked() != decision::ArmorAttacked::NONE
                    && blackboard_ptr->GetArmorAttacked() != decision::ArmorAttacked::FRONT)
                    return true;
                else
                    return false;
            }, decision::AbortType::LOW_PRIORITY);


    auto bonus_armor_detect_condition = std::make_shared<decision::PreconditionNode>(
            "bonus_armor_detect_condition",
            blackboard_ptr,
            turn_to_hurt_action,
            [&]() {
                if (blackboard_ptr->GetArmorAttacked() != decision::ArmorAttacked::NONE
                    && blackboard_ptr->GetArmorAttacked() != decision::ArmorAttacked::FRONT) {
                    return true;
                } else {
                    return false;
                }
            }, decision::AbortType::LOW_PRIORITY);


    auto wing_bonus_armor_detect_condition = std::make_shared<decision::PreconditionNode>(
            "wing bonus_armor_detect_condition",
            blackboard_ptr,
            turn_to_hurt_action,
            [&]() {
                if (blackboard_ptr->GetArmorAttacked() != decision::ArmorAttacked::NONE
                    && blackboard_ptr->GetArmorAttacked() != decision::ArmorAttacked::FRONT) {
                    return true;
                } else {
                    return false;
                }
            }, decision::AbortType::LOW_PRIORITY);


    // 没看到敌人且受伤
    auto rfid_armor_detect_condition = std::make_shared<decision::PreconditionNode>(
            "armor detect condition",
            blackboard_ptr,
            turn_to_hurt_action,
            [&]() {
                if (blackboard_ptr->GetArmorAttacked() != decision::ArmorAttacked::NONE
                    && blackboard_ptr->GetArmorAttacked() != decision::ArmorAttacked::FRONT &&
                    !blackboard_ptr->GetEnemyDetected())
                    return true;
                else
                    return false;
            }, decision::AbortType::LOW_PRIORITY);


    /*******************color detected*************************/
    auto bullet_color_detect_condition = std::make_shared<decision::PreconditionNode>(
            "bullet_color_detect_condition",
            blackboard_ptr,
            turn_to_back_action,
            [&]() {
                return blackboard_ptr->GetColorDetected();
            },
            decision::AbortType::LOW_PRIORITY);

    auto bonus_color_detect_condition = std::make_shared<decision::PreconditionNode>(
            "bonus_color_detect_condition",
            blackboard_ptr,
            turn_to_back_action,
            [&]() {
                return blackboard_ptr->GetColorDetected();
            },
            decision::AbortType::LOW_PRIORITY);


    auto wing_bonus_color_detect_condition = std::make_shared<decision::PreconditionNode>(
            "wing_bonus_color_detect_condition",
            blackboard_ptr,
            turn_to_back_action,
            [&]() {
                return blackboard_ptr->GetColorDetected();
            },
            decision::AbortType::LOW_PRIORITY);

    auto color_detect_condition = std::make_shared<decision::PreconditionNode>(
            "color detect condition",
            blackboard_ptr,
            turn_to_back_action,
            [&]() {
                return blackboard_ptr->GetColorDetected();
            },
            decision::AbortType::LOW_PRIORITY);

    auto wing_color_detect_condition = std::make_shared<decision::PreconditionNode>(
            "wing color detect condition",
            blackboard_ptr,
            turn_to_back_action,
            [&]() {
                return blackboard_ptr->GetColorDetected();
            },
            decision::AbortType::LOW_PRIORITY);



    /**************************Front Detected****************************************/
    auto detect_chase_condition = std::make_shared<decision::PreconditionNode>(
            "detect chase condition",
            blackboard_ptr,
            chase_action,
            [&]() {
                return blackboard_ptr->GetEnemyDetected();
            },
            decision::AbortType::BOTH);


    auto wing_detect_chase_condition = std::make_shared<decision::PreconditionNode>(
            "wing detect chase condition",
            blackboard_ptr,
            chase_action,
            [&]() {
                return blackboard_ptr->GetEnemyDetected();
            },
            decision::AbortType::BOTH);


    // 我在限定区外看到敌人,或是在限定区内看到敌人
    auto rfid_detect_chase_condition = std::make_shared<decision::PreconditionNode>(
            "rfid detect chase condition",
            blackboard_ptr,
            chase_action,
            [&]() {
                if ((
                            blackboard_ptr->GetEnemyDetected() &&
                            (
                                    !(
                                            //车距离buff区很近
                                            blackboard_ptr->IsRobotInOurBuffRange()
                                            //1s内下达了去buff区的指令,避免在其他条件与blackboard_ptr->IsRobotInOurBuffRange()两个来回摇摆,毕竟定位一直在变
                                            ||
                                            (
                                                    (ros::Time::now() - blackboard_ptr->GetBuffData().bonus_go_time <
                                                     ros::Duration(1)) &&
                                                    blackboard_ptr->GetBuffData().bonus_going
                                            )
                                            //敌人拿到了buff,我方也应该去拿
                                            || blackboard_ptr->GetBuffData().enemy_bonus_occupied

                                    )
                            )
                    )
                    //发现敌人在我方buff区内
                    || blackboard_ptr->IsEnemyInOurBuffRange()
                        ) {
//                    std::cout<<(blackboard_ptr->GetEnemyDetected() << ",!(("
//                              << (ros::Time::now() - blackboard_ptr->GetBuffData().bonus_go_time <
//                                  ros::Duration(1)) << "&&" << (blackboard_ptr->GetBuffData().bonus_going) << ")||"
//                              << (blackboard_ptr->IsRobotInOurBuffRange()) << ")"<<std::endl;

                    return true;
                }
                return false;
            },
            decision::AbortType::BOTH);


    auto bullet_detect_shoot_condition = std::make_shared<decision::PreconditionNode>(
            "bullet detect shoot condition",
            blackboard_ptr,
            shoot_action,
            //chase_action,
            [&]() {
                return blackboard_ptr->GetEnemyDetected();
            },
            decision::AbortType::BOTH);


    auto bonus_detect_shoot_condition = std::make_shared<decision::PreconditionNode>(
            "bonus detect shoot condition",
            blackboard_ptr,
            shoot_action,
            [&]() {
                return blackboard_ptr->GetEnemyDetected();
            },
            decision::AbortType::BOTH);


    auto wing_bonus_detect_shoot_condition = std::make_shared<decision::PreconditionNode>(
            "wing bonus detect shoot condition",
            blackboard_ptr,
            shoot_action,
            [&]() {
                return blackboard_ptr->GetEnemyDetected();
            },
            decision::AbortType::BOTH);


    /*********************Go Enemy Bonus*********************/
    auto get_enemy_bonusing_condition = std::make_shared<decision::PreconditionNode>(
            "get enemy bonusing condition",
            blackboard_ptr,
            go_enemybuff_action,
            [&]() {
                if (blackboard_ptr->GetEnemyBonusing()) {
                    return true;
                } else {
                    return false;
                }
            },
            decision::AbortType::LOW_PRIORITY);


    auto wing_get_enemy_bonusing_condition = std::make_shared<decision::PreconditionNode>(
            "wing get enemy bonusing condition",
            blackboard_ptr,
            go_enemybuff_action,
            [&]() {
                if (
                        blackboard_ptr->GetEnemyBonusing() && (
                                ((
                                        (blackboard_ptr->GetAnotherData().bonusing ||
                                         blackboard_ptr->GetAnotherData().supplying //在主机补弹和占buff时
                                        )
                                        &&
                                        blackboard_ptr->GetAnotherData().accessible)
                                )
                                ||
                                !blackboard_ptr->GetAnotherData().accessible)
                        ) {
                    return true;
                } else {
                    return false;
                }
            },
            decision::AbortType::LOW_PRIORITY);


    /*********************Go Enemy Supplier********************/
    auto get_enemy_suppling_condition = std::make_shared<decision::PreconditionNode>(
            "get enemy suppling condition",
            blackboard_ptr,
            go_enemysupplier_action,
            [&]() {
                if (blackboard_ptr->GetEnemySupplying()) {
                    return true;
                } else {
                    return false;
                }
            },
            decision::AbortType::LOW_PRIORITY);


    auto wing_get_enemy_supplying_condition = std::make_shared<decision::PreconditionNode>(
            "wing get enemy supplying condition",
            blackboard_ptr,
            go_enemysupplier_action,
            [&]() {
                if (
                        blackboard_ptr->GetEnemySupplying() && (
                                (
                                        (
                                                (blackboard_ptr->GetAnotherData().bonusing ||
                                                 blackboard_ptr->GetAnotherData().supplying //在主机补弹和占buff时
                                                )
                                                && blackboard_ptr->GetAnotherData().accessible
                                        )
                                )
                                || !blackboard_ptr->GetAnotherData().accessible)
                        ) {
                    return true;
                } else {
                    return false;
                }
            },
            decision::AbortType::LOW_PRIORITY);



    /****************Obtain Buff************************/
    auto obtain_buff_condition = std::make_shared<decision::PreconditionNode>(
            "obtain buff condition",
            blackboard_ptr,
            obtain_buff_selector,
            [&]() {
                if (blackboard_ptr->GetBuffFlag()) {
                    // ROS_INFO("obtain_buff_condition true"<<std::endl;
                    return true;
                } else {
                    // ROS_INFO("obtain_buff_condition false"<<std::endl;
                    return false;
                }
            },
            decision::AbortType::BOTH);


    auto wing_obtain_buff_condition = std::make_shared<decision::PreconditionNode>(
            "wing obtain buff condition",
            blackboard_ptr,
            wing_obtain_buff_selector,
            [&]() {
                if (blackboard_ptr->GetBuffFlag()) {
                    //ROS_INFO("wing obtain_buff_condition true");
                    return true;
                } else {
                    //ROS_INFO("wing obtain_buff_condition false"<<std::endl;
                    return false;
                }
            },
            decision::AbortType::BOTH);

    /****************Without Rfid************************/
    auto without_rfid_condition = std::make_shared<decision::PreconditionNode>(
            "without rfid condition",
            blackboard_ptr,
            without_rfid_selector,
            [&]() {
                if (blackboard_ptr->GetRfidScannning()) {
                    // 扫描到了,就开始占领
                    ROS_INFO("without_rfid_condition false");
                    return false;
                } else {
//                    ROS_INFO("without_rfid_condition true");
                    return true;
                }
            },
            decision::AbortType::BOTH);


    auto wing_without_rfid_condition = std::make_shared<decision::PreconditionNode>(
            "wing without rfid condition",
            blackboard_ptr,
            without_rfid_selector,
            [&]() {
                if (blackboard_ptr->GetRfidScannning()) {
                    // 扫描到了,就开始占领
                    ROS_INFO("wing_without_rfid_condition false");
                    return false;
                } else {
//                    ROS_INFO("wing_without_rfid_condition true");
                    return true;
                }
            },
            decision::AbortType::BOTH);


    /****************IDLE Supply************************/
    auto master_idle_suply_condition = std::make_shared<decision::PreconditionNode>(
            "master idle supply condition",
            blackboard_ptr,
            idle_supply_selector,
            [&]() {
                if (
                        (
                                (blackboard_ptr->GetMasterIdleSupply())
                                || (blackboard_ptr->GetSupplierData().supply_doing &&
                                    (ros::Time::now() - blackboard_ptr->GetSupplierData().supply_do_time <=
                                     ros::Duration(1)))
                        )
                        // && (!blackboard_ptr->IsDangerous())
                        //如果正在拿buff,求不管缺弹与否
                        && (!blackboard_ptr->IsClosedBuff())
                        ) {
                    ROS_INFO("master_idle_suply_condition true");
                    return true;
                } else {
                    // ROS_INFO("master_idle_suply_condition false"<<std::endl;
                    return false;
                }
            },
            decision::AbortType::BOTH);

    auto wing_idle_supply_condition = std::make_shared<decision::PreconditionNode>(
            "wing idle supply condition",
            blackboard_ptr,
            idle_supply_selector,
            [&]() {
                if (
                        (
                                (blackboard_ptr->GetWingIdleSupply())
                                || (blackboard_ptr->GetSupplierData().supply_doing &&
                                    (ros::Time::now() - blackboard_ptr->GetSupplierData().supply_do_time <=
                                     ros::Duration(1)))
                        )
                        // && (!blackboard_ptr->IsDangerous())
                        //如果正在拿buff,求不管缺弹与否
                        && (!blackboard_ptr->IsClosedBuff())
                        ) {
                    ROS_INFO("wing_idle_suply_condition true");
                    return true;
                } else {
                    return false;
                }
            },
            decision::AbortType::BOTH);


    //<10颗弹放回true
    auto bullet_lack_condition = std::make_shared<decision::PreconditionNode>(
            "bullet lack condition",
            blackboard_ptr,
            bullet_lack_selector,
            [&]() {
                if (blackboard_ptr->GetRemainBullet() <= 10
                    //                     && (!blackboard_ptr->IsDangerous())//机器人并不危急
                    && (!blackboard_ptr->IsClosedBuff())//机器人并不靠近buff
//                    && ((!blackboard_ptr->NoChanceBullet()))//机器人有机会补弹了
                        ) {
                    ROS_INFO("bullet_lack_condition true");
                    return true;
                } else {
                    // ROS_INFO("bullet_lack_condition false";
                    return false;
                }
            },
            decision::AbortType::BOTH);


    auto bullet_luck_go_supply_condition = std::make_shared<decision::PreconditionNode>(
            "bullet_luck_go_supply_condition",
            blackboard_ptr,
            idle_supply_selector,
            [&]() {
                if (blackboard_ptr->GetGoSupply() || blackboard_ptr->GetAimSupply() ||
                    blackboard_ptr->GetOutSupplier()) {
                    ROS_INFO("bullet_luck_go_supply_condition true");
                    return true;
                } else {
                    // ROS_INFO("bullet_luck_go_supply_condition false"<<std::endl;
                    return false;
                }
            },
            decision::AbortType::BOTH);


    //=0颗弹,返回true
    auto bullet_zero_condition = std::make_shared<decision::PreconditionNode>(
            "bullet zero condition",
            blackboard_ptr,
            escape_action,
            [&]() {
                if (blackboard_ptr->GetTrueZeroBullet()) {
                    ROS_INFO("bullet_zero_condition true");
                    return true;
                } else {
                    // ROS_INFO("bullet_zero_condition false"<<std::endl;
                    return false;
                }
            },
            decision::AbortType::BOTH);


    // 停留在禁止区,警告标志位为1
    auto get_forbiddenWarn_condition = std::make_shared<decision::PreconditionNode>(
            "get forbiddenWarn condition",
            blackboard_ptr,
            out_forbidden_action,
            [&]() {
                if (blackboard_ptr->IsForbiddenWarn()) {
                    ROS_INFO("get_forbiddenWarn_condition true");
                    return true;
                } else {
                    // ROS_INFO("get_forbiddenWarn_condition false"<<std::endl;
                    return false;
                }
            },
            decision::AbortType::BOTH);


    // 另外一台活着,则通过条件
    auto master_alive_condition = std::make_shared<decision::PreconditionNode>(
            "master alive condition",
            blackboard_ptr,
            wing_bot_selector,
            [&]() {
                if (blackboard_ptr->GetAnotherData().alive) {
                    return true;
                } else {
                    return false;
                }
            },
            decision::AbortType::BOTH);

    auto waiting_start_condition = std::make_shared<decision::PreconditionNode>(
            "waiting start condition",
            blackboard_ptr,
            wait_action,
            [&]() {
                if (blackboard_ptr->GetStartRun()) {
                    return false;
                } else {
                    return true;
                }
            },
            decision::AbortType::BOTH);



    // 停留在禁止区,警告标志位为1
//    auto get_jamming_condition = std::make_shared<decision::PreconditionNode>(
//            "get_jamming_condition",
//            blackboard_ptr,
//            out_jamming_action,
//            [&]() {
//                if (blackboard_ptr->IsJamming()) {
//                    ROS_INFO("get_jamming_condition true");
//                    return true;
//                } else {
//                    // ROS_INFO("get_jamming_condition false"<<std::endl;
//                    return false;
//                }
//            },
//            decision::AbortType::BOTH);


    auto twist_condition = std::make_shared<decision::PreconditionNode>(
            "twist_condition",
            blackboard_ptr,
            twist_action,
            [&]() {
                if (blackboard_ptr->GetEnemeyDetectedValid(0.6)) {
                    return true;
                } else {
                    return false;
                }
            },
            decision::AbortType::BOTH);


    auto wing_twist_condition = std::make_shared<decision::PreconditionNode>(
            "wing_twist_condition",
            blackboard_ptr,
            twist_action,
            [&]() {
                if (blackboard_ptr->GetEnemeyDetectedValid(0.6)) {
                    return true;
                } else {
                    return false;
                }
            },
            decision::AbortType::BOTH);

    auto wait_patrol_condition = std::make_shared<decision::PreconditionNode>(
            "wait_patrol_condition",
            blackboard_ptr,
            wait_patrol_action,
            [&]() {
                if (!blackboard_ptr->GetAnotherData().wait) {
                    return true;
                } else {
                    return false;
                }
            },
            decision::AbortType::BOTH);

    auto nochance_master_condition = std::make_shared<decision::PreconditionNode>(
            "nochance_master_condition",
            blackboard_ptr,
            master_patrol_action,
            [&]() {
                if (blackboard_ptr->NoChanceBullet() && blackboard_ptr->GetRoleState()) {
                    return true;
                } else {
                    return false;
                }
            },
            decision::AbortType::BOTH);

    auto nochance_wing_condition = std::make_shared<decision::PreconditionNode>(
            "nochance_wing_condition",
            blackboard_ptr,
            wing_patrol_action,
            [&]() {
                if (blackboard_ptr->NoChanceBullet() && !blackboard_ptr->GetRoleState()) {
                    return true;
                } else {
                    return false;
                }
            },
            decision::AbortType::BOTH);


    bullet_lack_selector->AddChildren(
            bullet_luck_go_supply_condition);//bullet_luck_go_supply_condition->idle_supply_selector
    bullet_lack_selector->AddChildren(bullet_zero_condition);//bullet_zero_condition->escape_action
    bullet_lack_selector->AddChildren(bullet_detect_shoot_condition);
    bullet_lack_selector->AddChildren(bullet_armor_detect_condition);
    bullet_lack_selector->AddChildren(bullet_color_detect_condition);
    bullet_lack_selector->AddChildren(nochance_master_condition);
    bullet_lack_selector->AddChildren(nochance_wing_condition);
    bullet_lack_selector->AddChildren(wait_patrol_condition);
    bullet_lack_selector->AddChildren(whirl_action);

    idle_supply_selector->AddChildren(go_supply_condition);
    idle_supply_selector->AddChildren(aim_supply_condition);
    idle_supply_selector->AddChildren(out_supplier_condition);

    without_buff_selector->AddChildren(obtain_buff_condition);//obtain_buff_condition->obtain_buff_selector
    without_buff_selector->AddChildren(without_rfid_condition);//without_rfid_condition->without_rfid_condition
    without_buff_selector->AddChildren(bonus_detect_shoot_condition);
    without_buff_selector->AddChildren(bonus_armor_detect_condition);
    without_buff_selector->AddChildren(bonus_color_detect_condition);
    without_buff_selector->AddChildren(whirl_action);

    obtain_buff_selector->AddChildren(master_idle_suply_condition);//master_idle_suply_condition->idle_supply_selector
    obtain_buff_selector->AddChildren(detect_chase_condition);
    obtain_buff_selector->AddChildren(armor_detect_condition);
    obtain_buff_selector->AddChildren(get_enemy_bonusing_condition);
    obtain_buff_selector->AddChildren(color_detect_condition);
    obtain_buff_selector->AddChildren(master_receive_condition);

    //!5.14add
    obtain_buff_selector->AddChildren(twist_condition);

    obtain_buff_selector->AddChildren(search_action);
    obtain_buff_selector->AddChildren(master_patrol_action);


    without_rfid_selector->AddChildren(rfid_detect_chase_condition);
    without_rfid_selector->AddChildren(rfid_armor_detect_condition);
    without_rfid_selector->AddChildren(gain_buff_action);


// Build the Behavior Tree
    if (blackboard_ptr->GetRoleState()) // Master:
    {
        bot_selector->AddChildren(waiting_start_condition);
        bot_selector->AddChildren(get_forbiddenWarn_condition);
        bot_selector->AddChildren(bullet_lack_condition);//bullet_lack_condition->bullet_lack_selector
        bot_selector->AddChildren(without_buff_selector);
    } else // Wing:
    {
        bot_selector->AddChildren(waiting_start_condition);
        bot_selector->AddChildren(get_forbiddenWarn_condition);
        bot_selector->AddChildren(bullet_lack_condition);
        bot_selector->AddChildren(master_alive_condition);//master_alive_condition->wing_bot_selector
        bot_selector->AddChildren(without_buff_selector);

        wing_bot_selector->AddChildren(
                wing_obtain_buff_condition);//wing_obtain_buff_condition->wing_obtain_buff_selector
        wing_bot_selector->AddChildren(wing_without_rfid_condition);//wing_without_rfid_condition->obtain_buff_selector
        wing_bot_selector->AddChildren(wing_bonus_detect_shoot_condition);
        wing_bot_selector->AddChildren(wing_bonus_armor_detect_condition);
        wing_bot_selector->AddChildren(wing_bonus_color_detect_condition);
        wing_bot_selector->AddChildren(whirl_action);

        wing_obtain_buff_selector->AddChildren(
                wing_idle_supply_condition);//wing_idle_supply_condition->idle_supply_selector
        wing_obtain_buff_selector->AddChildren(wing_detect_chase_condition);
        wing_obtain_buff_selector->AddChildren(wing_armor_detect_condition);
        wing_obtain_buff_selector->AddChildren(wing_get_enemy_bonusing_condition);
        wing_obtain_buff_selector->AddChildren(wing_color_detect_condition);
        wing_obtain_buff_selector->AddChildren(wing_receive_condition);

        //!5.14add
        wing_obtain_buff_selector->AddChildren(wing_twist_condition);

        wing_obtain_buff_selector->AddChildren(wing_patrol_action);

    }

    decision::BehaviorTree root(bot_selector, 25);
    root.Execute();
}


/*
 *  ||******||表示重复使用的node
 *  (********)表示上面没空间展开的node
 *
 *
 *
 *                                                   bot_selector(master)
 *                                                            |
 *       waiting_start_condition  get_forbiddenWarn_condition  bullet_lack_condition  without_buff_selector
 *              wait_action         out_forbidden_action        bullet_lack_selector
 *                                                                        |
 *                         bullet_luck_go_supply_condition  bullet_zero_condition  bullet_detect_shoot_condition  bullet_armor_detect_condition  bullet_color_detect_condition  wait_patrol_action
 *                            idle_supply_selector              escape_action               shoot_action                turn_to_hurt_action            turn_to_back_action
 *                                    |
 * go_supply_condition  aim_supply_condition  out_supplier_condition 
 *  go_supply_action     aim_supply_action     out_supplier_action        
 *                                                                                   (without_buff_selector)
 *                                                                                              |
 *                                    obtain_buff_condition    without_rfid_condition  bonus_detect_shoot_condition  bonus_armor_detect_condition  bonus_color_detect_condition   whirl_action
 *                                            |                                                shoot_action            turn_to_hurt_action              turn_to_back_action
 *                                    obtain_buff_selector
 *                                            |
 *      master_idle_suply_condition  detect_chase_condition  armor_detect_condition  get_enemy_suppling_condition  get_enemy_bonusing_condition  color_detect_condition  master_receive_condition  search_action  inner_patrol_action
 *       ||idle_supply_selector||        chase_action         turn_to_hurt_action      go_enemysupplier_action         go_enemybuff_action         turn_to_back_action       auxiliary_action
 *
 * 
 *                                                           (without_rfid_condition)
 *                                                                      |
 *                                                             without_rfid_selector
 *                                                                      |
 *                              rfid_detect_chase_condition  rfid_armor_detect_condition gain_buff_action
 *                                      chase_action            turn_to_hurt_action
 *
 *
 *
 *
 *                                                   bot_selector(wing)
 *                                                            |
 * ||waiting_start_condition||   ||get_forbiddenWarn_condition||   ||bullet_lack_condition||   master_alive_condition  ||without_buff_selector||    
 *                                                                                               wing_bot_selector
 *                                                                                                        |
 *                                    wing_obtain_buff_condition    wing_without_rfid_condition  wing_bonus_detect_shoot_condition  wing_bonus_armor_detect_condition  wing_bonus_color_detect_condition   whirl_action
 *                                            |                                |                             shoot_action                         turn_to_hurt_action                 turn_to_back_action
 *                                    wing_obtain_buff_selector       ||without_rfid_selector||
 *                                            |
 *      wing_idle_supply_condition  wing_detect_chase_condition  wing_armor_detect_condition  wing_get_enemy_supplying_condition  wing_get_enemy_bonusing_condition   wing_color_detect_condition  wing_receive_condition  patrol_action
 *       ||idle_supply_selector||        chase_action                turn_to_hurt_action           go_enemysupplier_action            go_enemybuff_action                  turn_to_back_action         auxiliary_action
 *
 *
 */