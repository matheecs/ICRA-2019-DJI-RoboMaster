//
// Created by cxn on 19-4-20.
//

/***
 * 只能给color方用的伪裁判系统
 *
 *
 * master 必须是红3或者蓝3
 *
 */

#include "test_refree_node.h"

namespace leonard_serial_common {

    TestRefreeNode::TestRefreeNode(std::string name) {
        game_status_pub_1_ = nh_.advertise<roborts_msgs::GameStatus>("/master/game_status", 30);
        bonus_status_pub_1_ = nh_.advertise<roborts_msgs::BonusStatus>("/master/field_bonus_status", 30);
        supplier_status_pub_1_ = nh_.advertise<roborts_msgs::SupplierStatus>("/master/field_supplier_status", 30);
        robot_bonus_pub_1_ = nh_.advertise<roborts_msgs::RobotBonus>("/master/robot_bonus", 30);
        game_survival_pub_1_ = nh_.advertise<roborts_msgs::GameSurvivor>("/master/game_survivor", 30);

        game_status_pub_2_ = nh_.advertise<roborts_msgs::GameStatus>("/wing/game_status", 30);
        bonus_status_pub_2_ = nh_.advertise<roborts_msgs::BonusStatus>("/wing/field_bonus_status", 30);
        supplier_status_pub_2_ = nh_.advertise<roborts_msgs::SupplierStatus>("/wing/field_supplier_status", 30);
        robot_bonus_pub_2_ = nh_.advertise<roborts_msgs::RobotBonus>("/wing/robot_bonus", 30);
        game_survival_pub_2_ = nh_.advertise<roborts_msgs::GameSurvivor>("/wing/game_survivor", 30);

        test_refree_srv_ = nh_.advertiseService("/test_refree", &TestRefreeNode::TestRefreeService, this);
        timer_ = nh_.createTimer(ros::Duration(1), &TestRefreeNode::TimerCallback, this);


        //ros subscriber
        sub_projectile_supply_1_ = nh_.subscribe("/master/projectile_supply", 1,
                                                 &TestRefreeNode::ProjectileSupplyCallbackMaster,
                                                 this);
        sub_projectile_supply_2_ = nh_.subscribe("/wing/projectile_supply", 1,
                                                 &TestRefreeNode::ProjectileSupplyCallbackWing,
                                                 this);
        sub_pose_1_ = nh_.subscribe("/master/amcl_pose", 2,
                                    &TestRefreeNode::PoseCallbackMaster, this);
        sub_pose_2_ = nh_.subscribe("/wing/amcl_pose", 2,
                                    &TestRefreeNode::PoseCallbackWing, this);
        sub_robot_status_1_ = nh_.subscribe("/master/robot_status", 2, &TestRefreeNode::RobotStatusCallback, this);
    }


    void TestRefreeNode::RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &msg) {
        if (msg->id == 13 || msg->id == 14) {
            color = 1;//蓝方为1
        } else {
            color = 0;//红方为0
        }
    }


    bool TestRefreeNode::TestRefreeService(roborts_base::TestRefree::Request &req,
                                           roborts_base::TestRefree::Response &res) {
        switch (static_cast<uint8_t>(req.test_cmd)) {
            case 0://停止比赛,所有状态置0
                Reset();
                break;
            case 1://开始比赛
                Reset();
                s_game_status_.game_progress = 4;
                s_game_status_.stage_remain_time = 180;
                break;
            case 2://红buff开始踩
                bonus_array_[0].ask[0] = true;
                break;
            case 3://红buff离开
                bonus_array_[0].quit[0] = true;
                break;
            case 4://红buff开始踩
                bonus_array_[0].ask[1] = true;
                break;
            case 5://红buff离开
                bonus_array_[0].quit[1] = true;
                break;
            case 12://蓝buff开始踩
                bonus_array_[1].ask[0] = true;
                break;
            case 13://蓝buff离开
                bonus_array_[1].quit[0] = true;
                break;
            case 14://蓝buff开始踩
                bonus_array_[1].ask[1] = true;
                break;
            case 15://蓝buff离开
                bonus_array_[1].quit[1] = true;
                break;

            case 22://红3请求补弹
                supply_array_[0].ask = true;
                supply_array_[0].ask_id = 3;
                break;
            case 23://红4请求补弹
                supply_array_[0].ask = true;
                supply_array_[0].ask_id = 4;
                break;
            case 32://蓝3请求补弹
                supply_array_[1].ask = true;
                supply_array_[1].ask_id = 13;
                break;
            case 33://蓝4请求补弹
                supply_array_[1].ask = true;
                supply_array_[1].ask_id = 14;
                break;
            case 40:
                s_game_robot_survivors_.red3 = 0;
                break;
            case 41:
                s_game_robot_survivors_.red4 = 0;
                break;
            case 42:
                s_game_robot_survivors_.blue3 = 0;
                break;
            case 43:
                s_game_robot_survivors_.blue4 = 0;
                break;
            case 50:
                s_game_robot_survivors_.red3 = 1;
                break;
            case 51:
                s_game_robot_survivors_.red4 = 1;
                break;
            case 52:
                s_game_robot_survivors_.blue3 = 1;
                break;
            case 53:
                s_game_robot_survivors_.blue4 = 1;
                break;

            default:
                res.received = false;
                return false;
        }
        res.received = true;
        return true;
    }


    uint16_t TestRefreeNode::GetGameTime() {
        return s_game_status_.stage_remain_time;
    }

    void TestRefreeNode::TimerCallback(const ros::TimerEvent &) {

        if (s_game_status_.game_progress == 4) {

            /*************BUFF状态机************/
            if (GetGameTime() % 60 == 0 && GetGameTime() != 0) {
                bonus_array_[0].allow = bonus_array_[1].allow = true;

                //19.5.5 modifiy
                for (uint8_t i = 0; i < 2; i++) {
                    for (uint8_t j = 0; j < 2; j++) {
                        occupy_bonus_[i][j] = last_occupy_bonus_[i][j] = false;
                    }
                }

                std::cout << "You can ask bonus!" << std::endl;
            }
            for (uint8_t i = 0; i < 2; i++) {
                for (uint8_t robot_id = 0; robot_id < 2; robot_id++) {
                    if (bonus_array_[i].ask[robot_id]) {
                        if (bonus_array_[i].allow) {
                            if (bonus_array_[i].event[robot_id] == 2) {
                                bonus_array_[i].parallel_event[robot_id] = 1;
                                bonus_array_[i].parallel_cnt[robot_id] = GetGameTime();
                            } else if (bonus_array_[i].event[robot_id] == 0) {
                                bonus_array_[i].event[robot_id] = 1;
                                bonus_array_[i].cnt[robot_id] = GetGameTime();
                                bonus_array_[i].parallel_event[robot_id] = 0;
                            }
                        } else {
                            std::cout << "You can't ask bonus " << (int) i << ", now time: " << (int) GetGameTime()
                                      << std::endl;
                        }
                        bonus_array_[i].ask[robot_id] = false;
                    }

                    if (bonus_array_[i].event[robot_id] == 1) {
                        if (bonus_array_[i].cnt[robot_id] - GetGameTime() == 5) {
                            bonus_array_[i].event[0] = bonus_array_[i].event[1] = 2;
                            bonus_array_[i].cnt[0] = bonus_array_[i].cnt[1] = GetGameTime();
                            bonus_array_[i].allow = false;
                            std::cout << "You get bonus " << (int) i << std::endl;
                            if (i == color) {
                                RobotBonusPublish(true);
                            }

                        }
                    } else if (bonus_array_[i].event[robot_id] == 2) {
                        if (bonus_array_[i].cnt[robot_id] - GetGameTime() == 30) {
                            bonus_array_[i].event[robot_id] = 0;
                            if (robot_id == 1) {
                                std::cout << "You lost bonus " << (int) i << std::endl;
                                if (i == color) {
                                    RobotBonusPublish(false);
                                }
                            }
                        }
                        if (bonus_array_[i].parallel_event[robot_id] == 1) {
                            if (bonus_array_[i].parallel_cnt[robot_id] - GetGameTime() == 5) {
                                bonus_array_[i].event[0] = bonus_array_[i].event[1] = 2;
                                bonus_array_[i].cnt[0] = bonus_array_[i].cnt[1] = GetGameTime();
                                bonus_array_[i].allow = false;
                                std::cout << "You get bonus " << (int) i << std::endl;
                                bonus_array_[i].parallel_event[robot_id] = 0;
                            }
                            if (bonus_array_[i].event[robot_id] == 0) {
                                bonus_array_[i].event[robot_id] = bonus_array_[i].parallel_event[robot_id];
                                bonus_array_[i].cnt[robot_id] = bonus_array_[i].parallel_cnt[robot_id];
                                bonus_array_[i].parallel_event[robot_id] = 0;
                            }
                        }
                    }

                    if (bonus_array_[i].quit[robot_id]) {
                        if (bonus_array_[i].event[robot_id] == 1) {
                            bonus_array_[i].event[robot_id] = 0;
                        } else if (bonus_array_[i].quit[robot_id] && bonus_array_[i].event[robot_id] == 2 &&
                                   bonus_array_[i].parallel_event[robot_id] == 1) {
                            bonus_array_[i].parallel_event[robot_id] = 0;
                        }
                        bonus_array_[i].quit[robot_id] = false;
                    }
                }
            }


            /************SUPPLY状态机************/
            if (GetGameTime() % 60 == 0 && GetGameTime() != 0) {
                supply_array_[0].allow_cnt = supply_array_[1].allow_cnt = 2;
                std::cout << "Supplier reset!" << std::endl;
            }

            for (uint8_t i = 0; i < 2; i++) {
                if (supply_array_[i].ask == true) {
                    if (supply_array_[i].allow_cnt == 2 ||
                        (supply_array_[i].allow_cnt == 1 && supply_array_[i].ask_num == 1)) {
                        supply_array_[i].allow_cnt -= supply_array_[i].ask_num;
                        if (supply_array_[i].suppling == false) {
                            supply_array_[i].supply_id = supply_array_[i].ask_id;
                            supply_array_[i].step = 0;
                            supply_array_[i].cnt = GetGameTime();
                            supply_array_[i].suppling = true;
                        }
                    } else {
                        std::cout << "You can't ask supply color: " << (int) i << std::endl;
                    }
                    supply_array_[i].ask = false;
                }

                if (supply_array_[i].suppling == true) {

                    if (supply_array_[i].cnt - GetGameTime() == 1 && supply_array_[i].step == 0) {
                        supply_array_[i].step = 1;
                        SupplierPublish(supply_array_[i].supply_id, supply_array_[i].step, supply_array_[i].ask_num);
                    } else if (supply_array_[i].cnt - GetGameTime() == 2 && supply_array_[i].step == 1) {
                        supply_array_[i].step = 2;
                        SupplierPublish(supply_array_[i].supply_id, supply_array_[i].step, supply_array_[i].ask_num);
                    } else if (supply_array_[i].cnt - GetGameTime() == 5 && supply_array_[i].step == 2) {
                        //这里有个bug,如果是上一分钟的最后几秒请求补弹,结果下一分钟才补完,就会返回"supplier 0 still has 2!"
                        //实际上的裁判系统以落弹时间计数,而不是按请求时间计数
                        std::cout << "supplier " << (int) (i) << " still has " << (int) (supply_array_[i].allow_cnt)
                                  << "!" << std::endl;
                        supply_array_[i].step = 0;
                        SupplierPublish(supply_array_[i].supply_id, supply_array_[i].step, supply_array_[i].ask_num);
                        supply_array_[i].suppling = false;
                    }
                }
            }


            /************Forbidden状态机************/
            for (uint8_t i = 0; i < 2; i++) {
                if (forbidden_array_[i].ask == true) {
                    if (forbidden_array_[i].is_died == 0) {
                        forbidden_array_[i].cnt = GetGameTime();
                        forbidden_array_[i].is_died = 1;
                    } else if (forbidden_array_[i].is_died == 2) {
                        std::cout << "!!!!!!!!!!" << std::endl;
                        std::cout << "robot" << (int) i << " has died" << std::endl;
                        std::cout << "!!!!!!!!!!" << std::endl;
                    }
                    forbidden_array_[i].ask = false;
                }

                if (forbidden_array_[i].is_died == 1) {
                    std::cout << "robot" << (int) i << " dying: " << forbidden_array_[i].cnt - GetGameTime()
                              << std::endl;
                    if (forbidden_array_[i].cnt - GetGameTime() >= 5) {
                        forbidden_array_[i].is_died = 2;
                        std::cout << "!!!!!!!!!!" << std::endl;
                        std::cout << "robot" << (int) i << " died in forbidden zone" << std::endl;
                        std::cout << "!!!!!!!!!!" << std::endl;
                    }
                }

                if (forbidden_array_[i].quit == true) {
                    if (forbidden_array_[i].is_died == 1) {
                        forbidden_array_[i].is_died = 0;
                    }
                    forbidden_array_[i].quit = false;
                }
            }


            if (s_game_status_.stage_remain_time-- == 0) {
                std::cout << "Game Over!" << std::endl;
                Reset();
            }
        }


        /******************发布********************/
        roborts_msgs::GameStatus game_status;
        game_status.game_status = s_game_status_.game_progress;
        game_status.remaining_time = s_game_status_.stage_remain_time;
        game_status_pub_1_.publish(game_status);
        game_status_pub_2_.publish(game_status);

        roborts_msgs::BonusStatus bonus_status;
        bonus_status.red_bonus = (bonus_array_[0].event[0] > bonus_array_[0].event[1]) ? bonus_array_[0].event[0]
                                                                                       : bonus_array_[0].event[1];
        bonus_status.blue_bonus = (bonus_array_[1].event[0] > bonus_array_[1].event[1]) ? bonus_array_[1].event[0]
                                                                                        : bonus_array_[1].event[1];
        bonus_status_pub_1_.publish(bonus_status);
        bonus_status_pub_2_.publish(bonus_status);

        roborts_msgs::GameSurvivor game_survivor;
        game_survivor.red3 = s_game_robot_survivors_.red3;
        game_survivor.red4 = s_game_robot_survivors_.red4;
        game_survivor.blue3 = s_game_robot_survivors_.blue3;
        game_survivor.blue4 = s_game_robot_survivors_.blue4;
        game_survival_pub_1_.publish(game_survivor);
        game_survival_pub_2_.publish(game_survivor);
    }


    void TestRefreeNode::Reset() {
        s_game_status_.game_progress = 3;
        s_game_status_.stage_remain_time = 1;
        bonus_array_[0].Reset();
        bonus_array_[1].Reset();
        supply_array_[0].Reset();
        supply_array_[1].Reset();
        forbidden_array_[0].Reset();
        forbidden_array_[1].Reset();
        RobotBonusPublish(false);
    }


    void
    TestRefreeNode::ProjectileSupplyCallbackMaster(const roborts_msgs::ProjectileSupply::ConstPtr projectile_supply) {
        if (projectile_supply->supply) {
            supply_array_[color].ask = true;
            if (color == 0)
                supply_array_[color].ask_id = 3;
            else
                supply_array_[color].ask_id = 13;
            supply_array_[color].ask_num = projectile_supply->supply;
        }
    }

    void
    TestRefreeNode::ProjectileSupplyCallbackWing(const roborts_msgs::ProjectileSupply::ConstPtr projectile_supply) {
        if (projectile_supply->supply) {
            supply_array_[color].ask = true;
            if (color == 0)
                supply_array_[color].ask_id = 4;
            else
                supply_array_[color].ask_id = 14;
            supply_array_[color].ask_num = projectile_supply->supply;
        }
    }

    void TestRefreeNode::PoseCallbackMaster(const geometry_msgs::PoseStamped::ConstPtr msg) {
        PoseCallbackCmd(0, msg);
    }

    void TestRefreeNode::PoseCallbackWing(const geometry_msgs::PoseStamped::ConstPtr msg) {
        PoseCallbackCmd(1, msg);
    }

    void TestRefreeNode::PoseCallbackCmd(uint8_t robot_id, const geometry_msgs::PoseStamped::ConstPtr msg) {
        /********Buff区,从外到里ask,从里到外quit*********/
        for (uint8_t i = 0; i < 2; i++) {
            if (msg->pose.position.x > buff_point_[i][0] && msg->pose.position.x < buff_point_[i][1]
                && msg->pose.position.y > buff_point_[i][2] && msg->pose.position.y < buff_point_[i][3]) {
                if (occupy_bonus_[robot_id][i] == false) {
                    occupy_bonus_[robot_id][i] = true;
                }
            } else if (occupy_bonus_[robot_id][i] == true) {
                occupy_bonus_[robot_id][i] = false;
            }

            //robot_id号车进入了第i个buff区,如果车是红色,则第i个buff区有请求buff
            //如果是车是蓝色的,比如踩到了i=0的buff_point_中,对应的是本方的buff区(蓝buff),那么应该是(~i & 1)个buff区有请求
            // bonus_array_[2]的红蓝是绝对的

            //! 19.5.5 这里有个缺陷:即车从外到内或者从内到外,只ask一次,没什么问题
            //!        但一分钟刷新后,应该将occupy_bonus_与last_occupy_bonus_都置为false,否则,车一直趴在buff区上,只ask一次了
            if (occupy_bonus_[robot_id][i] == true && last_occupy_bonus_[robot_id][i] == false) {
                if (color == 0) {
                    if (bonus_array_[i].quit[robot_id])
                        bonus_array_[i].quit[robot_id] = false;
                    bonus_array_[i].ask[robot_id] = true;
                } else {
                    if (bonus_array_[(~i & 1)].quit[robot_id])
                        bonus_array_[(~i & 1)].quit[robot_id] = false;
                    bonus_array_[(~i & 1)].ask[robot_id] = true;
                }
                last_occupy_bonus_[robot_id][i] = occupy_bonus_[robot_id][i];
            } else if (occupy_bonus_[robot_id][i] == false && last_occupy_bonus_[robot_id][i] == true) {
                if (color == 0) {
                    if (bonus_array_[i].ask[robot_id])
                        bonus_array_[i].ask[robot_id] = false;
                    bonus_array_[i].quit[robot_id] = true;
                } else {
                    if (bonus_array_[(~i & 1)].ask[robot_id])
                        bonus_array_[(~i & 1)].ask[robot_id] = false;
                    bonus_array_[(~i & 1)].quit[robot_id] = true;
                }
                last_occupy_bonus_[robot_id][i] = occupy_bonus_[robot_id][i];
            }
        }

        /*****补给站,从外到里ask,从里到外quit*****/
        if (msg->pose.position.x > forbidden_point_[1][0] && msg->pose.position.x < forbidden_point_[1][1]
            && msg->pose.position.y > forbidden_point_[1][2] && msg->pose.position.y < forbidden_point_[1][3]) {
            if (occupy_forbidden_[robot_id] == false) {
                occupy_forbidden_[robot_id] = true;
            }
        } else if (occupy_forbidden_[robot_id] == true) {
            occupy_forbidden_[robot_id] = false;
        }

        if (occupy_forbidden_[robot_id] == true && last_forbidden_bonus_[robot_id] == false) {
            forbidden_array_[robot_id].ask = true;
            last_forbidden_bonus_[robot_id] = occupy_forbidden_[robot_id];
        } else if (occupy_forbidden_[robot_id] == false && last_forbidden_bonus_[robot_id] == true) {
            forbidden_array_[robot_id].quit = true;
            last_forbidden_bonus_[robot_id] = occupy_forbidden_[robot_id];
        }
    }


    void TestRefreeNode::SupplierPublish(uint8_t robot_id, uint8_t step, uint8_t num) {
        roborts_msgs::SupplierStatus supplier_status;
        supplier_status.supply_projectile_id = 1;
        if (step != 1)
            supplier_status.supply_robot_id = 0;
        else
            supplier_status.supply_robot_id = robot_id;
        supplier_status.supply_projectile_step = step;
        supplier_status.supply_projectile_num = num * 50;

        if ((color == 0 && (robot_id == 3 || robot_id == 4)) || (color == 1 && (robot_id == 13 || robot_id == 14))) {
            if (robot_id == 3 || robot_id == 13) {
                supplier_status_pub_1_.publish(supplier_status);
            } else {
                supplier_status_pub_2_.publish(supplier_status);
            }
        }
    }

    void TestRefreeNode::RobotBonusPublish(bool bonus) {
        roborts_msgs::RobotBonus robotBonus;
        robotBonus.bonus = bonus;
        robot_bonus_pub_1_.publish(robotBonus);
        robot_bonus_pub_2_.publish(robotBonus);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_refree_node");
    leonard_serial_common::TestRefreeNode test_refree_node("test_refree_node");
    ros::spin();
    ros::waitForShutdown();
    return 0;
}