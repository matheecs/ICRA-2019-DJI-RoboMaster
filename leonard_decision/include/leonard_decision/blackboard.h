#ifndef MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H
#define MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>

// !!! leonard_decision/
#include "leonard_decision/EnemyDetect.h"
#include "leonard_decision/ChassisMode.h"
#include "leonard_decision/GimbalMode.h"
#include "leonard_decision/GoalTask.h"

#include "leonard_decision/GameStatus.h"
#include "leonard_decision/GameSurvivor.h"
#include "leonard_decision/BonusStatus.h"
#include "leonard_decision/SupplierStatus.h"
#include "leonard_decision/RobotStatus.h"
#include "leonard_decision/RobotBonus.h"
#include "leonard_decision/RobotDamage.h"
#include "leonard_decision/Bullet.h"
#include "types.h"
#include "log.h"

namespace decision {
    // 云台模式
    enum class GimbalMode {
        GIMBAL_RELAX = 0,
        GIMBAL_PATROL_MODE = 1,
        GIMBAL_RELATIVE_MODE = 2
    };
// 底盘模式
    enum class ChassisMode {
        AUTO_SEPARATE_GIMBAL = 0,
        DODGE_MODE = 1
    };

    enum class ArmorAttacked {
        FRONT = 0,
        LEFT = 1,
        BACK = 2,
        RIGHT = 3,
        NONE = 4
    };


    class ForbiddenData {
    public:
        ForbiddenData() = default;

        ~ForbiddenData() = default;

//        const double buff_point[2][4] = {{5.8,  6.8,  1.25, 2.25},//本方buff
//                                          {1.05, 2.35, 2.9,  3.75}};//敌人的buff
        //        double supplier_point[2][4] = {{3.5, 4.5, 4, 5},//本方forbidden(相对与敌人的禁区)
//                                         {3.5, 4.5, 0, 1}};//敌人的forbidden

        const Vec4d buff_point[2] = {Vec4d(5.8, 6.8, 1.25, 2.25),
                                     Vec4d(1.2, 2.2, 2.75, 3.75)}; //0.15
        const Vec2d buff_center[2] = {Vec2d(6.3, 1.75), Vec2d(1.7, 3.25)};

        const Vec4d supplier_point[2] = {Vec4d(3.5, 4.5, 4, 5),
                                         Vec4d(3.5, 4.5, 0, 1)};
        const Vec2d supplier_center[2] = {Vec2d(4, 4.5), Vec2d(4, 0.5)};


        bool occ_es_warn = false;
        int occ_enemysupplier_cnt = 0;
        bool occ_eb_warn = false;
        int occ_enemybuff_cnt = 0;
        bool occ_os_warn = false;
        int occ_oursupplier_cnt = 0;
        bool occ_ob_warn = false;
        int occ_ourbuff_cnt = 0;

    };

    class BuffData {
    public:
        BuffData() = default;

        ~BuffData() = default;

        //Field Bonus Info
        bool enemy_bonus_occupying = false;
        ros::Time enemy_bonus_occupying_time;
        bool enmey_bonus_warn = false; //敌人在拿buff
        bool enemy_bonus_occupied = false;//敌人拿到了buff
        ros::Time enemy_bonus_occupied_time;
        bool enemy_bonus_cnt = true;
        bool our_bonus_occupying = false;
        bool bonus_occupied = false;
        ros::Time bonus_occupied_time;
        bool bonus_cnt = true;

        bool bonus_going = false;
        bool bonus_permission = false; //去buff区的权限
        ros::Time bonus_go_time;
    };


    class AnotherRobotData {
    public:
        AnotherRobotData() = default;

        ~AnotherRobotData() = default;

        bool supplying;
        uint16_t hp;
        double pose_x;
        double pose_y;
        ros::Time time;
        bool accessible = false;  //这里指通讯有没有断
        bool alive = true;       //另外一台死了没?
        bool bonusing = false;
        bool supply_warn = false;
        bool lack_bullet = false;
        bool wait = false;
    };

    class SupplierData {
    public:
        uint8_t our_cnt = 2;
        uint8_t our_projectile_step = leonard_decision::SupplierStatus::CLOSE;
        ros::Time supply_done_time_;//!5.13add
        bool supply_done_flag_ = false;//!5.13add
        int16_t addnum;//!5.13add

        bool enemy_supply_flag = false; //当对面补给站关闭时,false
        uint8_t enemy_projectile_step = leonard_decision::SupplierStatus::CLOSE;
        ros::Time enemy_end_time;    //关闭时,记录时间

        // my supply status
        bool supply_doing = false;
        ros::Time supply_do_time;
        bool supply_aimming = false;
        bool supply_permission = false;  //true的时候才能补
        bool cantsee_flag = false;
        ros::Time cantsee_time;
        bool fireout_flag = false;
        ros::Time fireoout_time;
        uint16_t resttime = 0;
        bool outsupplier_flag = false;
    };

    class Blackboard {
    public:
        typedef std::shared_ptr<Blackboard> Ptr;

        explicit Blackboard() :
                gimbal_mode_(GimbalMode::GIMBAL_RELAX),
                chassis_mode_(ChassisMode::AUTO_SEPARATE_GIMBAL) {
            //ctor
            ros::NodeHandle nh;
            enemy_sub_ = nh.subscribe("test_enemy_pose", 1, &Blackboard::EnemyCallback, this);
            color_sub_ = nh.subscribe("enemy_back", 1, &Blackboard::ColorCallback, this);

            robot_hurt_sub_ = nh.subscribe("robot_damage", 10, &Blackboard::RobotDamageCallback, this);
            robot_info_sub_ = nh.subscribe("robot_status", 10, &Blackboard::RobotInfoCallback, this);
            game_info_sub_ = nh.subscribe("game_status", 10, &Blackboard::GameInfoCallback, this);
            bonus_info_sub_ = nh.subscribe("field_bonus_status", 10, &Blackboard::BonusInfoCallback, this);
            supplier_info_sub_ = nh.subscribe("field_supplier_status", 10, &Blackboard::SupplierInfoCallback, this);
            bullet_info_sub_ = nh.subscribe("robot_bullet", 10, &Blackboard::BulletInfoCallback, this);
            game_survivor_sub_ = nh.subscribe("game_survivor", 10, &Blackboard::GameSurvivorCallback, this);
            robot_bonus_sub_ = nh.subscribe("robot_bonus", 10, &Blackboard::RobotBonusCallback, this);
            robot_pose_sub_ = nh.subscribe("amcl_pose", 10, &Blackboard::RobotPoseCallback, this);

            chassis_mode_client_ = nh.serviceClient<leonard_decision::ChassisMode>("set_chassis_mode");
            gimbal_mode_client_ = nh.serviceClient<leonard_decision::GimbalMode>("set_gimbal_mode");

            // master-or-wing:
            nh.getParam("is_master", is_master_);

            if (is_master_) {
                supplier_data_.resttime = 20;
                remain_bullet_ = 40;
            } else {
                supplier_data_.resttime = 18;
                remain_bullet_ = 0;
            }

            if (is_master_) {
                goal_task_sub_ = nh.subscribe("/wing/goal_task", 10, &Blackboard::GoalTaskCallBack, this);
                goal_task_pub_ = nh.advertise<leonard_decision::GoalTask>("/master/goal_task", 10);
                ROS_INFO("create master receive goal subscribe");
            } else {
                goal_task_sub_ = nh.subscribe("/master/goal_task", 10, &Blackboard::GoalTaskCallBack, this);
                goal_task_pub_ = nh.advertise<leonard_decision::GoalTask>("/wing/goal_task", 10);
                ROS_INFO("create wing receive goal subscribe");
            }

            timer_ = nh.createTimer(ros::Duration(0.1), &Blackboard::TimerCallback, this);

            planner_state_sub_ = nh.subscribe("planner_state", 30, &Blackboard::PlannerStateCallback, this);

        }

        ~Blackboard() = default;


        /********************** !Callback! **********************************/

        // Robot Hurt
        void RobotDamageCallback(const leonard_decision::RobotDamage::ConstPtr &robot_hurt_data) {
            if (robot_hurt_data->damage_type == 0) {
                last_armor_attacked_time_ = ros::Time::now();
                switch (robot_hurt_data->damage_source) {
                    case 0:
                        armor_attacked_ = ArmorAttacked::FRONT;
                        //ROS_INFO("BB--ArmorAttacked::FRONT");
                        break;
                    case 1:
                        armor_attacked_ = ArmorAttacked::LEFT;
                        //ROS_INFO("BB--ArmorAttacked::LEFT");
                        break;
                    case 2:
                        armor_attacked_ = ArmorAttacked::BACK;
                        //ROS_INFO("BB--ArmorAttacked::BACK");
                        break;
                    case 3:
                        armor_attacked_ = ArmorAttacked::RIGHT;
                        //ROS_INFO("BB--ArmorAttacked::RIGHT");
                        break;
                    default:
                        armor_attacked_ = ArmorAttacked::NONE;
                        return;
                }
            } else {
                armor_attacked_ = ArmorAttacked::NONE;
            }
        }

        // robot Info
        void RobotInfoCallback(const leonard_decision::RobotStatus::ConstPtr &robot_status) {
            remain_hp_ = (robot_status->remain_hp);
            robot_id_ = (robot_status->id);
        }

        // game Info
        void GameInfoCallback(const leonard_decision::GameStatus::ConstPtr &msg) {

            if (game_status_ != leonard_decision::GameStatus::ROUND &&
                msg->game_status == leonard_decision::GameStatus::ROUND) {
                supplier_data_.our_cnt = 2;
                buff_data_.bonus_cnt = true;
                buff_data_.enemy_bonus_cnt = true;
                //std::cout << "field reset!" << std::endl;
                ROS_INFO("field reset!");
            }

            if (msg->game_status == leonard_decision::GameStatus::ROUND &&
                ((remain_time_ >= 120 && msg->remaining_time < 120) ||
                 (remain_time_ >= 60 && msg->remaining_time < 60))) {
                supplier_data_.our_cnt = 2;
                buff_data_.bonus_cnt = true;
                buff_data_.enemy_bonus_cnt = true;
//                std::cout << "field reset!" << std::endl;
                ROS_INFO("field reset!");
            }

            game_status_ = (msg->game_status);
            remain_time_ = (msg->remaining_time);
        }


        void BonusInfoCallback(const leonard_decision::BonusStatus::ConstPtr &msg) {
//            uint8
//            UNOCCUPIED = 0
//            uint8
//            BEING_OCCUPIED = 1
//            uint8
//            OCCUPIED = 2
//            uint8
//            red_bonus
//            uint8
//            blue_bonus

            if (robot_id_ == 3 || robot_id_ == 4) {
                BonusStateMachine(msg->red_bonus, msg->blue_bonus);
            } else {
                // 我们是蓝车
                BonusStateMachine(msg->blue_bonus, msg->red_bonus);
            }
        }


        void BonusStateMachine(const uint8_t &ourbonus, const uint8_t &enemybonus) {
            // 我们是蓝车
            // 蓝buff在被占领
            if (ourbonus == leonard_decision::BonusStatus::BEING_OCCUPIED) {
                buff_data_.our_bonus_occupying = true;
            } else if (ourbonus == leonard_decision::BonusStatus::OCCUPIED) {
                buff_data_.our_bonus_occupying = false;
            } else {
                buff_data_.our_bonus_occupying = false;
            }

            // 红buuf在被占领
            if (enemybonus == leonard_decision::BonusStatus::BEING_OCCUPIED) {
                // 若车在外面,且次数超过15次,即对方在加buff
                if (!IsPointInCircle(my_pose_x_, my_pose_y_, forbidden_data_.buff_center[1], 0.70)
                    && (
                            //通讯没断,另一台车也在外边 或者 车通讯断开了
                            ((!IsPointInCircle(another_data_.pose_x, another_data_.pose_y,
                                               forbidden_data_.buff_center[1], 0.70)) &&
                             another_data_.accessible)
                            || !another_data_.accessible
                    )) {
                    if (buff_data_.enemy_bonus_occupying == false) {
                        buff_data_.enemy_bonus_occupying_time = ros::Time::now();
                        buff_data_.enmey_bonus_warn = true;
                    }
                } else {
                    buff_data_.enmey_bonus_warn = false;
                }

                buff_data_.enemy_bonus_occupying = true;

            } else if (enemybonus == leonard_decision::BonusStatus::OCCUPIED) {
                buff_data_.enemy_bonus_occupying = false;
                buff_data_.enemy_bonus_occupied = true;
                buff_data_.enemy_bonus_occupied_time = ros::Time::now();
                buff_data_.enemy_bonus_cnt = false;
                buff_data_.enmey_bonus_warn = false;
            } else {
                buff_data_.enemy_bonus_occupying = false;
                buff_data_.enmey_bonus_warn = false;
            }
        }


        void RobotBonusCallback(const leonard_decision::RobotBonus::ConstPtr &msg) {
            if (msg->bonus) {
//                std::cout << "we get bonus" << std::endl;
                ROS_INFO("we get bonus");
                buff_data_.bonus_occupied_time = ros::Time::now();
                buff_data_.bonus_occupied = true;
                buff_data_.bonus_cnt = false;
            }
//            else {
//                std::cout << "we lost bonus" << std::endl;
//                buff_data_.bonus_occupied = false;
//            }
        }


        void SupplierInfoCallback(const leonard_decision::SupplierStatus::ConstPtr &msg) {
            //#supplier status
            //            uint8 CLOSE = 0
            //            uint8 PREPARING = 1
            //            uint8 SUPPLYING = 2
            //
            //            uint8 supply_projectile_id
            //            uint8 supply_robot_id
            //            uint8 supply_projectile_step
            //            uint8 supply_projectile_num

//            if (robot_id_ == 3 || robot_id_ == 4) {
//                if (msg->supply_robot_id == 13 || msg->supply_robot_id == 14) {
//                    //敌人补充弹药
//                    EnemySupplyProcess(msg);
//
//                } else if (msg->supply_robot_id == 3 || msg->supply_robot_id == 4) {
//                    // 我方补充弹药
//                    OurSupplyProcess(msg);
//                }
//
//            } else {
//                if (msg->supply_robot_id == 3 || msg->supply_robot_id == 4) {
//                    //敌人补充弹药
//                    EnemySupplyProcess(msg);
//
//                } else if (msg->supply_robot_id == 13 || msg->supply_robot_id == 14) {
//                    // 我方补充弹药
//                    OurSupplyProcess(msg);
//                }
//            }



//5.13之前的代码
//            if (msg->supply_projectile_step == leonard_decision::SupplierStatus::CLOSE
//                &&
//                supplier_data_.our_projectile_step == leonard_decision::SupplierStatus::SUPPLYING) {
//                supplier_data_.our_cnt -= msg->supply_projectile_num / 50;
//                if (supplier_data_.our_cnt < 0)
//                    supplier_data_.our_cnt = 0;
//
//                std::cout << " supplier_data_.our_cnt" << (int) (supplier_data_.our_cnt) << std::endl;
//
//                //!TODO 比赛时请注释掉!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//                if (supplier_data_.supply_aimming) {
//                    if (msg->supply_projectile_num == 100) {
//                        std::cout << "!!!!!!!!!!!!!!!You Get 100 bullet!!!!!!!!!!!!!!" << std::endl;
//                        remain_bullet_ += 100;
//                    } else if (msg->supply_projectile_num == 50) {
//                        std::cout << "!!!!!!!!!!!!!!!You Get 50 bullet!!!!!!!!!!!!!!" << std::endl;
//                        remain_bullet_ += 50;
//                    }
//                }
//            }
//            supplier_data_.our_projectile_step = msg->supply_projectile_step;


            //!5.13 add, 只有在aimming的时候，我们才需要关注这个topic
            if (supplier_data_.supply_aimming) {

                if (msg->supply_projectile_step == leonard_decision::SupplierStatus::CLOSE
                    &&
                    supplier_data_.our_projectile_step == leonard_decision::SupplierStatus::SUPPLYING) {
                    supplier_data_.supply_done_flag_ = true;
                    supplier_data_.supply_done_time_ = ros::Time::now();
                    if (msg->supply_projectile_num == 100) {
                        supplier_data_.addnum = 100;
                    } else if (msg->supply_projectile_num == 50) {
                        supplier_data_.addnum = 50;
                    }
                } else if (msg->supply_projectile_step == leonard_decision::SupplierStatus::SUPPLYING
                           &&
                           supplier_data_.our_projectile_step == leonard_decision::SupplierStatus::CLOSE) {
                    supplier_data_.supply_done_flag_ = false;
                }
            }
            supplier_data_.our_projectile_step = msg->supply_projectile_step;
        }


//        inline void EnemySupplyProcess(const leonard_decision::SupplierStatus::ConstPtr &msg) {
//            if (msg->supply_projectile_step == leonard_decision::SupplierStatus::CLOSE &&
//                supplier_data_.enemy_projectile_step == leonard_decision::SupplierStatus::SUPPLYING) {
//                supplier_data_.enemy_end_time = ros::Time::now();
//                supplier_data_.enemy_supply_flag = false;
//            }
//            if (msg->supply_projectile_step != leonard_decision::SupplierStatus::CLOSE) {
//                supplier_data_.enemy_supply_flag = true;
//            }
//            supplier_data_.enemy_projectile_step = msg->supply_projectile_step;
//        }
//
//
//        inline void OurSupplyProcess(const leonard_decision::SupplierStatus::ConstPtr &msg) {
//            if (msg->supply_projectile_step == leonard_decision::SupplierStatus::CLOSE
//                &&
//                supplier_data_.our_projectile_step == leonard_decision::SupplierStatus::SUPPLYING &&
//                supplier_data_.our_cnt > 0) {
//                supplier_data_.our_cnt--;
//
//                std::cout << " supplier_data_.our_cnt" << (int) (supplier_data_.our_cnt) << std::endl;
//
//                //!TODO 比赛时请注释掉!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//                if (msg->supply_robot_id == robot_id_) {
//                    if (supplier_data_.supply_doing)
//                        std::cout << "!!!!!!!!!!!!!!!You Get 50 bullet!!!!!!!!!!!!!!" << std::endl;
//                    remain_bullet_ += 50;
//                }
//            }
//            supplier_data_.our_projectile_step = msg->supply_projectile_step;
//        }


        void BulletInfoCallback(const leonard_decision::Bullet::ConstPtr &msg) {
            //比赛用

            last_bullet_ = true_bullet_;
            true_bullet_ = msg->bullet_num;

            if (true_bullet_ != last_bullet_) {
                //std::cout << "true_bullet" << (int) true_bullet_ << std::endl;
                ROS_INFO("true_bullet%d", (int) true_bullet_);

                if(true_bullet_-last_bullet_>45){
                    if(supplier_data_.addnum==50||supplier_data_.addnum==100){
                        supplier_data_.our_cnt -= supplier_data_.addnum / 50;
                    }
                    else{
                        supplier_data_.our_cnt -= 1;
                    }

                    if (supplier_data_.our_cnt < 0) {
                        supplier_data_.our_cnt = 0;
                    }

                    ROS_INFO("!!!!!!!!!!supplier_data_.our_cnt: %d !!!!!!!!", (int) (supplier_data_.our_cnt));
                    ROS_INFO("!!!!!!!!!!You Get %d bullet!!!!!!!!", (int) (supplier_data_.addnum));
                }
            }

            static bool cnt = false;
            if (!cnt) {
                bullet_msgs_ = msg->bullet_num;
                cnt = true;
            }

            if (msg->bullet_num == 0 && bullet_msgs_ != 0) {
                remain_bullet_ = 0;
                bullet_zero_time_ = ros::Time::now();
            } else if (msg->bullet_num < bullet_msgs_) {
                remain_bullet_ -= (bullet_msgs_ - msg->bullet_num);
            }
                //加了50颗
            else if (msg->bullet_num - bullet_msgs_ <= 51 && msg->bullet_num - bullet_msgs_ >= 10) {
                remain_bullet_ -= (bullet_msgs_ + 50 - msg->bullet_num);
            }
                //加了100颗
            else if (msg->bullet_num - bullet_msgs_ <= 101 && msg->bullet_num - bullet_msgs_ >= 60) {
                remain_bullet_ -= (bullet_msgs_ + 100 - msg->bullet_num);
            }
            if (bullet_msgs_ == 0 && msg->bullet_num == 1) {
                //记下时间
                bullet_zero_time_ = ros::Time::now();
                return;
            }
            if (bullet_msgs_ != msg->bullet_num) {
                //std::cout << "remain_bullet" << (int) remain_bullet_ << std::endl;
                ROS_INFO("remain_bullet%d", (int) remain_bullet_);
            }

            bullet_msgs_ = msg->bullet_num;

            //自己发topic or  用下位机的原生数据
            //remain_bullet_ = msg->bullet_num;
        }


        bool GetTrueZeroBullet() const {
            return GetRemainBullet() <= 0;
        }


        void GameSurvivorCallback(const leonard_decision::GameSurvivor::ConstPtr &msg) {
            // #robot survival
            // bool red3
            // bool red4
            // bool blue3
            // bool blue4

            if (robot_id_ == 3) {
                if (msg->red4 == 1)
                    another_data_.alive = true;
                else
                    another_data_.alive = false;
            } else if (robot_id_ == 4) {
                if (msg->red3 == 1)
                    another_data_.alive = true;
                else
                    another_data_.alive = false;
            } else if (robot_id_ == 13) {
                if (msg->blue4 == 1)
                    another_data_.alive = true;
                else
                    another_data_.alive = false;
            } else if (robot_id_ == 14) {
                if (msg->blue3 == 1)
                    another_data_.alive = true;
                else
                    another_data_.alive = false;
            }

        }

        void RobotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
            my_pose_x_ = msg->pose.position.x;
            my_pose_y_ = msg->pose.position.y;

            geometry_msgs::Quaternion orientation = msg->pose.orientation;
            tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
            double pitch, roll;
            mat.getEulerYPR(my_pose_yaw_, pitch, roll);


            if (IsPointInCircle(my_pose_x_, my_pose_y_, forbidden_data_.buff_center[1], 0.70)) {
                if (forbidden_data_.occ_eb_warn == false && forbidden_data_.occ_enemybuff_cnt++ >= 10) {
                    forbidden_data_.occ_eb_warn = true;
                }
            } else {
                forbidden_data_.occ_enemybuff_cnt = 0;
                if (forbidden_data_.occ_eb_warn) {
                    forbidden_data_.occ_eb_warn = false;
                }
            }

            if (IsPointInCircle(my_pose_x_, my_pose_y_, forbidden_data_.supplier_center[1], 0.95) &&
                my_pose_x_ >= 3.4) {
                if (forbidden_data_.occ_es_warn == false && forbidden_data_.occ_enemysupplier_cnt++ >= 5) {
                    forbidden_data_.occ_es_warn = true;
                }
            } else {
                forbidden_data_.occ_enemysupplier_cnt = 0;
                if (forbidden_data_.occ_es_warn) {
                    forbidden_data_.occ_es_warn = false;
                }
            }


            if (IsPointInCircle(my_pose_x_, my_pose_y_, forbidden_data_.buff_center[0], 0.70)) {
                if (forbidden_data_.occ_ob_warn == false && forbidden_data_.occ_ourbuff_cnt++ >= 10) {
                    forbidden_data_.occ_ob_warn = true;
                }
            } else {
                forbidden_data_.occ_ourbuff_cnt = 0;
                if (forbidden_data_.occ_ob_warn) {
                    forbidden_data_.occ_ob_warn = false;
                }
            }


            if (IsPointInCircle(my_pose_x_, my_pose_y_, forbidden_data_.supplier_center[0], 0.90) &&
                my_pose_x_ <= 4.6) {
                if (forbidden_data_.occ_os_warn == false && forbidden_data_.occ_oursupplier_cnt++ >= 5) {
                    forbidden_data_.occ_os_warn = true;
                }
            } else {
                forbidden_data_.occ_oursupplier_cnt = 0;
                if (forbidden_data_.occ_os_warn) {
                    forbidden_data_.occ_os_warn = false;
                }
            }

        }


        bool IsPointInSequre(const double &x, const double &y, const Vec4d &sequre) const {

            if (x > (sequre[0]) && x < (sequre[1])
                && y > (sequre[2]) && y < (sequre[3])) {
                return true;
            }
            return false;
        }


        bool IsPointInCircle(double &x, double &y, const Vec2d &circle, double r = 0.75) {
            if (EularDistance(x, circle(0), y, circle(1)) <= r) {
                return true;
            }
            return false;
        }


        //Shoot Info
        // icra_decison namespace!
        void EnemyCallback(const leonard_decision::EnemyDetect::ConstPtr &enemy) {
            if (enemy->detected) {
                if (enemy->enemy_pos.pose.position.x < -0.5 || enemy->enemy_pos.pose.position.x > 8.5 ||
                    enemy->enemy_pos.pose.position.y < -0.5 || enemy->enemy_pos.pose.position.y > 5.5) {
                    enemy_detected_ = false;
                    return;
                }
                enemy_detected_ = true;
                enemy_pose_ = enemy->enemy_pos;
                last_enemy_detected_time_ = ros::Time::now();
            } else {
                enemy_detected_ = false;
            }
        }

        void ColorCallback(const std_msgs::Bool color_detect) {
            color_detected_ = color_detect.data;
            if (color_detect.data)
                last_color_detected_time_ = ros::Time::now();
        }


        //goal task callback
        void GoalTaskCallBack(const leonard_decision::GoalTask::ConstPtr &req) {

            //            geometry_msgs/PoseStamped goal
            //            bool is_goal_set
            //            bool supplying
            //            uint16 remain_hp
            //            double pose_x;
            //            double pose_y;

            if (req->is_goal_set) {
                auxiliary_data_.auxiliary_position = req->goal;
                auxiliary_data_.last_auxiliary_time = ros::Time::now();
                SetAuxiliaryIsUsed(false);
                //ROS_INFO("receive goal task");
            }
            another_data_.hp = req->remain_hp;
            another_data_.supplying = req->supplying;
            another_data_.bonusing = req->bonusing;
            another_data_.pose_x = req->pose_x;
            another_data_.pose_y = req->pose_y;
            another_data_.time = ros::Time::now();
            another_data_.supply_warn = req->supply_warn;
            another_data_.lack_bullet = req->lack_bullet;
            another_data_.wait = req->wait;
            if ((remain_time_ >= 120 && remain_time_ <= 177) || (remain_time_ >= 60 && remain_time_ <= 117) ||
                (remain_time_ >= 0 && remain_time_ <= 57)) {
                supplier_data_.our_cnt = std::min<uint8_t>(supplier_data_.our_cnt, req->supplycnt);
            }
        }


        /*************Jamming Judge*************/
        void PlannerStateCallback(const std_msgs::Bool state) {
            jamming_judge_.goal_planner_state = state.data;

            if (game_status_ == leonard_decision::GameStatus::ROUND && GetRemainHp() > 0) {
                Vec3d pose(my_pose_x_, my_pose_y_, my_pose_yaw_);
                if (jamming_judge_.goal_planner_state) {
                    if (fabs(jamming_judge_.pose_vec(0) - pose(0)) < 0.05
                        && fabs(jamming_judge_.pose_vec(1) - pose(1)) < 0.05
                        && fabs(angle_diff<double>(jamming_judge_.pose_vec(2), pose(2)) < 0.08)) {
                        jamming_judge_.jam_cnt++;
                    } else {
                        jamming_judge_.jam_cnt = 0;
                        jamming_judge_.pose_vec(0) = pose(0);
                        jamming_judge_.pose_vec(1) = pose(1);
                        jamming_judge_.pose_vec(2) = pose(2);
                    }
                } else {
                    jamming_judge_.jam_cnt = 0;
                    jamming_judge_.pose_vec(0) = pose(0);
                    jamming_judge_.pose_vec(1) = pose(1);
                    jamming_judge_.pose_vec(2) = pose(2);
                }

                if (jamming_judge_.jam_cnt >= 60) {
                    jamming_judge_.is_jamming = true;
                    jamming_judge_.jam_cnt = 0;
                    std::cout << "<<<<<<<<<<<<<<<<JammingCnt:60<<<<<<<<<<<<<<<<" << std::endl;
                }
            }
        }

        bool IsJamming() {
            if (jamming_judge_.is_jamming) {
                jamming_judge_.is_jamming = false;
                jamming_judge_.jam_cnt = 0;
                jamming_judge_.jam_time = ros::Time::now();
            }
            return ros::Time::now() - jamming_judge_.jam_time <= ros::Duration(0.5);
        }


        void TimerCallback(const ros::TimerEvent &) {
            // 1s内有心跳,则通讯正常
            if (ros::Time::now() - another_data_.time >= ros::Duration(1)) {
                another_data_.accessible = false;
            } else {
                another_data_.accessible = true;
            }

            //如果通讯顺畅,另外一台车正在补,就锁死去补给站的权限
            if (another_data_.accessible && another_data_.supplying) {
                //如果另外一台车在占禁区，则不能补充
                if (another_data_.supply_warn) {
                    supplier_data_.supply_permission = false;
                } else {
                    if (!(supplier_data_.supply_doing && GetRoleState())) //主机正在补时,不用管这个
                        supplier_data_.supply_permission = false;
                }
            } else {
                supplier_data_.supply_permission = true;
            }

            if ((another_data_.accessible) && another_data_.bonusing) {
                if (!((buff_data_.bonus_going || GetRfidScannning()) && GetRoleState())) //主机正在补时,不用管这个
                    buff_data_.bonus_permission = false;
            } else {
                buff_data_.bonus_permission = true;
            }

            if (supplier_data_.cantsee_flag &&
                ros::Time::now() - supplier_data_.cantsee_time >= ros::Duration(30)) {
                supplier_data_.cantsee_flag = false;
            }

            if (supplier_data_.fireout_flag &&
                ros::Time::now() - supplier_data_.fireoout_time >= ros::Duration(15)) {
                supplier_data_.fireout_flag = false;
            }

            if (ros::Time::now() - buff_data_.bonus_occupied_time >= ros::Duration(30)
                && buff_data_.bonus_occupied) {
                //std::cout << "we lost bonus" << std::endl;
                ROS_INFO("we lost bonus");
                buff_data_.bonus_occupied = false;
            }

            if (ros::Time::now() - buff_data_.enemy_bonus_occupied_time >= ros::Duration(30) &&
                buff_data_.enemy_bonus_occupied) {
                //std::cout << "enemy lost bonus" << std::endl;
                ROS_INFO("enemy lost bonus");
                buff_data_.enemy_bonus_occupied = false;
            }


//             if (
//                     supplier_data_.supply_aimming &&
//                     supplier_data_.supply_done_flag_ &&
//                     (ros::Time::now() - supplier_data_.supply_done_time_ > ros::Duration(0.5))
//                     ) {
//                 remain_bullet_ += supplier_data_.addnum;
//                 supplier_data_.supply_done_flag_ = false;

//                 supplier_data_.our_cnt -= supplier_data_.addnum / 50;
//                 if (supplier_data_.our_cnt < 0) {
//                     supplier_data_.our_cnt = 0;
//                 }
// //                std::cout << "!!!!!!!!!!supplier_data_.our_cnt:" << (int) (supplier_data_.our_cnt) << " !!!!!!!!"
// //                          << std::endl;
// //
// //                std::cout << "!!!!!!!!!!!!!!!You Get " << (int) supplier_data_.addnum << " bullet!!!!!!!!!!!!!!"
// //                          << std::endl;

//                 ROS_INFO("!!!!!!!!!!supplier_data_.our_cnt: %d !!!!!!!!", (int) (supplier_data_.our_cnt));
//                 ROS_INFO("!!!!!!!!!!You Get %d bullet!!!!!!!!", (int) (supplier_data_.addnum));

//             }


            if (GetRemainHp() > 0) {
                //没死时才发信号给另外一台车
                goal_task_msg_.is_goal_set = false;
                goal_task_msg_.remain_hp = remain_hp_;
                goal_task_msg_.supplying = (supplier_data_.supply_doing);
                goal_task_msg_.bonusing = buff_data_.bonus_going || GetRfidScannning();
                goal_task_msg_.pose_x = my_pose_x_;
                goal_task_msg_.pose_y = my_pose_y_;
                goal_task_msg_.supplycnt = supplier_data_.our_cnt;
                goal_task_msg_.supply_warn = forbidden_data_.occ_os_warn;
                goal_task_msg_.lack_bullet = GetRemainBullet() <= 10;
                goal_task_msg_.wait = wait_flag_;
                goal_task_pub_.publish(goal_task_msg_);
            }
        }

        /********************** !Get Data! **********************************/

        bool GetRoleState() const {
            return is_master_;
        }

        bool GetStartRun() const {
            // 比赛开始,置为true
            return (game_status_ == leonard_decision::GameStatus::ROUND) && (GetRemainHp() > 0);
        }

        bool GetEnemyDetected() const {
            return enemy_detected_;
        }

        bool GetEnemeyDetectedValid(double value) const {
            return (ros::Time::now() - last_enemy_detected_time_ < ros::Duration(value));
        }


        geometry_msgs::PoseStamped GetEnemyPose() const {
            return enemy_pose_;
        }

        ArmorAttacked GetArmorAttacked(double interval = 0.2) const {
            // 根据热量计算,修正时间,问孙丰瑞
            if (ros::Time::now() - last_armor_attacked_time_ > ros::Duration(interval)) {
                return ArmorAttacked::NONE;
            } else {
                return armor_attacked_;
            }
        }

        bool GetColorDetected() const {
            // 0.3s内有记录,则回头
            if (ros::Time::now() - last_color_detected_time_ > ros::Duration(0.2)) {
                return false;
            } else {
                return true;
            }
        }


        bool GetEnemySupplying() const {
            if (supplier_data_.enemy_supply_flag ||
                (ros::Time::now() - supplier_data_.enemy_end_time < ros::Duration(0.1)))
                return true;
            return false;
        }

        bool GetEnemyBonusing() const {
            return buff_data_.enmey_bonus_warn &&
                   ros::Time::now() - buff_data_.enemy_bonus_occupying_time >= ros::Duration(1.0);
        }


        int16_t GetRemainBullet() const {
            //return remain_bullet_;
            return true_bullet_;
        }

        bool GetBuffFlag() const {
            // 已经有buff或者没buff次数或者没有权限去buff区
            return buff_data_.bonus_occupied || (!buff_data_.bonus_cnt) || (!buff_data_.bonus_permission) ||
                   IsDangerous()
                   || supplier_data_.supply_aimming;//加个条件,如果正在对准,就不要去拿buff了
        }


        bool GetRfidScannning() const {
            bool temp = (my_pose_x_ > (forbidden_data_.buff_point[0][0] + 0.1) &&
                         my_pose_x_<(forbidden_data_.buff_point[0][1] - 0.1) &&
                                    my_pose_y_>(forbidden_data_.buff_point[0][2]) &&
                         my_pose_y_ < (forbidden_data_.buff_point[0][3] - 0.1));
            return temp && buff_data_.our_bonus_occupying;
        }


        bool IsForbiddenWarn() {
            return forbidden_data_.occ_es_warn
                   || (forbidden_data_.occ_eb_warn && buff_data_.enemy_bonus_cnt &&
                       buff_data_.enemy_bonus_occupying)// TODO 需要注意是BUFF激活后的这段时间，占领buff区给的信号是1还是2？，如果是2，要把第三个条件删除
                   || (forbidden_data_.occ_os_warn &&
                       (
                               (!supplier_data_.supply_doing) && (!NoChanceBullet())//有机会补弹药且没在补充
                       )
                           //    (!supplier_data_.supply_permission)
                           //    ||
                           //    (
                           //            IsDangerous() &&
                           //            (!(supplier_data_.our_cnt == 0 && remain_time_ < 55))//如果快死了,别碍事,但是如果没有补弹机会可以不管
                           //    )

                   )
                   || (forbidden_data_.occ_ob_warn &&
                       (
                               (!buff_data_.bonus_permission)
                               ||
                               (
                                       IsDangerous() &&
                                       (!(buff_data_.bonus_cnt == false && remain_time_ < 50))
                               )
                               ||
                               (
                                       buff_data_.bonus_cnt && buff_data_.bonus_occupied &&
                                       buff_data_.our_bonus_occupying
                               )

                       )
                   );
        }


        bool IsDangerous() const {
            return GetRemainTureHp(remain_hp_) <= 3 &&
                   ((another_data_.accessible && GetRemainTureHp(another_data_.hp) > 3)
                    || (another_data_.alive && !another_data_.accessible));
        }


        bool IsClosedBuff() {
            return
                    (
                            //有权限且没buff
                            (buff_data_.bonus_going || GetRfidScannning())
                            &&
                            (EularDistance(my_pose_x_, forbidden_data_.buff_center[0](0),
                                           my_pose_y_, forbidden_data_.buff_center[0](1)) <
                             (EularDistance(my_pose_x_, forbidden_data_.supplier_center[0](0),
                                            my_pose_y_, forbidden_data_.supplier_center[0](1))))
                            && (!buff_data_.bonus_occupied) && buff_data_.bonus_permission

                    );
        }


        bool NoChanceBullet() const {
            return (supplier_data_.our_cnt == 0 && remain_time_ < 55);
        }


        bool GetGoSupply() const {

            if (!((!supplier_data_.cantsee_flag) && (supplier_data_.supply_permission) &&
                  (!supplier_data_.supply_aimming) && (supplier_data_.our_cnt > 0) &&
                  (!supplier_data_.fireout_flag))) {
//                std::cout << "GetGoSupply:" << (!supplier_data_.cantsee_flag) << (supplier_data_.supply_permission)
//                          << (!supplier_data_.supply_aimming)
//                          << (supplier_data_.our_cnt > 0) <<
//                          (!supplier_data_.fireout_flag) << std::endl;

                ROS_INFO("GetGoSupply:%d%d%d%d%d",
                         (!supplier_data_.cantsee_flag),
                         (supplier_data_.supply_permission),
                         (!supplier_data_.supply_aimming),
                         (supplier_data_.our_cnt > 0),
                         (!supplier_data_.fireout_flag)
                );

            }
            return (!supplier_data_.cantsee_flag) && (supplier_data_.supply_permission) &&
                   (!supplier_data_.supply_aimming) && (supplier_data_.our_cnt > 0) &&
                   (!supplier_data_.fireout_flag)
                   && (!IsDangerous());

        }

        bool GetAimSupply() {
            if (supplier_data_.supply_aimming) {

                bool temp = (
                        (!
                                (GetRemainTureHp(remain_hp_) <= 6 &&
                                 GetArmorAttacked() != ArmorAttacked::NONE)
                        )
                        || (another_data_.alive == false));

                if (temp == false) {
                    //被人打了
                    SetFireOutFlag(true);
                    SetOutSpplierFlag(true);
                }

                return supplier_data_.supply_aimming && temp;
            }

            return false;
        }

        bool GetOutSupplier() const {
            return (forbidden_data_.occ_os_warn && supplier_data_.outsupplier_flag);
        }


        bool IdleSupply() {
            bool is_vision_allow = false;
            bool is_armor_allow = false;
            bool is_pose_allow = false;
            if (GetEnemeyDetectedValid(0.3)) {
                if (std::sqrt(std::pow((GetEnemyPose().pose.position.x - forbidden_data_.supplier_center[0](0)), 2) +
                              std::pow((GetEnemyPose().pose.position.y - forbidden_data_.supplier_center[0](1)), 2)) >=
                    2.0) {
                    is_vision_allow = true;
                }
            } else {
                is_vision_allow = true;
            }
            if (GetArmorAttacked(0.5) == ArmorAttacked::NONE) {
                is_armor_allow = true;
            }
            if ((std::sqrt(std::pow((forbidden_data_.supplier_center[0](0) - my_pose_x_), 2) +
                           std::pow((forbidden_data_.supplier_center[0](1) - my_pose_y_), 2)) <= 2)
//                &&
//                !((my_pose_y_ > 4.7) && (my_pose_x_ > 4.5))
                    )
                is_pose_allow = true;
            return is_vision_allow && is_armor_allow && is_pose_allow;
        }


        bool GetWingIdleSupply() {
            return
                    (
                            ((remain_time_ > 120 && remain_time_ < 120 + supplier_data_.resttime) ||
                             (remain_time_ > 60 && remain_time_ < 60 + supplier_data_.resttime))
                            || supplier_data_.supply_aimming
                    )//时间条件
                    && (GetRemainBullet() <= 40)//弹药条件
                    && (GetGoSupply() || GetAimSupply() || GetOutSupplier())
                    && (IdleSupply()//敌人离得远或者看不见敌人 没人打 距supplier近
                        || supplier_data_.supply_aimming                                   //正在补弹
                    );
        }


        bool GetMasterIdleSupply() {
            // 与从机不同,加几个条件,对面补给区buff区没人,屁股没人
            return
                    (
                            ((remain_time_ > 120 && remain_time_ < 120 + supplier_data_.resttime) ||
                             (remain_time_ > 60 && remain_time_ < 60 + supplier_data_.resttime))
                            || supplier_data_.supply_aimming
                    )//时间条件
                    && (GetRemainBullet() <= 40)//弹药条件
                    && (GetGoSupply() || GetAimSupply() || GetOutSupplier())
                    && ((IdleSupply()                          //敌人离得远或者看不见敌人 没人打 距supplier近
                         && !GetColorDetected()
//                         && !GetEnemyBonusing()
                        )
                        || supplier_data_.supply_aimming);                                //正在补弹
        }


        bool IsRobotInOurBuffRange() const {
            return
                    (std::sqrt(std::pow(my_pose_x_ - forbidden_data_.buff_center[0](0), 2) +
                               std::pow(my_pose_y_ - forbidden_data_.buff_center[0](1), 2)) <= 3.0);
//                    && !(my_pose_x_ > 5.9 && my_pose_x_ < 6.7 && my_pose_y_ < 1.2);
        }


        bool IsEnemyInOurBuffRange() const {
            if (!GetEnemyDetected())
                return false;
            return (std::sqrt(std::pow(enemy_pose_.pose.position.x - forbidden_data_.buff_center[0](0), 2) +
                              std::pow(enemy_pose_.pose.position.y - forbidden_data_.buff_center[0](1), 2)) <
                    0.4);
//                    &&
//                   !(enemy_pose_.pose.position.x > 5.9 && enemy_pose_.pose.position.x < 6.7 &&
//                     enemy_pose_.pose.position.y < 1.2);

        }


        uint8_t DecisionSupplierCnt() const {
            if (GetAnotherData().alive == false)
                return std::max<uint8_t>(supplier_data_.our_cnt, 1);
            if ((remain_time_ > 65 && remain_time_ <= 73) || (remain_time_ > 125 && remain_time_ <= 133)) {
                return std::max<uint8_t>(supplier_data_.our_cnt, 1);
            }
            if (another_data_.accessible && GetRemainTureHp(another_data_.hp) <= 3) {
                return std::max<uint8_t>(supplier_data_.our_cnt, 1);
            }
            return 1;
        }


        uint16_t GetRemainHp() const {
            return remain_hp_;
        }

        uint16_t GetRemainTureHp(uint16_t hp) const {
            if (buff_data_.bonus_occupied) {
                return hp / 25;
            } else {
                return hp / 50;
            }
        }

        // 开始去补给站,true
        // 补弹失败 或者 初始化逃出补给区action, false
        void SetSupplyDoing(bool value) {
            supplier_data_.supply_doing = value;
            if (value) {
                supplier_data_.supply_do_time = ros::Time::now();
            }
        }

        // 到补给区成功,true
        // 补给成功false , 或逃出补给区action终止, false
        void SetSupplyAimming(bool value) {
            supplier_data_.supply_aimming = value;
        }

        void SetFireOutFlag(bool value) {
            supplier_data_.fireout_flag = value;
            if (value) {
                supplier_data_.fireoout_time = ros::Time::now();
            }
        }


        void SetOutSpplierFlag(bool value) {
            supplier_data_.outsupplier_flag = value;
        }


        //supplypid action失败后(只有看不见才会失败,若视觉没问题,定位出问题时会出现这种情况),置为true,30s后才能去补弹
        void SetCantseeFlagTrue() {
            supplier_data_.cantsee_flag = true;
            supplier_data_.cantsee_time = ros::Time::now();
        }


        void SetBonusGoing(bool value) {
            buff_data_.bonus_going = value;
            if (value) {
                buff_data_.bonus_go_time = ros::Time::now();
            }
//            std::cout << "bonusgoing:" << (int) value << std::endl;
            ROS_INFO("bonusgoing:%d", (int) value);
        }


        SupplierData GetSupplierData() {
            return supplier_data_;
        }

        BuffData GetBuffData() {
            return buff_data_;
        }

        AnotherRobotData GetAnotherData() const {
            return another_data_;
        }

        ForbiddenData GetForbiddenData() const {
            return forbidden_data_;
        }

        geometry_msgs::PoseStamped GetAuxiliaryPosition() const {
            return auxiliary_data_.auxiliary_position;
        }

        void SetAuxiliaryIsUsed(bool value) {
            auxiliary_data_.is_used = value;
        }

        bool GetAuxiliaryIsUsed() {
            return auxiliary_data_.is_used;
        }

        bool GetAuxiliaryState() {
            if (ros::Time::now() - auxiliary_data_.last_auxiliary_time < ros::Duration(0.3)) {
                return true;
            }
            return false;
        }

        bool SetGimbalMode(const GimbalMode &gimbal_mode) {
            if (gimbal_mode == gimbal_mode_) {
                return true;
            }
            leonard_decision::GimbalMode gimbal_mode_msg;
            gimbal_mode_msg.request.gimbal_mode = static_cast<uint8_t >(gimbal_mode);
            gimbal_mode_client_.call(gimbal_mode_msg);
            gimbal_mode_ = gimbal_mode;
            //ROS_INFO("Set gimbal mode to %d", static_cast<int>(gimbal_mode));
            return true;
        }

        GimbalMode GetGimbalMode() {
            return gimbal_mode_;
        }

        bool SetChassisMode(const ChassisMode &chassis_mode) {
            if (chassis_mode == chassis_mode_) {
                return true;
            }
            leonard_decision::ChassisMode chassis_mode_msg;
            chassis_mode_msg.request.chassis_mode = static_cast<uint8_t >(chassis_mode);
            chassis_mode_client_.call(chassis_mode_msg);
            chassis_mode_ = chassis_mode;
            //ROS_INFO("Set chassis mode to %d", static_cast<int>(chassis_mode));
            return true;
        }


        void PublishGoalTaskByGoal(leonard_decision::GoalTask &msg) {
            goal_task_msg_.goal = msg.goal;
            goal_task_msg_.is_goal_set = true;
            goal_task_pub_.publish(goal_task_msg_);
        }


        std::vector<Vec2d> GetEnemyBuffRadiuPointArr(double r = 1.2) {
            std::vector<Vec2d> array;
            for (double i = 0; i < 360; i = i + 5) {
                if (forbidden_data_.buff_center[1](1) + r * sin(i * M_PI / 180) <
                    forbidden_data_.buff_point[1](3) - 0.25) {
                    array.push_back(
                            Vec2d(forbidden_data_.buff_center[1](0) + r * std::cos(i * M_PI / 180),
                                  forbidden_data_.buff_center[1](1) + r * sin(i * M_PI / 180)));
                }
            }
            return array;
        }

        std::vector<Vec2d> GetEnemySupplierRadiuPointArr(double r = 1.3) {
            std::vector<Vec2d> array;
            for (double i = 0; i < 360; i = i + 5) {
                if ((forbidden_data_.supplier_center[1](1) + r * sin(i * M_PI / 180) >
                     forbidden_data_.supplier_point[1](2) + 0.25)
                    && (forbidden_data_.supplier_center[1](0) + r * cos(i * M_PI / 180) >
                        forbidden_data_.supplier_point[1](0) + 0.25)) {
                    array.push_back(
                            Vec2d(forbidden_data_.supplier_center[1](0) + r * std::cos(i * M_PI / 180),
                                  forbidden_data_.supplier_center[1](1) + r * sin(i * M_PI / 180)));
                }
            }
            return array;
        }

        std::vector<Vec2d> GetOurBuffRadiuPointArr(double r = 1.2) {
            std::vector<Vec2d> array;
            for (double i = 0; i < 360; i = i + 5) {
                if (forbidden_data_.buff_center[0](1) + r * sin(i * M_PI / 180) >
                    forbidden_data_.buff_point[0](2) + 0.25) {
                    array.push_back(
                            Vec2d(forbidden_data_.buff_center[0](0) + r * std::cos(i * M_PI / 180),
                                  forbidden_data_.buff_center[0](1) + r * sin(i * M_PI / 180)));
                }
            }
            return array;
        }

        std::vector<Vec2d> GetOurSupplierRadiuPointArr(double r = 1.15) {
            std::vector<Vec2d> array;

            bool jam_angle = false;
            float a[4];
            float min = 400;
            float max = 0;
            if (another_data_.accessible &&
                (EularDistance(another_data_.pose_x, my_pose_x_, another_data_.pose_y, my_pose_y_) <= 2.5) &&
                ((another_data_.pose_x + 0.5) < my_pose_x_)
                    ) {

                a[0] = atan2(another_data_.pose_y + 1 - my_pose_y_, another_data_.pose_x - my_pose_x_ - 0.5);
                a[1] = atan2(another_data_.pose_y - 1 - my_pose_y_, another_data_.pose_x - my_pose_x_ + 0.5);
                a[2] = atan2(another_data_.pose_y + 1 - my_pose_y_, another_data_.pose_x - my_pose_x_ - 0.5);
                a[3] = atan2(another_data_.pose_y - 1 - my_pose_y_, another_data_.pose_x - my_pose_x_ + 0.5);

                for (int t = 0; t < 4; t++) {
                    if (a[t] < 0)
                        a[t] = (M_PI - fabs(a[t])) / M_PI * 180 + 180;
                    else
                        a[t] = a[t] / M_PI * 180;
                }

                for (int t = 0; t < 4; t++) {
                    if (min > a[t]) {
                        min = a[t];
                    }
                    if (max < a[t]) {
                        max = a[t];
                    }
                }

                std::cout << "jam_angle:" << min << "," << max << std::endl;
                std::cout << "jam_angle:" << my_pose_x_ << "," << my_pose_y_ << std::endl;
                std::cout << "jam_angle:" << another_data_.pose_x << "," << another_data_.pose_y << std::endl;

                jam_angle = true;
            }


            for (double i = 0; i < 360; i = i + 5) {
                if ((forbidden_data_.supplier_center[0](1) + r * sin(i * M_PI / 180) <
                     forbidden_data_.supplier_point[0](3) - 0.25)
                    && (forbidden_data_.supplier_center[0](0) + r * cos(i * M_PI / 180) <
                        forbidden_data_.supplier_point[0](1) - 0.25)) {
                    if (jam_angle && i < max && i > min) {
                        continue;
                    }
                    array.push_back(
                            Vec2d(forbidden_data_.supplier_center[0](0) + r * std::cos(i * M_PI / 180),
                                  forbidden_data_.supplier_center[0](1) + r * sin(i * M_PI / 180)));
                }
            }
            return array;
        }


        bool wait_flag_ = false;

    private:
        bool enemy_detected_ = false;
        geometry_msgs::PoseStamped enemy_pose_;
        ros::Time last_enemy_detected_time_;


        GimbalMode gimbal_mode_;
        ChassisMode chassis_mode_;
        ros::ServiceClient gimbal_mode_client_;
        ros::ServiceClient chassis_mode_client_;
        //! Enenmy detection
        ros::Subscriber enemy_sub_;
        ros::Subscriber color_sub_;

        // Color:
        bool color_detected_ = false;
        ros::Time last_color_detected_time_;

        // master-wing:
        bool is_master_;
        ros::Subscriber goal_task_sub_;

        class AuxiliaryData {
        public:
            geometry_msgs::PoseStamped auxiliary_position;
            bool is_used;
            ros::Time last_auxiliary_time;
        };

        AuxiliaryData auxiliary_data_;


        // ArmorAttacked:
        ArmorAttacked armor_attacked_ = ArmorAttacked::NONE;
        ros::Time last_armor_attacked_time_;

        //! Referee system info
        uint16_t remain_hp_ = 2000;
        uint8_t robot_id_ = 3;

        // Game Info
        uint8_t game_status_;
        uint16_t remain_time_;


        // Bonus Info
        BuffData buff_data_;

        //My Pose Info
        double my_pose_x_;
        double my_pose_y_;
        double my_pose_yaw_;

        // another Info
        AnotherRobotData another_data_;
        leonard_decision::GoalTask goal_task_msg_;

        //Forbidden Info
        ForbiddenData forbidden_data_;

        //Supplier Info
        SupplierData supplier_data_;

        // bullet_num;
        int16_t true_bullet_ = 0;

        int16_t remain_bullet_ = 0;//master 50 wing 0
        int16_t bullet_msgs_ = 0;
        int16_t last_bullet_ = 0;
        ros::Time bullet_zero_time_;

        /**** 用于检测机器人是否卡死*****/
        class JammingJudge {
        public:
            JammingJudge() {
                pose_vec.setZero();
            };

            ~JammingJudge() = default;

            //记录规划action状态
            bool goal_planner_state = false;
            //查看机器人是否卡死
            bool is_jamming = false;
            int jam_cnt = 0;
            Vec3d pose_vec;
            ros::Time jam_time;
        };

        JammingJudge jamming_judge_;
        ros::Subscriber planner_state_sub_;


        //! Referee system subscriber
        ros::Subscriber robot_hurt_sub_;
        ros::Subscriber robot_info_sub_;
        ros::Subscriber game_info_sub_;
        ros::Subscriber bonus_info_sub_;
        ros::Subscriber supplier_info_sub_;
        ros::Subscriber bullet_info_sub_;
        ros::Subscriber game_survivor_sub_;
        ros::Subscriber robot_bonus_sub_;
        ros::Subscriber robot_pose_sub_;
        ros::Subscriber master_pose_sub_;
        ros::Subscriber wing_pose_sub_;
        ros::Timer timer_;
        ros::Publisher goal_task_pub_;

    private:

        inline double EularDistance(double x1, double x2, double y1, double y2) {
            return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
        }

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

    };


} //namespace decision
#endif
