//
// Created by cxn on 19-4-20.
//

#ifndef PROJECT_TEST_SERIAL_NODE_H
#define PROJECT_TEST_SERIAL_NODE_H

#include <ros/ros.h>
#include <ros/timer.h>
#include <geometry_msgs/PoseStamped.h>
#include "protocol_define.h"
#include "roborts_msgs/ProjectileSupply.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/GameSurvivor.h"
#include "roborts_msgs/BonusStatus.h"
#include "roborts_msgs/SupplierStatus.h"
#include "roborts_msgs/RobotBonus.h"
#include "roborts_base/TestRefree.h"
#include "roborts_msgs/ProjectileSupply.h"
#include "roborts_msgs/RobotStatus.h"

namespace leonard_serial_common {
    class BonusEvent {
    public:
        uint8_t event[2]; //3号车,4号车
        uint16_t cnt[2];
        uint8_t parallel_event[2];
        uint16_t parallel_cnt[2];
        bool ask[2];
        bool allow;
        bool quit[2];

        void Reset() {
            event[0] = cnt[0] = parallel_event[0] = parallel_cnt[0] = event[1] = cnt[1] = parallel_event[1] = parallel_cnt[1] = 0;
            ask[0] = ask[1] = allow = quit[0] = quit[1] = false;
        }
    };

    class SupplyStatus {
    public:
        uint8_t step;
        uint8_t supply_id;
        bool suppling;
        uint16_t cnt;
        bool ask;
        uint8_t ask_id;
        uint8_t ask_num;
        uint8_t allow_cnt;

        void Reset() {
            step = supply_id = cnt = ask_id = allow_cnt = 0;
            suppling = ask = false;
        }
    };


    class ForbiddenEvent {
    public:
        uint8_t is_died;//0 ,1正在死 ,2死了
        uint16_t cnt;
        bool ask;
        bool quit;

        void Reset() {
            cnt = is_died = 0;
            ask = quit = false;
        }
    };


    typedef struct {
        uint8_t red3:1;
        uint8_t red4:1;
        uint8_t blue3:1;
        uint8_t blue4:1;
    } my_game_robot_survivors;


    class TestRefreeNode {
    public:
        TestRefreeNode(std::string name);

        ~TestRefreeNode() = default;

    private:
        bool TestRefreeService(roborts_base::TestRefree::Request &req,
                               roborts_base::TestRefree::Response &res);

        void TimerCallback(const ros::TimerEvent &);

        void Reset();

        uint16_t GetGameTime();

        void SupplierPublish(uint8_t robot_id, uint8_t step,uint8_t num);

        void RobotBonusPublish(bool bonus);

        void ProjectileSupplyCallbackMaster(const roborts_msgs::ProjectileSupply::ConstPtr projectile_supply);

        void ProjectileSupplyCallbackWing(const roborts_msgs::ProjectileSupply::ConstPtr projectile_supply);

        void PoseCallbackMaster(const geometry_msgs::PoseStamped::ConstPtr msg);

        void PoseCallbackWing(const geometry_msgs::PoseStamped::ConstPtr msg);

        void PoseCallbackCmd(uint8_t robot_id, const geometry_msgs::PoseStamped::ConstPtr msg);

        void RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &msg);

    private:
        ros::NodeHandle nh_;

        ros::Publisher game_status_pub_1_;
        ros::Publisher bonus_status_pub_1_;
        ros::Publisher supplier_status_pub_1_;
        ros::Publisher robot_bonus_pub_1_;
        ros::Publisher game_status_pub_2_;
        ros::Publisher bonus_status_pub_2_;
        ros::Publisher supplier_status_pub_2_;
        ros::Publisher robot_bonus_pub_2_;

        leonard_serial_common::cmd_game_state s_game_status_{.game_type=3, .game_progress=3, .stage_remain_time=1};
        BonusEvent bonus_array_[2];//红色,蓝色
        SupplyStatus supply_array_[2];//红色,蓝色

        ros::ServiceServer test_refree_srv_;
        ros::Timer timer_;


        ros::Subscriber sub_projectile_supply_1_;
        ros::Subscriber sub_projectile_supply_2_;
        ros::Subscriber sub_pose_1_;
        ros::Subscriber sub_pose_2_;
        ros::Subscriber sub_robot_status_1_;

        uint8_t color = 0;//默认红车0,蓝车1

        float buff_point_[2][4] = {{5.8, 6.8, 1.25, 2.25},//color的本方buff
                                   {1.2, 2.2, 2.75, 3.75}};//color敌人的buff
        bool occupy_bonus_[2][2] = {
                {false, false},
                {false, false}
        };//主/从 机器是否在 本方/敌方的buff
        bool last_occupy_bonus_[2][2] = {{false, false},
                                         {false, false}};

        float forbidden_point_[2][4] = {{3.5, 4.5, 4, 5},//color的本方forbidden(相对与敌人的禁区)
                                        {3.5, 4.5, 0, 1}};//color敌人的forbidden
        bool occupy_forbidden_[2];//主/从 机器是否在 敌方的forbidden
        bool last_forbidden_bonus_[2];
        ForbiddenEvent forbidden_array_[2];//分别给两个台车用


        ros::Publisher game_survival_pub_1_;
        ros::Publisher game_survival_pub_2_;
        my_game_robot_survivors s_game_robot_survivors_{.red3=1, .red4=1, .blue3=1, .blue4=1};
    };
}
#endif //PROJECT_TEST_SERIAL_NODE_H
