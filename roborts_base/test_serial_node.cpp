//
// Created by cxn on 19-4-20.
//

#include "test_serial_node.h"

typedef struct {
    uint8_t red_bonus_event:2;
    uint8_t blue_bonus_event:2;
} cmd_bonus_event;

typedef struct {
    uint8_t red3:1;
    uint8_t red4:1;
    uint8_t blue3:1;
    uint8_t blue4:1;
} cmd_game_robot_survivors;


bool TestRefreeService(roborts_base::TestRefree::Request &req,
                       roborts_base::TestRefree::Response &res) {
//    gimbal_mode_e gimbal_mode = static_cast<gimbal_mode_e>(req.gimbal_mode);
//
//    if (!SendData((uint8_t *) &gimbal_mode, sizeof(gimbal_mode_e), GIMBAL_CMD_SET, CMD_SET_GIMBAL_MODE,
//                  GIMBAL_ADDRESS)) {
//        LOG_WARNING << "Overflow in Gimbal Mode";
//    }
//    gimbalmode_ = gimbal_mode;
    res.received = true;
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "test_serial_node");
    ros::NodeHandle nh_;
    ros::Rate loop_rate(10);
    int64_t count = 0;

    ros::Publisher game_status_pub_ = nh_.advertise<roborts_msgs::GameStatus>("game_status", 30);
    ros::Publisher game_survival_pub_ = nh_.advertise<roborts_msgs::GameSurvivor>("game_survivor", 30);
    ros::Publisher bonus_status_pub_ = nh_.advertise<roborts_msgs::BonusStatus>("field_bonus_status", 30);
    ros::Publisher supplier_status_pub_ = nh_.advertise<roborts_msgs::SupplierStatus>("field_supplier_status", 30);
    ros::Publisher robot_bonus_pub_ = nh_.advertise<roborts_msgs::RobotBonus>("robot_bonus", 30);

    leonard_serial_common::cmd_game_state s_game_status{.game_type=3, .game_progress=3, .stage_remain_time=1};
    cmd_bonus_event s_bonus_event{.red_bonus_event=0, .blue_bonus_event=0};
    leonard_serial_common::cmd_supply_projectile_action s_supply_projectile_action{.supply_projectile_id=1,
            .supply_robot_id=1, .supply_projectile_step=0, .supply_projectile_num=50};
    cmd_game_robot_survivors s_game_robot_survivors{.red3=1, .red4=1, .blue3=1, .blue4=1};
    roborts_msgs::RobotBonus robotBonus;
    robotBonus.bonus = true;

    ros::ServiceServer test_refree_srv_ = nh_.advertiseService("test_refree", &TestRefreeService);

    while (ros::ok()) {
        if (count % 10 == 0) {
            roborts_msgs::GameStatus game_status;
            game_status.game_status = s_game_status.game_progress;
            game_status.remaining_time = s_game_status.stage_remain_time;
            game_status_pub_.publish(game_status);

            roborts_msgs::BonusStatus bonus_status;
            bonus_status.red_bonus = s_bonus_event.red_bonus_event;
            bonus_status.blue_bonus = s_bonus_event.blue_bonus_event;
            bonus_status_pub_.publish(bonus_status);

            roborts_msgs::SupplierStatus supplier_status;
            supplier_status.supply_projectile_id = s_supply_projectile_action.supply_projectile_id;
            supplier_status.supply_robot_id = s_supply_projectile_action.supply_robot_id;
            supplier_status.supply_projectile_step = s_supply_projectile_action.supply_projectile_step;
            supplier_status.supply_projectile_num = s_supply_projectile_action.supply_projectile_num;
            supplier_status_pub_.publish(supplier_status);

            robot_bonus_pub_.publish(robotBonus);

//            roborts_msgs::GameSurvivor game_survivor;
//            game_survivor.red3 = s_game_robot_survivors.red3;
//            game_survivor.red4 = s_game_robot_survivors.red4;
//            game_survivor.blue3 = s_game_robot_survivors.blue3;
//            game_survivor.blue4 = s_game_robot_survivors.blue4;
//            game_survival_pub_.publish(game_survivor);
        }
        count++;
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}