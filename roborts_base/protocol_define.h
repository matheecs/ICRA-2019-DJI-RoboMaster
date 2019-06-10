//
// Created by cxn on 19-2-20.
//

#ifndef PROJECT_PROTOCOL_DEFINE_H
#define PROJECT_PROTOCOL_DEFINE_H

namespace leonard_serial_common {

#pragma pack(push, 1)

#define SOF 0xAA
#define DEVICE 0x00
#define HEADER_LEN sizeof(FrameHeader)
#define CMD_LEN 2
#define CRC_HEAD_LEN 2
#define CRC_DATA_LEN 4
#define PROTOCAL_FRAME_MAX_SIZE 1024
#define UART_BUFF_SIZE 1024
#define VERSION 0x00

#define PC_ADDRESS (0x00u)
#define CHASSIS_ADDRESS (0X01u)
#define GIMBAL_ADDRESS (0X02u)
#define REFEREE_SEND_CMD_SET (0x01u)
#define CHASSIS_CMD_SET (0x02u)
#define GIMBAL_CMD_SET (0x03u)

#define COMPATIBLE_CMD_SET (0x04u)

#define REFEREE_GAME_CMD_SET (0x40u)
#define REFEREE_BATTLEFIELD_CMD_SET (0x41u)
#define REFEREE_ROBOT_CMD_SET (0x42u)
#define REFEREE_RECEIVE_CMD_SET (0x43u)

#define PACK_MAX_SIZE 200
#define COMPRESS_TIME 1

/*----------------------------UNIVERSAL_CMD--- 0x00 ---------------------*/
#define CMD_HEARTBEAT (0x01u)
    typedef struct {
        uint32_t heartbeat;
    } cmd_heartbeat;

#define CMD_REPORT_VERSION (0X02u)
    typedef struct {
        uint32_t version_id;
    } cmd_version_id;

/*-----------------------------REFEREE_SEND_CMD--- 0x01 ---------------------*/
#define CMD_REFEREE_SEND_DATA (0X01u)

    typedef struct {
        uint16_t cmd = 0x0103u;
        uint8_t supply_projectile_id;
        uint8_t supply_robot_id;
        uint8_t supply_num;
    } cmd_supply_projectile_booking;


    typedef struct {
        uint16_t cmd = 0x0301u;
        uint16_t id = 0x0200u;
        uint16_t sendid;
        uint16_t recid;
        float goalx;
        float goaly;
        float goalyaw;
        uint8_t is_goal_set;
        uint16_t remain_hp;
        uint8_t supplying;
        uint8_t supplycnt;
        uint8_t bonusing;
        float pose_x;
        float pose_y;
        uint8_t supply_warn;
        uint8_t lack_bullet;
    } cmd_our_booking;

/**
  * @brief  frame header structure definition
  */
    typedef struct {
        uint8_t sof;
        union {
            struct {
                uint16_t data_len : 10;
                uint16_t version : 6;
            };
            uint16_t ver_data_len;
        };
        union {
            struct {
                uint8_t session : 5;   /*!< Need(0~1) Or Not Need(2~63) Ack */
                uint8_t pack_type : 1; /*!< Ack Package Or Normal Package */
                uint8_t res : 2;       /*!< Reserve */
            };
            uint8_t S_A_R_c;
        };
        uint8_t sender;   /*!< Sender Module Information */
        uint8_t receiver; /*!< Receiver Module Information */
        uint16_t res1;    /*!< Reserve 1 */
        uint16_t seq_num; /*!< Sequence Number */
        uint16_t crc_16;  /*!< CRC16 */
    } __attribute__((packed)) FrameHeader;

#define CMD_PUSH_CHASSIS_INFO (0X01u)
    typedef struct {
        int16_t gyro_angle;
        int16_t gyro_rate;
        int32_t position_x_mm;
        int32_t position_y_mm;
        int16_t angle_deg;
        int16_t v_x_mm;
        int16_t v_y_mm;
        /*******leonard add******/
        int16_t bullet; //子弹数量
    } cmd_chassis_info;

#define CMD_SET_CHASSIS_MODE (0X02u)
    typedef uint8_t chassis_mode_e;

#define CMD_SET_CHASSIS_SPEED (0X03u)
    typedef struct {
        int16_t vx;
        int16_t vy;
        int16_t vw;
        int16_t rotate_x_offset;
        int16_t rotate_y_offset;
        //        uint8_t mode;//0普通,1扭屁股
    } cmd_chassis_speed;

#define CMD_GET_CHASSIS_PARAM (0X04u)
    typedef struct {
        uint16_t wheel_perimeter;
        uint16_t wheel_track;
        uint16_t wheel_base;
        int16_t gimbal_x_offset;
        int16_t gimbal_y_offset;
    } cmd_chassis_param;

#define CMD_SET_CHASSIS_SPD_ACC (0X05u)
    typedef struct {
        int16_t vx;
        int16_t vy;
        int16_t vw;
        int16_t ax;
        int16_t ay;
        int16_t wz;
        int16_t rotate_x_offset;
        int16_t rotate_y_offset;
    } cmd_chassis_spd_acc;

#define CMD_SUPPLY_CMD (0X06u)
    typedef struct {
        uint8_t cmd;
    } cmd_supply_cmd;

/*-----------------------------GIMBAL_CMD---- 0x03 ---------------------*/

#define CMD_PUSH_GIMBAL_INFO (0X01u)
    typedef struct {
        uint8_t mode;
        int16_t pitch_ecd_angle;
        int16_t yaw_ecd_angle;
        int16_t pitch_gyro_angle;
        int16_t yaw_gyro_angle;
        int16_t yaw_rate;
        int16_t pitch_rate;
    } cmd_gimbal_info;

#define CMD_SET_GIMBAL_MODE (0X02u)
//    typedef enum {
//        GYRO_CONTROL,
//        CODE_CONTROL,
//        G_MODE_MAX_NUM,
//    } gimbal_mode_e;
// leoard add

    typedef uint8_t gimbal_mode_e;

#define CMD_SET_GIMBAL_ANGLE (0x03u)
    typedef struct {
        union {
            uint8_t flag;
            struct {
                uint8_t yaw_mode : 1; //0 means absolute, 1 means relative;
                uint8_t pitch_mode : 1;
            } bit;
        } ctrl;
        int16_t pitch;
        int16_t yaw;
        int16_t distance;
        int32_t yaw_spd;
        int32_t pitch_spd;
        int64_t time;
        //        uint8_t mode;//0巡逻,1瞄准
    } cmd_gimbal_angle;

#define CMD_SET_FRIC_WHEEL_SPEED (0X04u)
    typedef struct {
        uint16_t left;
        uint16_t right;
    } cmd_fric_wheel_speed;

#define CMD_SET_SHOOT_INFO (0x05u)
    typedef enum {
        SHOOT_STOP = 0,
        SHOOT_ONCE,
        SHOOT_CONTINUOUS,
    } shoot_cmd_e;

    typedef struct {
        uint8_t shoot_cmd;
        uint32_t shoot_add_num;
        uint16_t shoot_freq;
    } cmd_shoot_info;

/*------------------------COMPATIBLE_CMD---- 0x04 -------------------*/
#define CMD_RC_DATA_FORWARD (0X01u)

#define CMD_PUSH_UWB_INFO (0X02u)
    typedef struct {
        int16_t x;
        int16_t y;
        uint16_t yaw;
        int16_t distance[6];
        uint16_t error;
        uint16_t res;
    } cmd_uwb_info;

/*------------------------REFEREE_GAME_CMD---- 0x40 -------------------*/
#define CMD_GAME_STATUS (0X01u)
//发送频率:1Hz
    typedef struct {
        uint8_t game_type : 4;      //3 ICRA赛
        uint8_t game_progress : 4;  //0:未开始 1:准备阶段 2:自检阶段 3:5s倒计时 4:对战中(共3min) 5:比赛结算中
        uint16_t stage_remain_time; //当前阶段剩余时间s
    } cmd_game_state;

#define CMD_GAME_RESULT (0X02u)
    typedef struct {
        uint8_t winner;
    } cmd_game_result;

#define CMD_GAME_SURVIVAL (0X03u)
//发送频率:1Hz
    typedef struct {
        uint16_t robot_legion; //bit2:红3 bit3:红4 bit10:蓝3 bit11:蓝4 1表示存活,0表示死亡
    } cmd_game_robot_survivors;

/*-------------------REFEREE_BATTLEFIELD_CMD_SET---- 0x41 -------------*/
#define CMD_BATTLEFIELD_EVENT (0X01u)
//发送频率:1Hz
    typedef struct {
        uint32_t event_type; //12-13红方 14-15蓝方 0:未激活 1:5s触发 2:已激活
    } cmd_event_data;

#define CMD_SUPPLIER_ACTION (0X02u)
//补给站动作改变时
    typedef struct {
        uint8_t supply_projectile_id;   //补给站口ID
        uint8_t supply_robot_id;        //补给车ID 3/4红 13/14蓝
        uint8_t supply_projectile_step; //0关 1准备 2下落(50发3s左右,1min最多2次)
        uint8_t supply_projectile_num;  //50 100 150 200
    } cmd_supply_projectile_action;

/*------------------------REFEREE_ROBOT_CMD---- 0x42 -------------------*/
#define CMD_ROBOT_STATUS (0X01u)
//发送频率:10Hz
    typedef struct {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t remain_HP;                   //剩余血量
        uint16_t max_HP;                      //max血量
        uint16_t shooter_heat0_cooling_rate;  //17mm枪口每s冷却值
        uint16_t shooter_heat0_cooling_limit; //17mm枪口热量上限
        uint16_t shooter_heat1_cooling_rate;
        uint16_t shooter_heat1_cooling_limit;
        uint8_t mains_power_gimbal_output : 1;  //0:无24V输出 1:有24V输出
        uint8_t mains_power_chassis_output : 1; //0:无24V输出 1:有24V输出
        uint8_t mains_power_shooter_output : 1; //0:无24V输出 1:有24V输出
    } cmd_game_robot_state;

#define CMD_ROBOT_POWER_HEAT (0X02u)
    typedef struct {
        uint16_t chassis_volt;
        uint16_t chassis_current;
        float chassis_power;
        uint16_t chassis_power_buffer;
        uint16_t shooter_heat0;
        uint16_t shooter_heat1;
    } cmd_power_heat_data;

#define CMD_ROBOT_POSITION (0X03u)
    typedef struct {
        float x;
        float y;
        float z;
        float yaw;
    } cmd_game_robot_pos;

#define CMD_ROBOT_BUFF (0X04u)
    typedef struct {
        uint8_t power_rune_buff;
    } cmd_buff_musk;

#define CMD_AERIAL_ENERGY (0X05u)
    typedef struct {
        uint8_t energy_point;
        uint8_t attack_time;
    } cmd_aerial_robot_energy;

#define CMD_ROBOT_HURT (0X06u)
    typedef struct {
        uint8_t armor_id : 4;
        uint8_t hurt_type : 4;
    } cmd_robot_hurt;

#define CMD_ROBOT_SHOOT (0X07u)
    typedef struct {
        uint8_t bullet_type;
        uint8_t bullet_freq;
        float bullet_speed;
    } cmd_shoot_data;

#pragma pack(pop)
} // namespace leonard_serial_common

#endif //PROJECT_PROTOCOL_DEFINE_H
