//
// Created by cxn on 19-2-19.
//
#include "serial_comm_node.h"

namespace leonard_serial_common {
    SerialComNode::SerialComNode(std::string name) {
        CHECK(Init()) << "Module " << name << " initialized failed!";
    }

    bool SerialComNode::Init() {
        int baudrate = 921600;
        std::string serial_port;
        nh_.param<std::string>("roborts_base/serial_port", serial_port, "/dev/serial_sdk");
        hardware_device_ = std::make_shared<SerialDevice>(serial_port, baudrate);

        if (!hardware_device_->Init()) {
            LOG_ERROR << "Can not open device. ";
            return false;
        }

        is_open_ = true;
        stop_receive_ = false;
        stop_send_ = false;
        total_length_ = 0;
        free_length_ = UART_BUFF_SIZE;

        odom_.header.frame_id = "odom";
        odom_.child_frame_id = "base_link";
        odom_tf_.header.frame_id = "odom";
        odom_tf_.child_frame_id = "base_link";

        //ros_message_init
        gimbal_tf_.header.frame_id = "base_link";
        gimbal_tf_.child_frame_id = "gimbal";

        receive_loop_thread_ = new std::thread(boost::bind(&SerialComNode::ReceiveLoop, this));
        send_loop_thread_ = new std::thread(boost::bind(&SerialComNode::SendPack, this));

        //ros service
        ctrl_fric_wheel_srv_ = nh_.advertiseService("cmd_fric_wheel", &SerialComNode::CtrlFricWheelService, this);
        ctrl_shoot_srv_ = nh_.advertiseService("cmd_shoot", &SerialComNode::CtrlShootService, this);
        chassis_mode_srv_ = nh_.advertiseService("set_chassis_mode", &SerialComNode::SetChassisModeService, this);
        gimbal_mode_srv_ = nh_.advertiseService("set_gimbal_mode", &SerialComNode::SetGimbalModeService, this);

        high_speed_client_ = nh_.serviceClient<roborts_msgs::ForceUpdateAmcl>("set_force_update_mode");

        turnangle_ptr_ = std::make_shared<TurnAngleAction>("turnangle", &base_yaw_);

        //ros publisher
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 30);
        game_status_pub_ = nh_.advertise<roborts_msgs::GameStatus>("game_status", 30);
        game_result_pub_ = nh_.advertise<roborts_msgs::GameResult>("game_result", 30);
        game_survival_pub_ = nh_.advertise<roborts_msgs::GameSurvivor>("game_survivor", 30);
        bonus_status_pub_ = nh_.advertise<roborts_msgs::BonusStatus>("field_bonus_status", 30);
        supplier_status_pub_ = nh_.advertise<roborts_msgs::SupplierStatus>("field_supplier_status", 30);
        robot_status_pub_ = nh_.advertise<roborts_msgs::RobotStatus>("robot_status", 30);
        robot_heat_pub_ = nh_.advertise<roborts_msgs::RobotHeat>("robot_heat", 2);
        robot_bonus_pub_ = nh_.advertise<roborts_msgs::RobotBonus>("robot_bonus", 30);
        robot_damage_pub_ = nh_.advertise<roborts_msgs::RobotDamage>("robot_damage", 30);
        robot_shoot_pub_ = nh_.advertise<roborts_msgs::RobotShoot>("robot_shoot", 2);
        bullet_pub_ = nh_.advertise<roborts_base::Bullet>("robot_bullet", 30);

        //ros subscriber
        sub_projectile_supply_ = nh_.subscribe("projectile_supply", 1, &SerialComNode::ProjectileSupplyCallback,
                                               this);

        sub_our_booking_ = nh_.subscribe("our_booking", 1, &SerialComNode::OurBookingCallback,
                                         this);

        //订阅云台目标、底盘速度、射击指令
        sub_cmd_gim_ = nh_.subscribe("cmd_gimbal_angle", 1, &SerialComNode::GimbalAngleCtrlCallback, this);
        sub_cmd_vel_ = nh_.subscribe("cmd_vel", 1, &SerialComNode::ChassisSpeedCtrlCallback, this);
        sub_cmd_vel_acc_ = nh_.subscribe("cmd_vel_acc", 1, &SerialComNode::ChassisSpeedAccCtrlCallback, this);

        //        std::vector<std::string> v = {"x", "y", "angle", "x_sp", "y_sp", "angle_sp"};
        //        csvWriter_ = std::make_unique<CsvWriter>("/home/cxn/data.csv", v);

        return true;
    }

    SerialComNode::~SerialComNode() {
        if (receive_loop_thread_ != nullptr) {
            stop_receive_ = true;
            receive_loop_thread_->join();
            delete receive_loop_thread_;
        }
        if (send_loop_thread_ != nullptr) {
            stop_send_ = true;
            send_loop_thread_->join();
            delete send_loop_thread_;
        }
        is_open_ = false;
    }

/*************************** Call back ****************************/
    void SerialComNode::ChassisSpeedCtrlCallback(const geometry_msgs::Twist::ConstPtr &vel) {
        cmd_chassis_speed chassis_speed;
        chassis_speed.vx = vel->linear.x * 1000;
        chassis_speed.vy = vel->linear.y * 1000;
        chassis_speed.vw = vel->angular.z * 1800.0 / M_PI;
        chassis_speed.rotate_x_offset = 0;
        chassis_speed.rotate_y_offset = 0;
        if (!SendData((uint8_t *) &chassis_speed, sizeof(cmd_chassis_speed), CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPEED,
                      CHASSIS_ADDRESS)) {
            LOG_WARNING << "Overflow in Chassis CB";
        }
    }

    void SerialComNode::ChassisSpeedAccCtrlCallback(const roborts_msgs::TwistAccel::ConstPtr &vel_acc) {
        cmd_chassis_spd_acc chassis_spd_acc;
        chassis_spd_acc.vx = vel_acc->twist.linear.x * 1000;
        chassis_spd_acc.vy = vel_acc->twist.linear.y * 1000;
        chassis_spd_acc.vw = vel_acc->twist.angular.z * 1800.0 / M_PI;
        chassis_spd_acc.ax = vel_acc->accel.linear.x * 1000;
        chassis_spd_acc.ay = vel_acc->accel.linear.y * 1000;
        chassis_spd_acc.wz = vel_acc->accel.angular.z * 1800.0 / M_PI;
        chassis_spd_acc.rotate_x_offset = 0;

        if (!SendData((uint8_t *) &chassis_spd_acc, sizeof(cmd_chassis_spd_acc), CHASSIS_CMD_SET,
                      CMD_SET_CHASSIS_SPD_ACC,
                      CHASSIS_ADDRESS)) {
            LOG_WARNING << "Overflow in Chassis ACC";
        }
    }

    void SerialComNode::GimbalAngleCtrlCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg) {
        cmd_gimbal_angle gimbal_angle;
        gimbal_angle.ctrl.bit.pitch_mode = msg->pitch_mode;
        gimbal_angle.ctrl.bit.yaw_mode = msg->yaw_mode;
        gimbal_angle.pitch = msg->pitch_angle * 18000 / M_PI;
        gimbal_angle.yaw = msg->yaw_angle * 18000 / M_PI;
        gimbal_angle.distance = msg->distance * 100;
        gimbal_angle.yaw_spd = msg->yaw_spd * 100;
        gimbal_angle.pitch_spd = msg->pitch_spd * 100;
        gimbal_angle.time = msg->time;
        if (!SendData((uint8_t *) &gimbal_angle, sizeof(cmd_gimbal_angle), GIMBAL_CMD_SET, CMD_SET_GIMBAL_ANGLE,
                      GIMBAL_ADDRESS)) {
            LOG_WARNING << "Overflow in Gimbal CB";
        }
    }

    bool SerialComNode::SetGimbalModeService(roborts_msgs::GimbalMode::Request &req,
                                             roborts_msgs::GimbalMode::Response &res) {
        gimbal_mode_e gimbal_mode = static_cast<gimbal_mode_e>(req.gimbal_mode);

        if (!SendData((uint8_t *) &gimbal_mode, sizeof(gimbal_mode_e), GIMBAL_CMD_SET, CMD_SET_GIMBAL_MODE,
                      GIMBAL_ADDRESS)) {
            LOG_WARNING << "Overflow in Gimbal Mode";
        }
        gimbalmode_ = gimbal_mode;
        res.received = true;
        return true;
    }

    bool SerialComNode::SetChassisModeService(roborts_msgs::ChassisMode::Request &req,
                                              roborts_msgs::ChassisMode::Response &res) {

        chassis_mode_e chassis_mode = static_cast<gimbal_mode_e>(req.chassis_mode);

        if (!SendData((uint8_t *) &chassis_mode, sizeof(chassis_mode_e), CHASSIS_CMD_SET,
                      CMD_SET_CHASSIS_MODE,
                      CHASSIS_ADDRESS)) {
            LOG_WARNING << "Overflow in Chassis Mode";
        }
        chassismode_ = chassis_mode;
        res.received = true;
        return true;
    }

    bool SerialComNode::CtrlFricWheelService(roborts_msgs::FricWhl::Request &req,
                                             roborts_msgs::FricWhl::Response &res) {
        cmd_fric_wheel_speed fric_speed;
        if (req.open) {
            fric_speed.left = 1200;
            fric_speed.right = 1200;
        } else {
            fric_speed.left = 1000;
            fric_speed.right = 1000;
        }

        if (!SendData((uint8_t *) &fric_speed, sizeof(cmd_fric_wheel_speed), GIMBAL_CMD_SET, CMD_SET_FRIC_WHEEL_SPEED,
                      GIMBAL_ADDRESS)) {
            LOG_WARNING << "Overflow in fricwheel open";
        }

        res.received = true;
        return true;
    }

    bool SerialComNode::CtrlShootService(roborts_msgs::ShootCmd::Request &req,
                                         roborts_msgs::ShootCmd::Response &res) {
        cmd_shoot_info gimbal_shoot;
        uint16_t default_freq = 1500;
        switch (static_cast<shoot_cmd_e>(req.mode)) {
            case SHOOT_STOP:
                gimbal_shoot.shoot_cmd = SHOOT_STOP;
                gimbal_shoot.shoot_add_num = 0;
                gimbal_shoot.shoot_freq = 0;
                break;
            case SHOOT_ONCE:
                if (req.number != 0) {
                    gimbal_shoot.shoot_cmd = SHOOT_ONCE;
                    gimbal_shoot.shoot_add_num = req.number;
                    gimbal_shoot.shoot_freq = default_freq;
                } else {
                    gimbal_shoot.shoot_cmd = SHOOT_ONCE;
                    gimbal_shoot.shoot_add_num = 1;
                    gimbal_shoot.shoot_freq = default_freq;
                }
                break;
            case SHOOT_CONTINUOUS:
                gimbal_shoot.shoot_cmd = SHOOT_CONTINUOUS;
                gimbal_shoot.shoot_add_num = req.number;
                gimbal_shoot.shoot_freq = default_freq;
                break;
            default:
                return false;
        }
        if (!SendData((uint8_t *) &gimbal_shoot, sizeof(cmd_shoot_info), GIMBAL_CMD_SET, CMD_SET_SHOOT_INFO,
                      GIMBAL_ADDRESS)) {
            LOG_WARNING << "Overflow in shoot";
        }
        res.received = true;
        return true;
    }

/*************************** Refreesystem ****************************/
    void SerialComNode::GameStateCallback(leonard_serial_common::cmd_game_state *raw_game_status) {
        roborts_msgs::GameStatus game_status;
        game_status.game_status = raw_game_status->game_progress;
        game_status.remaining_time = raw_game_status->stage_remain_time;
        game_status_pub_.publish(game_status);
    }

    void SerialComNode::GameResultCallback(leonard_serial_common::cmd_game_result *raw_game_result) {
        roborts_msgs::GameResult game_result;
        game_result.result = raw_game_result->winner;
        game_result_pub_.publish(game_result);
    }

    void SerialComNode::GameSurvivorCallback(
            leonard_serial_common::cmd_game_robot_survivors *raw_game_survivor) {
        roborts_msgs::GameSurvivor game_survivor;
        game_survivor.red3 = raw_game_survivor->robot_legion >> 2 & 1;
        game_survivor.red4 = raw_game_survivor->robot_legion >> 3 & 1;
        game_survivor.blue3 = raw_game_survivor->robot_legion >> 10 & 1;
        game_survivor.blue4 = raw_game_survivor->robot_legion >> 11 & 1;
        game_survival_pub_.publish(game_survivor);
    }

    void SerialComNode::GameEventCallback(leonard_serial_common::cmd_event_data *raw_game_event) {
        roborts_msgs::BonusStatus bonus_status;
        bonus_status.red_bonus = raw_game_event->event_type >> 12 & 3;
        bonus_status.blue_bonus = raw_game_event->event_type >> 14 & 3;
        bonus_status_pub_.publish(bonus_status);
    }

    void SerialComNode::SupplierStatusCallback(
            leonard_serial_common::cmd_supply_projectile_action *raw_supplier_status) {

        roborts_msgs::SupplierStatus supplier_status;
        supplier_status.supply_projectile_id = raw_supplier_status->supply_projectile_id;
        supplier_status.supply_robot_id = raw_supplier_status->supply_robot_id;
        supplier_status.supply_projectile_step = raw_supplier_status->supply_projectile_step;
        supplier_status.supply_projectile_num = raw_supplier_status->supply_projectile_num;
        supplier_status_pub_.publish(supplier_status);
    }

    void SerialComNode::RobotStatusCallback(
            leonard_serial_common::cmd_game_robot_state *raw_robot_status) {
        roborts_msgs::RobotStatus robot_status;

        if (robot_id_ == 0xFF || robot_id_ != raw_robot_status->robot_id) {
            robot_id_ = raw_robot_status->robot_id;
        }

        //        switch (raw_robot_status->robot_id) {
        //            case 3:
        //                robot_status.id = 3;
        //                break;
        //            case 4:
        //                robot_status.id = 4;
        //                break;
        //            case 13:
        //                robot_status.id = 13;
        //                break;
        //            case 14:
        //                robot_status.id = 14;
        //                break;
        //            default:
        //                LOG_ERROR
        //                        << "For AI challenge, please set robot id to Blue3/4 or Red3/4 in the referee system main control module";
        //                return;
        //        }

        robot_status.id = robot_id_;
        robot_status.level = raw_robot_status->robot_level;
        robot_status.remain_hp = raw_robot_status->remain_HP;
        robot_status.max_hp = raw_robot_status->max_HP;
        robot_status.heat_cooling_limit = raw_robot_status->shooter_heat0_cooling_limit;
        robot_status.heat_cooling_rate = raw_robot_status->shooter_heat0_cooling_rate;
        robot_status.chassis_output = raw_robot_status->mains_power_chassis_output;
        robot_status.gimbal_output = raw_robot_status->mains_power_gimbal_output;
        robot_status.shooter_output = raw_robot_status->mains_power_shooter_output;
        robot_status_pub_.publish(robot_status);
    }

    void SerialComNode::RobotHeatCallback(leonard_serial_common::cmd_power_heat_data *raw_robot_heat) {
        roborts_msgs::RobotHeat robot_heat;
        robot_heat.chassis_volt = raw_robot_heat->chassis_volt;
        robot_heat.chassis_current = raw_robot_heat->chassis_current;
        robot_heat.chassis_power = raw_robot_heat->chassis_power;
        robot_heat.chassis_power_buffer = raw_robot_heat->chassis_power_buffer;
        robot_heat.shooter_heat = raw_robot_heat->shooter_heat0;
        if (robot_heat.shooter_heat >= 150) {
            ROS_INFO("heat0:%d", (int) raw_robot_heat->shooter_heat0);
        }
        robot_heat_pub_.publish(robot_heat);
    }

    void SerialComNode::RobotBonusCallback(leonard_serial_common::cmd_buff_musk *raw_robot_bonus) {
        roborts_msgs::RobotBonus robot_bonus;
        robot_bonus.bonus = raw_robot_bonus->power_rune_buff >> 2 & 1;
        robot_bonus_pub_.publish(robot_bonus);
    }

    void SerialComNode::RobotDamageCallback(leonard_serial_common::cmd_robot_hurt *raw_robot_damage) {
        roborts_msgs::RobotDamage robot_damage;
        robot_damage.damage_type = raw_robot_damage->hurt_type;
        robot_damage.damage_source = raw_robot_damage->armor_id;
        robot_damage_pub_.publish(robot_damage);
    }

    void SerialComNode::RobotShootCallback(leonard_serial_common::cmd_shoot_data *raw_robot_shoot) {
        roborts_msgs::RobotShoot robot_shoot;
        robot_shoot.frequency = raw_robot_shoot->bullet_freq;
        robot_shoot.speed = raw_robot_shoot->bullet_speed;
        robot_shoot_pub_.publish(robot_shoot);
    }

    void SerialComNode::ProjectileSupplyCallback(const roborts_msgs::ProjectileSupply::ConstPtr projectile_supply) {
        if (projectile_supply->supply != 1 && projectile_supply->supply != 2) {
            ROS_WARN("Projectile supply command is invalid, supply flag is false.");
            return;
        } else if (robot_id_ == 0xFF) {
            ROS_ERROR("Can not get robot id before requesting for projectile supply.");
            return;
        }

        /*********官方代码*********/
        leonard_serial_common::cmd_supply_projectile_booking raw_projectile_booking;
        raw_projectile_booking.supply_projectile_id = 1;
        raw_projectile_booking.supply_robot_id = robot_id_;
        raw_projectile_booking.supply_num = 50 * projectile_supply->supply;

        //TODO
        //        projectile_supply_pub_->Publish(raw_projectile_booking);

        /*********leonard代码*********/
        if (!SendData((uint8_t *) &raw_projectile_booking, sizeof(cmd_supply_projectile_booking), REFEREE_SEND_CMD_SET,
                      CMD_REFEREE_SEND_DATA,
                      CHASSIS_ADDRESS)) {
            LOG_WARNING << "Overflow in Supply";
        }
    }


    void SerialComNode::OurBookingCallback(const roborts_base::GoalTask::ConstPtr req) {

        leonard_serial_common::cmd_our_booking booking;

        booking.sendid = robot_id_;
        if (robot_id_ == 3)
            booking.recid = 4;
        else if (robot_id_ == 4)
            booking.recid = 3;
        else if (robot_id_ == 13)
            booking.recid = 14;
        else if (robot_id_ == 14)
            booking.recid = 13;

        booking.goalx = req->goal.pose.position.x;
        booking.goaly = req->goal.pose.position.y;
        geometry_msgs::Quaternion orientation = req->goal.pose.orientation;
        tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
        double yaw, pitch, roll;
        mat.getEulerYPR(yaw, pitch, roll);
        booking.goalyaw = yaw;
        booking.is_goal_set = req->is_goal_set;
        booking.remain_hp = req->remain_hp;
        booking.supplying = req->supplying;
        booking.supplycnt = req->supplycnt;
        booking.bonusing = req->bonusing;
        booking.pose_x = req->pose_x;
        booking.pose_y = req->pose_y;
        booking.supply_warn = req->supply_warn;
        booking.lack_bullet = req->lack_bullet;

        if (!SendData((uint8_t *) &booking, sizeof(cmd_our_booking), REFEREE_SEND_CMD_SET,
                      CMD_REFEREE_SEND_DATA,
                      CHASSIS_ADDRESS)) {
            LOG_WARNING << "Overflow in ourbookingdata";
        }
    }

//    void SerialComNode::RobotOurBookingCallback(leonard_serial_common::cmd_our_booking *raw_our_booking) {
//        roborts_msgs::GoalTask robot_booking;
//
//        geometry_msgs::PoseStamped pose;
//        robot_shoot_pub_.publish(robot_shoot);
//    }


/*************************** Receive DATA ****************************/
    void SerialComNode::ReceiveLoop() {
        while (is_open_ && !stop_receive_ && ros::ok()) {
            usleep(1);
            read_buff_index_ = 0;
            //read_len_ = ReceiveData(fd_, UART_BUFF_SIZE);
            read_len_ = hardware_device_->Read(rx_buf_, UART_BUFF_SIZE);
            if (read_len_ > 0) {
                while (read_len_--) {
                    byte_ = rx_buf_[read_buff_index_++];
                    switch (unpack_step_e_) {
                        case STEP_HEADER_SOF: {
                            if (byte_ == SOF) {
                                protocol_packet_[index_++] = byte_;
                                unpack_step_e_ = STEP_LENGTH_LOW;
                            } else {
                                index_ = 0;
                            }
                        }
                            break;
                        case STEP_LENGTH_LOW: {
                            header_.ver_data_len = byte_;
                            protocol_packet_[index_++] = byte_;
                            unpack_step_e_ = STEP_LENGTH_HIGH;
                        }
                            break;
                        case STEP_LENGTH_HIGH: {
                            header_.ver_data_len |= (byte_ << 8);
                            protocol_packet_[index_++] = byte_;
                            if (header_.data_len < (PROTOCAL_FRAME_MAX_SIZE)) {
                                unpack_step_e_ = STEP_SARC;
                            } else {
                                // LOG_WARNING << "Data length too big";
                                unpack_step_e_ = STEP_HEADER_SOF;
                                index_ = 0;
                            }
                        }
                            break;
                        case STEP_SARC: {
                            protocol_packet_[index_++] = byte_;
                            header_.S_A_R_c = byte_;
                            if (header_.res == 0)
                                unpack_step_e_ = STEP_SENDER;
                            else {
                                // LOG_WARNING << "serial res error";
                                unpack_step_e_ = STEP_HEADER_SOF;
                                index_ = 0;
                            }
                        }
                            break;

                        case STEP_SENDER: {
                            protocol_packet_[index_++] = byte_;
                            header_.sender = byte_;
                            unpack_step_e_ = STEP_RECIVER;
                        }
                            break;

                        case STEP_RECIVER: {
                            protocol_packet_[index_++] = byte_;
                            header_.receiver = byte_;
                            if ((header_.receiver == DEVICE) || (header_.receiver == 0xFF))
                                unpack_step_e_ = STEP_RES1_LOW;
                            else {
                                // LOG_WARNING << "serial receiver error";
                                unpack_step_e_ = STEP_HEADER_SOF;
                                index_ = 0;
                            }
                        }
                            break;

                        case STEP_RES1_LOW: {
                            header_.res1 = byte_;
                            protocol_packet_[index_++] = byte_;
                            unpack_step_e_ = STEP_RES1_HIGH;
                        }
                            break;
                        case STEP_RES1_HIGH: {
                            header_.res1 |= (byte_ << 8);
                            protocol_packet_[index_++] = byte_;
                            if (header_.res1 == 0) {
                                unpack_step_e_ = STEP_HEADER_CRC16;
                            } else {
                                // LOG_WARNING << "Data RES1 ERROR";
                                unpack_step_e_ = STEP_HEADER_SOF;
                                index_ = 0;
                            }
                        }
                            break;

                        case STEP_HEADER_CRC16: {
                            if (index_ < (HEADER_LEN)) {
                                protocol_packet_[index_++] = byte_;
                            } else if (index_ > (HEADER_LEN)) {
                                // LOG_WARNING << "Header Index Beyond";
                                unpack_step_e_ = STEP_HEADER_SOF;
                                index_ = 0;
                            }
                            if (index_ == (HEADER_LEN)) {
                                if (CRCHeadCheck(protocol_packet_, HEADER_LEN)) {
                                    unpack_step_e_ = STEP_DATA_CRC16;
                                } else {
                                    // LOG_WARNING << "Header CRC16 error";
                                    unpack_step_e_ = STEP_HEADER_SOF;
                                    index_ = 0;
                                }
                            }
                        }
                            break;
                        case STEP_DATA_CRC16: {
                            if (index_ < header_.data_len) {
                                protocol_packet_[index_++] = byte_;
                            } else if (index_ > header_.data_len) {
                                // LOG_WARNING << "Data Index Beyond";
                                unpack_step_e_ = STEP_HEADER_SOF;
                                index_ = 0;
                            }
                            if (index_ == header_.data_len) {
                                unpack_step_e_ = STEP_HEADER_SOF;
                                index_ = 0;
                                if (CRCTailCheck(protocol_packet_, header_.data_len)) {
                                    DataHandle();
                                } else {
                                    // LOG_WARNING << "Data CRC32 error";
                                }
                            }
                        }
                            break;
                        default: {
                            // LOG_WARNING << "Unpack not well";
                            unpack_step_e_ = STEP_HEADER_SOF;
                            index_ = 0;
                        }
                            break;
                    }
                }
            }
        }
    }

    void SerialComNode::DataHandle() {
        ros::Time current_time = ros::Time::now();
        geometry_msgs::Quaternion q;
        std::lock_guard<std::mutex> guard(mutex_receive_);
        auto *p_header = (FrameHeader *) protocol_packet_;
        uint16_t data_length = p_header->data_len - HEADER_LEN - CMD_LEN - CRC_DATA_LEN;
        uint16_t cmd_id = *(uint16_t *) (protocol_packet_ + HEADER_LEN);
        uint8_t *data_addr = protocol_packet_ + HEADER_LEN + CMD_LEN;

        switch (cmd_id) {
            case CHASSIS_CMD_SET << 8 | CMD_PUSH_CHASSIS_INFO: {
                memcpy(&chassis_info_, data_addr, data_length);
                ros::Time current_time = ros::Time::now();
                odom_.header.stamp = current_time;
                odom_.pose.pose.position.x = chassis_info_.position_x_mm / 1000.;
                odom_.pose.pose.position.y = chassis_info_.position_y_mm / 1000.;
                odom_.pose.pose.position.z = 0.0;

                //leonard add
                base_yaw_ = chassis_info_.gyro_angle / 1800.0 * M_PI;

                geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(chassis_info_.gyro_angle / 1800.0 * M_PI);
                odom_.pose.pose.orientation = q;
                odom_.twist.twist.linear.x = chassis_info_.v_x_mm / 1000.0;
                odom_.twist.twist.linear.y = chassis_info_.v_y_mm / 1000.0;
                odom_.twist.twist.angular.z = chassis_info_.gyro_rate / 1800.0 * M_PI;

                // std::cout << odom_.twist.twist.angular.z << std::endl;

                odom_pub_.publish(odom_);
                odom_tf_.header.stamp = current_time;
                odom_tf_.transform.translation.x = chassis_info_.position_x_mm / 1000.;
                odom_tf_.transform.translation.y = chassis_info_.position_y_mm / 1000.;
                odom_tf_.transform.translation.z = 0.0;
                odom_tf_.transform.rotation = q;
                tf_broadcaster_.sendTransform(odom_tf_);

                /***********剩余弹药量************/
                roborts_base::Bullet bullet;
                bullet.bullet_num = chassis_info_.bullet;
                //!TODO 比赛时请不要注释!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                bullet_pub_.publish(bullet);
                /***********剩余弹药量************/

                if (highSpeedSta_ == false &&
                    (fabs(chassis_info_.v_x_mm) > 2000 || fabs(chassis_info_.v_y_mm) > 2000 ||
                     fabs(chassis_info_.gyro_rate) > 2000)) {
                    highSpeedCnt_ += 10;
                    if (highSpeedCnt_ >= 200) {
                        highSpeedCnt_ = 200;
                        highSpeedSta_ = true;
                        roborts_msgs::ForceUpdateAmcl srv;
                        srv.request.mode = 1;
                        high_speed_client_.call(srv);
                    }
                } else if (highSpeedSta_ == true && fabs(chassis_info_.v_x_mm) <= 2000 &&
                           fabs(chassis_info_.v_y_mm) <= 2000 && chassis_info_.gyro_rate <= 2000) {
                    if (--highSpeedCnt_ <= 0) {
                        highSpeedCnt_ = 0;
                        highSpeedSta_ = false;
                        roborts_msgs::ForceUpdateAmcl srv;
                        srv.request.mode = 0;
                        high_speed_client_.call(srv);
                    }
                }
                //                if(csvWriter_!=nullptr){
                //                csvWriter_->write((int)chassis_info_.position_x_mm);
                //                csvWriter_->write((int)chassis_info_.position_y_mm);
                //                csvWriter_->write((int)chassis_info_.gyro_angle);
                //                csvWriter_->write((int)chassis_info_.v_x_mm);
                //                csvWriter_->write((int)chassis_info_.v_y_mm);
                //                csvWriter_->write((int)chassis_info_.gyro_rate);
                //                }
            }
                break;
            case GIMBAL_CMD_SET << 8 | CMD_PUSH_GIMBAL_INFO: {
                memcpy(&gimbal_info_, data_addr, data_length);
                ros::Time current_time = ros::Time::now();

                double yaw_angle = gimbal_info_.yaw_ecd_angle / 1800.0 * M_PI;
                geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0,
                        //   gimbal_info_.pitch_ecd_angle /
                        //   1800.0 * M_PI,
                                                                                      0.0,
                                                                                      yaw_angle);
                gimbal_tf_.header.stamp = current_time;
                gimbal_tf_.transform.rotation = q;
                gimbal_tf_.transform.translation.x = 0.09 * std::cos(yaw_angle);
                gimbal_tf_.transform.translation.y = 0.09 * std::sin(yaw_angle);
                gimbal_tf_.transform.translation.z = 0.15;
                tf_broadcaster_.sendTransform(gimbal_tf_);
            }
                break;

            case REFEREE_GAME_CMD_SET << 8 | CMD_GAME_STATUS: {
                memcpy(&raw_game_status_, data_addr, data_length);
                GameStateCallback(&raw_game_status_);
            }
                break;

            case REFEREE_GAME_CMD_SET << 8 | CMD_GAME_RESULT: {
                memcpy(&raw_game_result_, data_addr, data_length);
                GameResultCallback(&raw_game_result_);
            }
                break;

            case REFEREE_GAME_CMD_SET << 8 | CMD_GAME_SURVIVAL: {
                memcpy(&raw_game_survivor_, data_addr, data_length);
                GameSurvivorCallback(&raw_game_survivor_);
            }
                break;

            case REFEREE_BATTLEFIELD_CMD_SET << 8 | CMD_BATTLEFIELD_EVENT: {
                memcpy(&raw_game_event_, data_addr, data_length);
                GameEventCallback(&raw_game_event_);
            }
                break;

            case REFEREE_BATTLEFIELD_CMD_SET << 8 | CMD_SUPPLIER_ACTION: {
                memcpy(&raw_supplier_status_, data_addr, data_length);
                SupplierStatusCallback(&raw_supplier_status_);
            }
                break;
                /**  Robot Related  **/

            case REFEREE_ROBOT_CMD_SET << 8 | CMD_ROBOT_STATUS: {
                memcpy(&raw_robot_status_, data_addr, data_length);
                RobotStatusCallback(&raw_robot_status_);
            }
                break;

            case REFEREE_ROBOT_CMD_SET << 8 | CMD_ROBOT_POWER_HEAT: {
                memcpy(&raw_robot_heat_, data_addr, data_length);
                RobotHeatCallback(&raw_robot_heat_);
            }
                break;

            case REFEREE_ROBOT_CMD_SET << 8 | CMD_ROBOT_BUFF: {
                memcpy(&raw_robot_bonus_, data_addr, data_length);
                RobotBonusCallback(&raw_robot_bonus_);
            }
                break;

            case REFEREE_ROBOT_CMD_SET << 8 | CMD_ROBOT_HURT: {
                memcpy(&raw_robot_damage_, data_addr, data_length);
                RobotDamageCallback(&raw_robot_damage_);
            }
                break;

            case REFEREE_ROBOT_CMD_SET << 8 | CMD_ROBOT_SHOOT: {
                memcpy(&raw_robot_shoot_, data_addr, data_length);
                RobotShootCallback(&raw_robot_shoot_);
            }
                break;

            case 0x4301:
                memcpy(&raw_our_booking_, data_addr, data_length);


            default:
                break;
        }
    }

/*************************** Send DATA ****************************/
    void SerialComNode::SendPack() {
        while (is_open_ && !stop_send_ && ros::ok()) {
            if (total_length_ > 0) {
                mutex_send_.lock();
                //                SendData(total_length_);
                hardware_device_->Write(tx_buf_, total_length_);
                total_length_ = 0;
                free_length_ = UART_BUFF_SIZE;
                mutex_send_.unlock();
            } else {
                usleep(100);
            }
        }
    }

    bool SerialComNode::SendData(uint8_t *data, int len, uint8_t cmd_set, uint8_t cmd_id, uint8_t receiver) {
        std::unique_lock<std::mutex> lock(mutex_pack_);
        uint8_t pack[PACK_MAX_SIZE];
        int pack_length = len + HEADER_LEN + CMD_LEN + CRC_DATA_LEN;
        SendDataHandle(data, pack, len, cmd_set, cmd_id, receiver);
        if (pack_length <= free_length_) {
            memcpy(tx_buf_ + total_length_, pack, pack_length);
            free_length_ -= pack_length;
            total_length_ += pack_length;
            return true;
        } else {
            return false;
        }
    }

    void SerialComNode::SendDataHandle(uint8_t *topack_data,
                                       uint8_t *packed_data,
                                       uint16_t len,
                                       uint8_t cmd_set,
                                       uint8_t cmd_id,
                                       uint8_t receiver) {
        FrameHeader *p_header = (FrameHeader *) packed_data;

        uint8_t cmd_set_prefix[] = {cmd_id, cmd_set};
        uint32_t crc_data;

        uint16_t pack_length = 0;

        pack_length = HEADER_LEN +
                      CMD_LEN +
                      len + CRC_DATA_LEN;

        p_header->sof = SOF;
        p_header->data_len = pack_length;
        p_header->version = VERSION;
        p_header->session = 0;
        p_header->pack_type = 0;
        p_header->res = 0;
        p_header->sender = DEVICE;
        p_header->receiver = receiver;
        p_header->res1 = 0;
        //        p_header->seq_num = seq_num_;
        p_header->crc_16 = CRC16Calc(packed_data, HEADER_LEN - CRC_HEAD_LEN);

        //        std::cout << "shiyishi" << std::endl;;
        //        std::cout << CRCHeadCheck(packed_data, HEADER_LEN) << std::endl;

        // pack the cmd prefix ,data and data crc into memory block one by one
        memcpy(packed_data + HEADER_LEN, (uint8_t *) &cmd_set_prefix, CMD_LEN);

        memcpy(packed_data + HEADER_LEN + CMD_LEN, topack_data, len);

        crc_data = CRC32Calc(packed_data, pack_length - CRC_DATA_LEN);
        memcpy(packed_data + len + HEADER_LEN + CMD_LEN, &crc_data, CRC_DATA_LEN);

        //leonard Test
        //        std::cout << "shiyishi" << std::endl;;
        //        std::cout << CRCTailCheck(packed_data, pack_length) << std::endl;
        //        Test(packed_data, pack_length);
    }

/*************************** CRC Calculationns ****************************/
    uint16_t SerialComNode::CRC16Update(uint16_t crc, uint8_t ch) {
        uint16_t tmp;
        uint16_t msg;

        msg = 0x00ff & static_cast<uint16_t>(ch);
        tmp = crc ^ msg;
        crc = (crc >> 8) ^ crc_tab16[tmp & 0xff];

        return crc;
    }

    uint32_t SerialComNode::CRC32Update(uint32_t crc, uint8_t ch) {
        uint32_t tmp;
        uint32_t msg;

        msg = 0x000000ffL & static_cast<uint32_t>(ch);
        tmp = crc ^ msg;
        crc = (crc >> 8) ^ crc_tab32[tmp & 0xff];
        return crc;
    }

    uint16_t SerialComNode::CRC16Calc(const uint8_t *data_ptr, size_t length) {
        size_t i;
        uint16_t crc = CRC_INIT;

        for (i = 0; i < length; i++) {
            crc = CRC16Update(crc, data_ptr[i]);
        }

        return crc;
    }

    uint32_t SerialComNode::CRC32Calc(const uint8_t *data_ptr, size_t length) {
        size_t i;
        uint32_t crc = CRC_INIT;

        for (i = 0; i < length; i++) {
            crc = CRC32Update(crc, data_ptr[i]);
        }

        return crc;
    }

    bool SerialComNode::CRCHeadCheck(uint8_t *data_ptr, size_t length) {
        if (CRC16Calc(data_ptr, length) == 0) {
            return true;
        } else {
            return false;
        }
    }

    bool SerialComNode::CRCTailCheck(uint8_t *data_ptr, size_t length) {
        if (CRC32Calc(data_ptr, length) == 0) {
            return true;
        } else {
            return false;
        }
    }

} // namespace leonard_serial_common

int main(int argc, char **argv) {
    leonard_serial_common::GLogWrapper gLogWrapper(argv[0]);
    ros::init(argc, argv, "roborts_base_node");
    leonard_serial_common::SerialComNode serial_common_node("roborts_base_node");
    ros::spin();
    ros::waitForShutdown();
    return 0;
}
