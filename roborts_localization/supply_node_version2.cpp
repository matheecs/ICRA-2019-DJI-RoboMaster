//
// Created by cxn on 19-2-15.
//
#include "supply_node_version2.h"

namespace leonard_supply {

#define VAL_LIMIT(val, min, max) \
    if ((val) <= (min))          \
    {                            \
        (val) = (min);           \
    }                            \
    else if ((val) >= (max))     \
    {                            \
        (val) = (max);           \
    }

    SupplyNodeVersion2::SupplyNodeVersion2(std::string name) {
        CHECK(Init()) << "Module " << name << " initialized failed!";
    }

    bool SupplyNodeVersion2::Init() {
        supplypid_ptr_ = std::make_shared<SupplyPid>("supplypid");
        return true;
    }

    SupplyPid::SupplyPid(std::string
                         name) : as_(nh_, name, boost::bind(&SupplyPid::executeCB, this, _1), false),
                                 action_name_(name),
                                 global_planner_actionlib_client_("global_planner_node_action", true),
                                 local_planner_actionlib_client_("local_planner_node_action", true) {
        as_.start();

        supply_cmd_pub_ = nh_.advertise<roborts_msgs::ProjectileSupply>("projectile_supply", 1);

        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 30);

        //用与纠正机器人错误的位姿
        modify_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 2);

        supplier_status_sub_ = nh_.subscribe("field_supplier_status", 2, &SupplyPid::SupplierStatusCallback, this);

        robot_status_sub_ = nh_.subscribe("robot_status", 2, &SupplyPid::RobotStatusCallback, this);

        std::cout << "waitServer" << std::endl;
        global_planner_actionlib_client_.waitForServer();
        local_planner_actionlib_client_.waitForServer();
        std::cout << "waitServerDone" << std::endl;

        start_vision_client_ = nh_.serviceClient<roborts_localization::StartVision>("start_vision");

        relocate_client_ = nh_.serviceClient<roborts_msgs::ForceUpdateAmcl>("set_force_update_mode");


        nh_.param<double>("supply_pid/x_p", pid_p_(0), 4);
        nh_.param<double>("supply_pid/y_p", pid_p_(1), 4);
        nh_.param<double>("supply_pid/z_p", pid_p_(2), 4);
        nh_.param<double>("supply_pid/x_tolerance", tolerance_dis_(0), 0.05);
        nh_.param<double>("supply_pid/y_tolerance", tolerance_dis_(1), 0.05);
        nh_.param<double>("supply_pid/z_tolerance", tolerance_dis_(2), 0.1);
        nh_.param<double>("supply_pid/target_supply_pose_x", target_pose_(0), 0.38);
        nh_.param<double>("supply_pid/target_supply_pose_y", target_pose_(1), 0.0);
        nh_.param<double>("supply_pid/target_supply_pose_z", target_pose_(2), 0.0);
        nh_.param<double>("supply_pid/max_supply_speed", max_supply_speed_, 0.3);
        nh_.param<double>("supply_pid/target_supply_pose_x_map", target_pose_map_(0), 4);
        nh_.param<double>("supply_pid/target_supply_pose_y_map", target_pose_map_(1), 4.2);
        nh_.param<double>("supply_pid/target_supply_pose_x_map_2", target_pose_map_x_2_, 4.0);
        nh_.param<double>("supply_pid/target_supply_pose_x_map_3", target_pose_map_x_3_, 3.8);
        nh_.param<double>("supply_pid/target_supply_pose_x_map_4", target_pose_map_x_4_, 4.2);
        nh_.param<double>("supply_pid/target_supply_pose_x_map_5", target_pose_map_x_5_, 4.0);
        nh_.param<double>("supply_pid/target_supply_pose_y_map_2", target_pose_map_y_2_, 4.4);
        nh_.param<double>("supply_pid/target_supply_pose_y_map_3", target_pose_map_y_3_, 4.4);
        nh_.param<double>("supply_pid/target_supply_pose_y_map_4", target_pose_map_y_4_, 4.4);
        nh_.param<double>("supply_pid/target_supply_pose_y_map_5", target_pose_map_y_5_, 4.4);
        nh_.param<double>("supply_pid/target_supply_pose_z_map", target_pose_map_(2), M_PI_2);
    }

    void SupplyPid::executeCB(const roborts_localization::SupplyPidGoalConstPtr &goal) {

        // helper variables
        ros::Rate r(10); //100ms,回复一次
        bool success = true;
        std::cout << "goal:" << (int) (goal->command) << std::endl;

        if (goal->command == 1 || goal->command == 2) {
            while (1) {
                if (as_.isPreemptRequested() || !ros::ok()) {
                    success = false;
                    Reset();
                    as_.setPreempted();
                    ROS_INFO("ptr ispreempt");
                    break;
                }
                if (blind_delay_ == 0) {
                    CallForceUpdate();
                    blind_delay_++;
                } else if (blind_delay_ < 3) {
                    blind_delay_++;//等掉1,2,即0.2s
                } else if (blind_delay_ == 3) {
                    geometry_msgs::PoseStamped goal;
                    goal.header.frame_id = "map";
                    goal.header.stamp = ros::Time::now();
                    goal.pose.position.x = target_pose_map_(0);
                    goal.pose.position.y = target_pose_map_(1);
                    goal.pose.orientation = tf::createQuaternionMsgFromYaw(
                            target_pose_map_(2));
                    SendGoal(goal, 0.8);
                    blind_delay_++;
                }
                if (blind_delay_ == 4) {
                    UpdateActionState();
                    if (planner_state_ == PlannerState::SUCCESS || planner_state_ == PlannerState::FAILURE) {
                        blind_delay_++;
                        //发布命令
                        roborts_msgs::ProjectileSupply projectileSupply;
                        projectileSupply.supply = goal->command;
                        supply_cmd_pub_.publish(projectileSupply);
                        wait_timestamp_ = ros::Time().now();
                    }
                }

                if (blind_delay_ == 5) {
                    geometry_msgs::PoseStamped goal;
                    goal.header.frame_id = "map";
                    goal.header.stamp = ros::Time::now();
                    if (blind_cnt_ == 0) {
                        goal.pose.position.x = target_pose_map_x_2_;
                        goal.pose.position.y = target_pose_map_y_2_;
                    } else if (blind_cnt_ == 1) {
                        goal.pose.position.x = target_pose_map_x_3_;
                        goal.pose.position.y = target_pose_map_y_3_;
                    }
                    goal.pose.orientation = tf::createQuaternionMsgFromYaw(
                            target_pose_map_(2));
                    SendGoal(goal, 0.8);
                    blind_delay_++;
                }

                if (blind_delay_ == 6) {
                    UpdateActionState();
                    if (planner_state_ == PlannerState::SUCCESS || planner_state_ == PlannerState::FAILURE) {
                        blind_delay_ = 5;
                        if (blind_cnt_ == 0)
                            blind_cnt_ = 1;
                        else
                            blind_cnt_ = 0;
                    }
                }

                if (blind_delay_ == 5 || blind_delay_ == 6) {
                    if ((ros::Time().now() - wait_timestamp_ > ros::Duration(20))) {
                        Reset();
                        break;
                    }
                }

                ROS_INFO("blind_delay_: %d", (int) blind_delay_);

                feedback_.detected = true;
                as_.publishFeedback(feedback_);
                r.sleep();
            }

            if (success) {
                ROS_INFO("yes");
                result_.result = true;
                as_.setSucceeded(result_);
            }

        } else {
            result_.result = false;
            as_.setAborted(result_);
        }
    }


    void SupplyPid::RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &msg) {
        if ((msg->id == 3 || msg->id == 4) && target_tag_id_ != 1) {
            target_tag_id_ = 1;
            robot_id_ = msg->id;
        } else if ((msg->id == 13 || msg->id == 14) && target_tag_id_ != 2) {
            target_tag_id_ = 2;
            robot_id_ = msg->id;
        }
    }


    void SupplyPid::Reset() {
        supply_status_[0] = supply_status_[1] = roborts_msgs::SupplierStatus::CLOSE;
        blind_delay_ = 0;
        blind_cnt_ = 0;
        CancelGoal();
        //! 5.13 add
        supply_done_flag_ = false;
    }


    void SupplyPid::CallForceUpdate() {
        //发送强制更新pose服务
        roborts_msgs::ForceUpdateAmcl srv;
        srv.request.mode = 2;
        relocate_client_.call(srv);
    }

    void SupplyPid::SupplierStatusCallback(const roborts_msgs::SupplierStatus::ConstPtr &msg) {
        supply_status_[1] = supply_status_[0];
        supply_status_[0] = msg->supply_projectile_step;

        if (supply_status_[0] == roborts_msgs::SupplierStatus::CLOSE &&
            supply_status_[1] == roborts_msgs::SupplierStatus::SUPPLYING) {
            supply_done_time_ = ros::Time::now();
            supply_done_flag_ = true;
        } else if (supply_status_[0] == roborts_msgs::SupplierStatus::SUPPLYING &&
                   supply_status_[1] == roborts_msgs::SupplierStatus::CLOSE) {
            supply_done_flag_ = false;
        }

    }


/*******************planner state***********************/
    void SupplyPid::SetLocalPlannerMaxVel(float max = 1.5) {
        local_planner_goal_.maxvelx = max;
        local_planner_goal_.maxvely = max;
    }

    void SupplyPid::SendGoal(geometry_msgs::PoseStamped goal, float maxvel) {
        SetLocalPlannerMaxVel(maxvel);
        global_planner_goal_.goal = goal;
        global_planner_actionlib_client_.sendGoal(global_planner_goal_,
                                                  GlobalActionClient::SimpleDoneCallback(),
                                                  GlobalActionClient::SimpleActiveCallback(),
                                                  boost::bind(&SupplyPid::GlobalPlannerFeedbackCallback, this, _1));
    }

    void SupplyPid::GlobalPlannerFeedbackCallback(const GlobalFeedback &feedback) {
        if (!feedback->path.poses.empty()) {
            local_planner_goal_.route = feedback->path;
            // Change the local planner's velocity:
//            local_planner_goal_.delta = 0.03;
            local_planner_actionlib_client_.sendGoal(local_planner_goal_);
        }
    }

    void SupplyPid::CancelGoal() {
        if (planner_state_ == PlannerState::RUNNING) {
            global_planner_actionlib_client_.cancelGoal();
            local_planner_actionlib_client_.cancelGoal();
            planner_state_ = PlannerState::IDLE;
        }
    }

/*******************planner state***********************/

    void SupplyPid::UpdateActionState() {
        auto state = global_planner_actionlib_client_.getState();
        if (state == actionlib::SimpleClientGoalState::ACTIVE) {
            planner_state_ = PlannerState::RUNNING;
        } else if (state == actionlib::SimpleClientGoalState::PENDING) {
            planner_state_ = PlannerState::RUNNING;
        } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            planner_state_ = PlannerState::SUCCESS;
        } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
            planner_state_ = PlannerState::FAILURE;
        } else {
            planner_state_ = PlannerState::FAILURE;
        }
    }


} // namespace leonard_localization

int main(int argc, char **argv) {
    leonard_localization::GLogWrapper gLogWrapper(argv[0]);
    ros::init(argc, argv, "supply_node_version2");
    leonard_supply::SupplyNodeVersion2 supply_node("supply_node_version2");
    ros::spin();
    return 0;
}
