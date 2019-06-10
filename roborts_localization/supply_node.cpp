//
// Created by cxn on 19-2-15.
//
#include "supply_node.h"

namespace leonard_localization
{

#define VAL_LIMIT(val, min, max) \
    if ((val) <= (min))          \
    {                            \
        (val) = (min);           \
    }                            \
    else if ((val) >= (max))     \
    {                            \
        (val) = (max);           \
    }

SupplyNode::SupplyNode(std::string name)
{
    CHECK(Init()) << "Module " << name << " initialized failed!";
}

bool SupplyNode::Init()
{
    supplypid_ptr_ = std::make_shared<SupplyPid>("supplypid");
    return true;
}

SupplyPid::SupplyPid(std::string
                         name) : as_(nh_, name, boost::bind(&SupplyPid::executeCB, this, _1), false),
                                 action_name_(name),
                                 global_planner_actionlib_client_("global_planner_node_action", true),
                                 local_planner_actionlib_client_("local_planner_node_action", true)
{
    as_.start();

    supply_cmd_pub_ = nh_.advertise<roborts_msgs::ProjectileSupply>("projectile_supply", 1);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 30);

    //用与纠正机器人错误的位姿
    modify_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 2);

    supplier_status_sub_ = nh_.subscribe("field_supplier_status", 2, &SupplyPid::SupplierStatusCallback, this);

    tag_detections_sub_ = nh_.subscribe("tag_detections", 2, &SupplyPid::TagCallback, this);

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

    // double tempd=std::sqrt(std::pow(target_pose_(0),2)+std::pow(target_pose_(1),2));
    // double tempa=std::atan2(target_pose_(1),target_pose_(0));
    // target_pose_(0)=tempd*cos(tempa+target_pose_(2));
    // target_pose_(1)=tempd*sin(tempa+target_pose_(2));
    // std::cout<<"target_pose"<< target_pose_(0)<<","<< target_pose_(1)<<","<< tempd<<","<< tempa<<std::endl;
}

void SupplyPid::executeCB(const roborts_localization::SupplyPidGoalConstPtr &goal)
{

    // helper variables
    ros::Rate r(10); //100ms,回复一次
    bool success = true;
    std::cout << "goal:" << (int)(goal->command) << std::endl;

    if (goal->command == 1 || goal->command == 2)
    {

        RestAndProcessState(ProcessState::PID);
        DetectedTagSwitch(true);

        //!5.21 Add
        end_cnt_ = 0;

        while (1)
        {
            variable_lock_.lock();
            //被打断
            if (as_.isPreemptRequested() || !ros::ok())
            {
                DetectedTagSwitch(false);
                RestAndProcessState(ProcessState::NOREACH);
                success = false;

                //!5.21 Add
                end_cnt_ = 0;

                as_.setPreempted();
                variable_lock_.unlock();
                //                    std::cout << "ptr ispreempt" << std::endl;
                ROS_INFO("ptr ispreempt");
                break;
            }

            if (process_state_ == ProcessState::BLINDERROR)
            {
                if (blind_delay_ == 0)
                {
                    CallForceUpdate();
                    blind_delay_++;
                }
                else if (blind_delay_ < 3)
                {
                    blind_delay_++; //等掉1,2,即0.2s
                }
                else if (blind_delay_ == 3)
                {
                    geometry_msgs::PoseStamped goal;
                    goal.header.frame_id = "map";
                    goal.header.stamp = ros::Time::now();
                    goal.pose.position.x = target_pose_map_(0);
                    if (blind_cnt_ == 0)
                    {
                        goal.pose.position.x = target_pose_map_x_2_;
                        goal.pose.position.y = target_pose_map_y_2_;
                    }
                    else if (blind_cnt_ == 1)
                    {
                        goal.pose.position.x = target_pose_map_x_3_;
                        goal.pose.position.y = target_pose_map_y_3_;
                    }
                    else if (blind_cnt_ == 2)
                    {
                        goal.pose.position.x = target_pose_map_x_4_;
                        goal.pose.position.y = target_pose_map_y_4_;
                    }
                    else if (blind_cnt_ == 3)
                    {
                        goal.pose.position.x = target_pose_map_x_5_;
                        goal.pose.position.y = target_pose_map_y_5_;
                    }
                    goal.pose.orientation = tf::createQuaternionMsgFromYaw(
                        target_pose_map_(2));
                    SendGoal(goal, 0.8);
                    blind_delay_++;
                }
                if (blind_delay_ == 4)
                {
                    UpdateActionState();
                    if (planner_state_ == PlannerState::SUCCESS || planner_state_ == PlannerState::FAILURE)
                    {
                        blind_cnt_++;
                        blind_delay_ = 0;
                    }

                    if (blind_cnt_ > 3)
                    {
                        DetectedTagSwitch(false);
                        RestAndProcessState(ProcessState::NOREACH);
                        success = false;
                        result_.result = false;
                        as_.setAborted(result_);
                        variable_lock_.unlock();
                        break;
                    }
                }
            }
            else if (process_state_ == ProcessState::LOSTPOSEERROR)
            {
                UpdateActionState();
                if (planner_state_ == PlannerState::SUCCESS)
                {
                    RestAndProcessState(ProcessState::PID);
                    DetectedTagSwitch(true);
                }
                else if (planner_state_ == PlannerState::FAILURE)
                {
                    RestAndProcessState(ProcessState::NOREACH);
                    success = false;
                    result_.result = false;
                    as_.setAborted(result_);
                    variable_lock_.unlock();
                    break;
                }
            }
            else if (process_state_ == ProcessState::REACH)
            {
                DetectedTagSwitch(false);
                //发送补弹指令
                roborts_msgs::ProjectileSupply projectileSupply;
                //projectileSupply.supply = 1;
                projectileSupply.supply = goal->command;
                supply_cmd_pub_.publish(projectileSupply);
                process_state_ = ProcessState::WAITSUPPLY;
                wait_timestamp_ = ros::Time().now();
            }
            else if (process_state_ == ProcessState::WAITSUPPLY)
            {

                //! 官方裁判系統不太靠譜，存在着跳变的情况，所以换一个保险版本的
                //                    if ((supply_status_[0] == roborts_msgs::SupplierStatus::CLOSE &&
                //                         supply_status_[1] == roborts_msgs::SupplierStatus::SUPPLYING) ||
                //                        (ros::Time().now() - wait_timestamp_ > ros::Duration(6)))

                if (
                    //                            (supply_done_flag_ && ros::Time().now() - supply_done_time_ > ros::Duration(1.5))
                    //                        ||
                    (ros::Time().now() - wait_timestamp_ > ros::Duration(20)))
                {
                    Reset();
                    process_state_ = ProcessState::NOREACH;
                    variable_lock_.unlock();
                    break;
                }
            }

            //                std::cout << "process_state_: " << (int) process_state_ << std::endl;

            //
            if (process_state_ == ProcessState::BLINDERROR || process_state_ == ProcessState::PID)
            {
                if (++end_cnt_ >= 20)
                {
                    if (process_state_ == ProcessState::PID)
                    {
                        CancelWhirl();
                    }
                    else
                    {
                        CancelGoal();
                        CancelWhirl();
                    }
                    process_state_ = ProcessState::REACH;
                }
            }
            else
            {
                end_cnt_ = 0;
            }

            ROS_INFO("process_state_: %d", (int)process_state_);

            feedback_.detected = true;
            as_.publishFeedback(feedback_);
            variable_lock_.unlock();
            r.sleep();
        }

        if (success)
        {
            //                std::cout << "yes!" << std::endl;

            ROS_INFO("yes");

            result_.result = true;
            as_.setSucceeded(result_);
        }
    }
    else
    {
        result_.result = false;
        as_.setAborted(result_);
    }
}

void SupplyPid::TagCallback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(variable_lock_);

    if (process_state_ == ProcessState::PID || process_state_ == ProcessState::BLINDERROR)
    {

        bool detected = false;

        if (msg->detections.size() == 1 && msg->detections[0].id.size() == 1)
        {
            detected_tag_id_ = msg->detections[0].id[0] + 1;
            //tags_detected
            if (detected_tag_id_ == 1 || detected_tag_id_ == 2)
            {

                if (process_state_ == ProcessState::BLINDERROR)
                {
                    RestAndProcessState(ProcessState::PID);
                }

                if (detected_tag_id_ == target_tag_id_)
                {
                    if (process_state_ == ProcessState::REACH)
                    {
                        CancelWhirl();
                    }
                    else
                    {
                        this->PidProcess(msg);
                    }
                }
                else
                {

                    tf::Pose tagToGimbal;
                    tf::poseMsgToTF(msg->detections[0].pose.pose.pose, tagToGimbal);
                    double yaw, pitch, roll;
                    tagToGimbal.getBasis().getEulerYPR(yaw, pitch, roll);

                    Vec3d tag_gimbal_vec, true_pose;
                    tag_gimbal_vec(0) = tagToGimbal.getOrigin().z(); //tag相对于相机的深度
                    tag_gimbal_vec(1) = -tagToGimbal.getOrigin().x();
                    tag_gimbal_vec(2) = -pitch;
                    true_pose(0) = 4 - tag_gimbal_vec(1);
                    true_pose(1) = tag_gimbal_vec(0) + 0.235; //相机装前面 tag_gimbal_vec(0) + 0.09;
                    true_pose(2) = -target_pose_map_(2) - tag_gimbal_vec(2);

                    geometry_msgs::PoseWithCovarianceStamped poseWithCovarianceStamped;
                    poseWithCovarianceStamped.header.stamp = ros::Time::now();
                    poseWithCovarianceStamped.header.frame_id = "map";
                    poseWithCovarianceStamped.pose.pose.position.x = true_pose(0);
                    poseWithCovarianceStamped.pose.pose.position.y = true_pose(1);
                    poseWithCovarianceStamped.pose.pose.orientation = tf::createQuaternionMsgFromYaw(
                        true_pose(2));
                    poseWithCovarianceStamped.pose.covariance.at(
                        0) = poseWithCovarianceStamped.pose.covariance.at(7) = poseWithCovarianceStamped.pose.covariance.at(35) = 0.1;
                    modify_pose_pub_.publish(poseWithCovarianceStamped);

                    //丢了,发送新目标
                    process_state_ = ProcessState::LOSTPOSEERROR;
                    geometry_msgs::PoseStamped goal;
                    goal.header.frame_id = "map";
                    goal.header.stamp = ros::Time::now();
                    goal.pose.position.x = target_pose_map_(0);
                    goal.pose.position.y = target_pose_map_(1);
                    goal.pose.orientation = tf::createQuaternionMsgFromYaw(
                        target_pose_map_(2));
                    SendGoal(goal);
                    DetectedTagSwitch(false);
                    //                        std::cout << "we reach error point" << std::endl;
                    ROS_INFO("we reach error point");
                }
                detected = true;
            }
        }
        if (detected == false)
        {
            if (++undetected_cnt_ > 20)
            {
                //开启微调模式
                process_state_ = ProcessState::BLINDERROR;
                undetected_cnt_ = 0;
                //                    std::cout << "we can't see" << std::endl;
                ROS_INFO("we can't see");
            }
        }
    }
}

void SupplyPid::RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &msg)
{
    if ((msg->id == 3 || msg->id == 4) && target_tag_id_ != 1)
    {
        target_tag_id_ = 1;
        robot_id_ = msg->id;
    }
    else if ((msg->id == 13 || msg->id == 14) && target_tag_id_ != 2)
    {
        target_tag_id_ = 2;
        robot_id_ = msg->id;
    }
}

void SupplyPid::PidProcess(const apriltags2_ros::AprilTagDetectionArray::ConstPtr &msg)
{

    tf::Pose tagToGimbal;
    tf::poseMsgToTF(msg->detections[0].pose.pose.pose, tagToGimbal);
    double yaw, pitch, roll;
    tagToGimbal.getBasis().getEulerYPR(yaw, pitch, roll);
    Vec3d tag_gimbal_vec;
    tag_gimbal_vec(0) = tagToGimbal.getOrigin().z();
    tag_gimbal_vec(1) = -tagToGimbal.getOrigin().x();
    tag_gimbal_vec(2) = -pitch;

    geometry_msgs::Twist twist;
    Vec3d distence;
    // distence(0) = tag_gimbal_vec(0) - target_pose_(0);
    // distence(1) = tag_gimbal_vec(1) - target_pose_(1);
    // distence(2) = tag_gimbal_vec(2) - target_pose_(2);

    distence(0) = tag_gimbal_vec(0) - target_pose_(0);
    distence(1) = tag_gimbal_vec(1) - target_pose_(1);
    distence(2) = tag_gimbal_vec(2) - target_pose_(2);

    twist.linear.x = -distence(0) * pid_p_(0); //相机装尾巴与装前方相反 distence(0) * pid_p_(0);
    twist.linear.y = -distence(1) * pid_p_(1); //相机装尾巴与装前方相反 distence(1) * pid_p_(1);
    twist.angular.z = distence(2) * pid_p_(2); //相机装尾巴与装前方相同 distence(2) * pid_p_(2);

    VAL_LIMIT(twist.linear.x, -max_supply_speed_, max_supply_speed_);
    VAL_LIMIT(twist.linear.y, -max_supply_speed_, max_supply_speed_);
    VAL_LIMIT(twist.angular.z, -max_supply_speed_, max_supply_speed_);

    cmd_vel_pub_.publish(twist);

    //        std::cout << "twist.linear.x:" << twist.linear.x << std::endl;
    //        std::cout << "twist.linear.y:" << twist.linear.y << std::endl;
    //        std::cout << "twist.angular.z:" << twist.angular.z << std::endl;
    //        std::cout << "<<<<<<<<<<<<<<" << std::endl;

    ROS_INFO("twist.linear.x:%f", twist.linear.x);
    ROS_INFO("twist.linear.y:%f", twist.linear.y);
    ROS_INFO("twist.angular.z:%f", twist.angular.z);
    ROS_INFO("<<<<<<<<<<<<<<");

    if (fabs(distence(0)) < tolerance_dis_(0) &&
        fabs(distence(1)) < tolerance_dis_(1) &&
        fabs(distence(2)) < tolerance_dis_(2))
    {
        if (++reach_cnt_ > 10)
        {
            reach_cnt_ = 0;
            process_state_ = ProcessState::REACH;
            CancelWhirl();
            //                std::cout << "we reach" << std::endl;
            ROS_INFO("we reach");
        }
    }
    else
    {
        reach_cnt_ = 0;
    }

    //tags_detected
    undetected_cnt_ = 0;
}

void SupplyPid::Reset()
{
    undetected_cnt_ = 0;
    reach_cnt_ = 0;
    supply_status_[0] = supply_status_[1] = roborts_msgs::SupplierStatus::CLOSE;
    blind_delay_ = 0;
    blind_cnt_ = 0;

    //! 5.13 add
    supply_done_flag_ = false;
}

void SupplyPid::RestAndProcessState(ProcessState state)
{
    CancelGoal();
    CancelWhirl();
    Reset();
    process_state_ = state;
}

void SupplyPid::DetectedTagSwitch(bool isAllowed)
{
    roborts_localization::StartVision srv;
    srv.request.start_vision = isAllowed;
    start_vision_client_.call(srv);
}

void SupplyPid::CallForceUpdate()
{
    //发送强制更新pose服务
    roborts_msgs::ForceUpdateAmcl srv;
    srv.request.mode = 2;
    relocate_client_.call(srv);
}

void SupplyPid::SupplierStatusCallback(const roborts_msgs::SupplierStatus::ConstPtr &msg)
{
    //        if (robot_id_ = msg->supply_robot_id) {
    supply_status_[1] = supply_status_[0];
    supply_status_[0] = msg->supply_projectile_step;

    if (supply_status_[0] == roborts_msgs::SupplierStatus::CLOSE &&
        supply_status_[1] == roborts_msgs::SupplierStatus::SUPPLYING)
    {
        supply_done_time_ = ros::Time::now();
        supply_done_flag_ = true;
    }
    else if (supply_status_[0] == roborts_msgs::SupplierStatus::SUPPLYING &&
             supply_status_[1] == roborts_msgs::SupplierStatus::CLOSE)
    {
        supply_done_flag_ = false;
    }

    //        }
}

/*******************planner state***********************/
void SupplyPid::SetLocalPlannerMaxVel(float max = 1.5)
{
    local_planner_goal_.maxvelx = max;
    local_planner_goal_.maxvely = max;
}

void SupplyPid::SendGoal(geometry_msgs::PoseStamped goal, float maxvel)
{
    SetLocalPlannerMaxVel(maxvel);
    global_planner_goal_.goal = goal;
    global_planner_actionlib_client_.sendGoal(global_planner_goal_,
                                              GlobalActionClient::SimpleDoneCallback(),
                                              GlobalActionClient::SimpleActiveCallback(),
                                              boost::bind(&SupplyPid::GlobalPlannerFeedbackCallback, this, _1));
}

void SupplyPid::GlobalPlannerFeedbackCallback(const GlobalFeedback &feedback)
{
    if (!feedback->path.poses.empty())
    {
        local_planner_goal_.route = feedback->path;
        // Change the local planner's velocity:
        //            local_planner_goal_.delta = 0.03;
        local_planner_actionlib_client_.sendGoal(local_planner_goal_);
    }
}

void SupplyPid::CancelGoal()
{
    if (planner_state_ == PlannerState::RUNNING)
    {
        global_planner_actionlib_client_.cancelGoal();
        local_planner_actionlib_client_.cancelGoal();
        planner_state_ = PlannerState::IDLE;
    }
}

/*******************planner state***********************/

void SupplyPid::CancelWhirl()
{
    geometry_msgs::Twist zero_angle_vel;
    zero_angle_vel.linear.x = 0;
    zero_angle_vel.linear.y = 0;
    zero_angle_vel.angular.z = 0;
    cmd_vel_pub_.publish(zero_angle_vel);
}

void SupplyPid::UpdateActionState()
{
    auto state = global_planner_actionlib_client_.getState();
    if (state == actionlib::SimpleClientGoalState::ACTIVE)
    {
        planner_state_ = PlannerState::RUNNING;
    }
    else if (state == actionlib::SimpleClientGoalState::PENDING)
    {
        planner_state_ = PlannerState::RUNNING;
    }
    else if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        planner_state_ = PlannerState::SUCCESS;
    }
    else if (state == actionlib::SimpleClientGoalState::ABORTED)
    {
        planner_state_ = PlannerState::FAILURE;
    }
    else
    {
        planner_state_ = PlannerState::FAILURE;
    }
}

} // namespace leonard_localization

int main(int argc, char **argv)
{
    leonard_localization::GLogWrapper gLogWrapper(argv[0]);
    ros::init(argc, argv, "supply_node");
    leonard_localization::SupplyNode supply_node("supply_node");
    ros::spin();
    return 0;
}
