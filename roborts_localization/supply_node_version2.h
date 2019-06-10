//
// Created by cxn on 19-2-15.
//

#ifndef PROJECT_LOCALIZATION_NODE_H
#define PROJECT_LOCALIZATION_NODE_H


#include "log.h"
#include "types.h"

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

#include "apriltags2_ros/AprilTagDetectionArray.h"

#include <mutex>
#include <std_msgs/String.h>

#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/ProjectileSupply.h"
#include "roborts_msgs/SupplierStatus.h"
#include "roborts_localization/LocalPlannerAction.h"
#include "roborts_localization/GlobalPlannerAction.h"
#include "roborts_msgs/ForceUpdateAmcl.h"
#include "roborts_localization/SupplyPidAction.h"
#include "roborts_localization/StartVision.h"

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "writetxt.h"


namespace leonard_supply {
    typedef actionlib::SimpleActionClient<roborts_localization::LocalPlannerAction> LocalActionClient;
    typedef actionlib::SimpleActionClient<roborts_localization::GlobalPlannerAction> GlobalActionClient;
    typedef roborts_localization::GlobalPlannerFeedbackConstPtr GlobalFeedback;
    typedef roborts_localization::LocalPlannerGoal LocalGoal;
    typedef roborts_localization::GlobalPlannerGoal GlobalGoal;
    enum class PlannerState {
        RUNNING,
        SUCCESS,
        FAILURE,
        IDLE,
    };

    enum class ProcessState {
        NOREACH,
        PID,
        BLINDERROR,
        LOSTPOSEERROR,
        REACH,
        WAITSUPPLY,
    };

    class SupplyPid {
    public:
        SupplyPid(std::string name);

        ~SupplyPid() = default;

    private:
        //执行函数,里面就是一个状态机
        void executeCB(const roborts_localization::SupplyPidGoalConstPtr &goal);

        //视觉检测,里面会进行pid
        void TagCallback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr &msg);

        //用于启用或者停止april_tag2节点检测程序
        void DetectedTagSwitch(bool isAllowed);


        void CallForceUpdate();


        //更改目标标签id,1or2(红车看1,蓝车看2)
        void RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &msg);

        //pid
        void PidProcess(const apriltags2_ros::AprilTagDetectionArray::ConstPtr &msg);

        //补给站信息回调
        void SupplierStatusCallback(const roborts_msgs::SupplierStatus::ConstPtr &msg);

        //重置一些标志位
        void Reset();

        //发送规划点
        void SetLocalPlannerMaxVel(float max);

        void SendGoal(geometry_msgs::PoseStamped goal, float maxvel = 3.0);

        //全局规划回调
        void GlobalPlannerFeedbackCallback(const GlobalFeedback &feedback);

        void CancelGoal();

        void UpdateActionState();

        void CancelWhirl();

        void RestAndProcessState(ProcessState state);

    private:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<roborts_localization::SupplyPidAction> as_;
        std::string action_name_;
        roborts_localization::SupplyPidFeedback feedback_;
        roborts_localization::SupplyPidResult result_;

        ros::Subscriber tag_detections_sub_;
        ros::Subscriber supplier_status_sub_;
        ros::Subscriber robot_status_sub_;

        ros::Publisher cmd_vel_pub_;
        ros::Publisher supply_cmd_pub_;
        ros::Publisher modify_pose_pub_;

        ros::ServiceClient start_vision_client_;
        ros::ServiceClient relocate_client_;


        LocalActionClient local_planner_actionlib_client_;
        GlobalActionClient global_planner_actionlib_client_;
        GlobalGoal global_planner_goal_;
        LocalGoal local_planner_goal_;

        //从ros服务器中读取
        Vec3d target_pose_;
        Vec3d pid_p_;
        Vec3d tolerance_dis_;
        double max_supply_speed_;
        Vec3d target_pose_map_;
        double target_pose_map_x_2_;
        double target_pose_map_x_3_;
        double target_pose_map_x_4_;
        double target_pose_map_x_5_;
        double target_pose_map_y_2_;
        double target_pose_map_y_3_;
        double target_pose_map_y_4_;
        double target_pose_map_y_5_;


        //补给站状态信息,[0]为当前,[1]为上一次
        uint8_t supply_status_[2];
        ros::Time supply_done_time_;
        bool supply_done_flag_;
        uint8_t robot_id_;

        uint8_t detected_tag_id_;
        uint8_t target_tag_id_ = 1;


        std::mutex variable_lock_;
        int64_t undetected_cnt_ = 0;
        int reach_cnt_ = 0;
        ros::Time wait_timestamp_;
        PlannerState planner_state_ = PlannerState::IDLE;
        ProcessState process_state_ = ProcessState::NOREACH;
        uint8_t blind_cnt_ = 0;
        uint8_t blind_delay_ = 0;
    };


    class SupplyNodeVersion2 {
    public:
        SupplyNodeVersion2(std::string name);

        bool Init();

    private:
        ros::NodeHandle nh_;
        std::shared_ptr<SupplyPid> supplypid_ptr_;
    };
}

#endif //PROJECT_LOCALIZATION_NODE_H
