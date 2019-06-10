//
// Created by cxn on 19-5-5.
//

#include "turn_angle_action.h"

namespace leonard_serial_common {
/*************************** TurnAngleAction ****************************/

    TurnAngleAction::TurnAngleAction(std::string name, double *base_yaw) : as_(nh_, name,
                                                                               boost::bind(
                                                                                       &TurnAngleAction::executeCB,
                                                                                       this, _1),
                                                                               false),
                                                                           action_name_(name) {
        as_.start();
        base_yaw_ptr_ = base_yaw;
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        whirl_vel_.angular.z = 0;
        whirl_vel_.linear.x = whirl_vel_.linear.y = 0;
    }

    void TurnAngleAction::executeCB(const roborts_base::TurnAngleGoalConstPtr &goal) {

        // helper variables
        ros::Rate r(20); //50ms,回复一次
        bool success = true;
        // std::cout << "goalangle" << goal->angle << std::endl;

        // Wrapping angle to [-pi .. pi]
        // std::fmod 返回浮点余数 std::fmod(18.5,4.2)=1.7
        target_yaw_ = std::fmod(*base_yaw_ptr_ + goal->angle + 5 * M_PI, 2 * M_PI) - M_PI;
        stop_cnt_ = 0;

        while (1) {
            //被打断
            if (as_.isPreemptRequested() || !ros::ok()) {
                as_.setPreempted();
                whirl_vel_.angular.z = 0;
                cmd_vel_pub_.publish(whirl_vel_);
                success = false;
                // std::cout << "turnangle ispreempt" << std::endl;
                break;
            }

            double err = angle_diff<double>(target_yaw_, *base_yaw_ptr_);
            feedback_.angle_feedback = err;
            as_.publishFeedback(feedback_);

            if (fabs(err) < 0.1 && (++stop_cnt_ > 2)) {
                whirl_vel_.angular.z = 0;
                cmd_vel_pub_.publish(whirl_vel_);
                break;
            }
            whirl_vel_.angular.z = err * 5;
            VAL_LIMIT(whirl_vel_.angular.z, -5.0, 5.0);
            cmd_vel_pub_.publish(whirl_vel_);
            // std::cout << "whsped" << std::endl;
            r.sleep();
        }

        if (success) {
            // std::cout << "turnangle yes!" << std::endl;
            result_.result = true;
            as_.setSucceeded(result_);
        }
    }
}