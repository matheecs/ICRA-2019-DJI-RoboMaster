//
// Created by cxn on 19-5-5.
//

#ifndef PROJECT_TURNANGLEACTION_H
#define PROJECT_TURNANGLEACTION_H

#include <ros/ros.h>
#include "roborts_base/TurnAngleAction.h"
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>

namespace leonard_serial_common {
    class TurnAngleAction {
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<roborts_base::TurnAngleAction> as_;
        std::string action_name_;
        roborts_base::TurnAngleFeedback feedback_;
        roborts_base::TurnAngleResult result_;
        double *base_yaw_ptr_;
        double target_yaw_;
        geometry_msgs::Twist whirl_vel_;
        uint8_t stop_cnt_ = 0;

#define VAL_LIMIT(val, min, max) \
    if ((val) <= (min))              \
    {                                \
      (val) = (min);                 \
    }                                \
    else if ((val) >= (max))         \
    {                                \
      (val) = (max);                 \
    }                                \


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

        ros::Publisher cmd_vel_pub_;

    public:
        TurnAngleAction(std::string name, double *base_yaw);

        ~TurnAngleAction() = default;

        void executeCB(const roborts_base::TurnAngleGoalConstPtr &goal);
    };
}
#endif //PROJECT_TURNANGLEACTION_H
