//
// Created by cxn on 19-3-17.
//
#include "log.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "roborts_localization/SupplyPidAction.h"


class testActionClient {
public:
    testActionClient() : ac("supplypid", true) {
        // create the action client
        // true causes the client to spin its own thread
//        ac("fibonacci", true);
        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac.waitForServer(); //will wait for infinite time

        ROS_INFO("Action server started, sending goal.");
        // send a goal to the action
        roborts_localization::SupplyPidGoal goal;
        goal.command = 1;
        ac.sendGoal(goal);

        ros::Rate rate(2);
        while (1) {
            auto state = ac.getState();
            if (state == actionlib::SimpleClientGoalState::ACTIVE) {
                ROS_INFO("ACTIVE");
            } else if (state == actionlib::SimpleClientGoalState::PENDING) {
                ROS_INFO("PENDING");
            } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("SUCCESS");
                break;
            } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                ROS_INFO("FAILURE");
                break;
            }

            rate.sleep();
        }
    }



    actionlib::SimpleActionClient<roborts_localization::SupplyPidAction> ac;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "test_fibonacci");
    testActionClient cxn;
    //exit
    ros::waitForShutdown();
    return 0;
}
