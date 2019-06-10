#ifndef MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_TREE_H
#define MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_TREE_H

#include <chrono>

#include <ros/ros.h>

#include "behavior_node.h"

namespace decision {
    class BehaviorTree {
    public:
        BehaviorTree(BehaviorNode::Ptr root_node, int cycle_duration) :
                root_node_(root_node),
                cycle_duration_(cycle_duration),
                running_(false) {}

        void Execute() {
            running_ = true;
            unsigned int frame = 0;
            while (ros::ok() && running_) {

                std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
                ros::spinOnce();
                // LOG_INFO<<"Frame"<<frame;
                root_node_->Run();

                std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
                std::chrono::milliseconds execution_duration =
                        std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                std::chrono::milliseconds sleep_time = cycle_duration_ - execution_duration;

                if (sleep_time > std::chrono::microseconds(0)) {
                    std::this_thread::sleep_for(sleep_time);
                    //LOG_INFO << "sleep: " << sleep_time.count() << "ms";
                } else {
                    // LOG_WARNING << "The time tick once is " << execution_duration.count() << " beyond the expected time "
                    //           << cycle_duration_.count();
                }

                // LOG_INFO("----------------------------------");
                frame++;
            }
        }

    private:
        BehaviorNode::Ptr root_node_;
        std::chrono::milliseconds cycle_duration_;
        bool running_;
    };

}//namespace decision

#endif //MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_TREE_H
