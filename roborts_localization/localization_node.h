//
// Created by cxn on 19-2-15.
//

#ifndef PROJECT_LOCALIZATION_NODE_H
#define PROJECT_LOCALIZATION_NODE_H


#include "log.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "types.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include "amcl/amcl.h"
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include "writetxt.h"
#include "roborts_msgs/ForceUpdateAmcl.h"
#include "apriltags2_ros/AprilTagDetectionArray.h"

#define THREAD_NUM 4 // ROS SPIN THREAD NUM origin：4

namespace leonard_localization {
    class LocalizationNode {
    public:
        /**
        * @brief Localization Node construct function
        * @param name Node name
        */
        LocalizationNode(std::string name);

        /**
         * @brief Localization initialization
         * @return Returns true if initialize success
         */
        bool Init();

        /**
         * @brief Laser scan messages callback function (as main loop in localization node)
         * @param laser_scan_msg Laser scan data
         */
        void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg);

        /**
          * @brief Manually initialize localization init_pose
          * @param init_pose_msg Init pose (2D Pose Estimate button in RViz)
          */
        void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &init_pose_msg);


        /**
         * @brief Publish visualize messages
         */
        void PublishVisualize();

        /**
         * @brief Publish transform information between odom_frame and global_frame
         * @return True if no errors
         */
        bool PublishTf();


    private:
        bool GetPoseFromTf(const std::string &target_frame,
                           const std::string &source_frame,
                           const ros::Time &timestamp,
                           Vec3d &pose);

        bool GetStaticMap();

        bool GetLaserPose();

        void TransformLaserscanToBaseFrame(double &angle_min,
                                           double &angle_increment,
                                           const sensor_msgs::LaserScan &laser_scan_msg);

    private:

        //ROS Node handle
        ros::NodeHandle nh_;

        //Algorithm object
        std::unique_ptr<Amcl> amcl_ptr_;

        //Parameters
        std::string odom_frame_;
        std::string global_frame_;
        std::string base_frame_;
        std::string laser_topic_;
        Vec3d init_pose_;
        Vec3d init_cov_;
        ros::Duration transform_tolerance_;
        bool publish_visualize_;

        //TF
        std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_ptr_;
        std::unique_ptr<tf::TransformListener> tf_listener_ptr_;

        //ROS Subscriber
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laser_scan_sub_;
        std::unique_ptr<tf::MessageFilter<sensor_msgs::LaserScan>> laser_scan_filter_;
        ros::Subscriber initial_pose_sub_;

        //ROS Publisher
        ros::Publisher pose_pub_;
        ros::Publisher particlecloud_pub_;
        ros::Publisher distance_map_pub_;//会将AmclMap算出来的地图发出来，就一次

        //ROS Service
        ros::ServiceClient static_map_srv_;

        //Status
        bool initialized_ = false;
        bool map_init_ = false;//地图处理好了后，置为true
        bool laser_init_ = false;//等到第一帧laserScan处理好后，置为true
        bool publish_first_distance_map_ = false;//distance_map_pub_发送一次距离地图后，置为true，不再发
        bool latest_tf_valid_ = false;//amcl更新一次预测坐标后，置为true

        //Data
        ros::Time last_laser_msg_timestamp_;
        HypPose hyp_pose_;
        geometry_msgs::PoseArray particlecloud_msg_;
        geometry_msgs::PoseStamped pose_msg_;
        tf::Transform latest_tf_;


        //! ros service server for gimbal mode set
        ros::ServiceServer force_update_mode_srv_;
        bool SetResamplelModeService(roborts_msgs::ForceUpdateAmcl::Request &req,
                                                       roborts_msgs::ForceUpdateAmcl::Response &res);

    };
}

#endif //PROJECT_LOCALIZATION_NODE_H
