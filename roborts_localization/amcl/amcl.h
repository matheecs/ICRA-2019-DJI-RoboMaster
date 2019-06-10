//
// Created by cxn on 19-2-15.
//

#ifndef PROJECT_AMCL_H
#define PROJECT_AMCL_H

#include <ros/ros.h>
#include "log.h"
#include "types.h"
#include <nav_msgs/OccupancyGrid.h>
#include "sensors/sensor_laser.h"
#include "sensors/sensor_odom.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <mutex>
#include "localization_math.h"
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

namespace leonard_localization {
    /**
     * @brief Pose hypothesis type in Amcl
     */
    class AmclHyp {
    public:
        //! Total weight (weights sum to 1)
        double weight;
        //! Mean of pose esimate
        Vec3d pf_pose_mean;
        //! Covariance of pose estimate
        Mat3d pf_pose_cov;
    };

    /**
     * @brief Pose hypothesis type in localization module
     */
    class HypPose {
    public:
        //! Mean of pose esimate
        Vec3d pose_mean;
        //! Covariance of pose estimate
        Mat3d pose_set_cov;
    };

    class Amcl {
    public:
        void GetParamFromRos(ros::NodeHandle *nh);

        /**
          * @brief Amcl initialization function
          */
        void Init(const Vec3d &init_pose, const Vec3d &init_cov);

        /**
         * @brief Map message handler
         * @param map_msg Static Map message
         */
        void HandleMapMessage(const nav_msgs::OccupancyGrid &map_msg,
                              const Vec3d &init_pose, const Vec3d &init_cov);

        /**
         * @brief Set up laser pose in base frame
         * @param laser_pose Laser pose in base frame
         */
        void SetLaserSensorPose(Vec3d laser_pose);


        /**
         * @brief Initial pose estimation message handler
         * @param pf_init_pose_mean Initial mean of pose estimation
         * @param pf_init_pose_cov Initial covariance of pose estimation
         */
        void HandleInitialPoseMessage(Vec3d pf_init_pose_mean,
                                      Mat3d pf_init_pose_cov);


        /**
         * @brief Update AMCL algorithm
         * @param pose Odom pose
         * @param laser_scan Laser_scan msg
         * @param angle_min Laser scan msg angle min in base frame
         * @param angle_increment Laser scan msg angle increment in base frame
         * @param particle_cloud_pose_msg Particle cloud pose msg to publish
         * @param hyp_pose Pose hypothesis to publish
         * @return Error code
         */
        int Update(const Vec3d &pose,
                   const sensor_msgs::LaserScan &laser_scan,
                   const double &angle_min,
                   const double &angle_increment,
                   geometry_msgs::PoseArray &particle_cloud_pose_msg,
                   HypPose &hyp_pose
        );

        /**
         * @brief Check TF update state
         * @return Return true if need update TF
         */
        bool CheckTfUpdate();

        const nav_msgs::OccupancyGrid &GetDistanceMapMsg();

        /***********test code**************/
        int NoUpdate(const Vec3d &pose,
                     const sensor_msgs::LaserScan &laser_scan,
                     const double &angle_min,
                     const double &angle_increment,
                     geometry_msgs::PoseArray &particle_cloud_pose_msg,
                     HypPose &hyp_pose);

        void SetForceUpdateMode(uint8_t forceUpdateMode);

    private:
        Vec3d UniformPoseGenerator();

        void UpdatePoseFromParam(const Vec3d &init_pose, const Vec3d &init_cov);

        void Reset();

        bool GlobalLocalization();

        // 初始角度随机
        bool RandomHeadingGlobalLocalization();

        // 初始角度固定
        bool FixedHeadingGlobalLocalization();

        void ApplyInitPose();


    private:
        Vec3d init_pose_;
        Vec3d init_cov_;

        /*原来AmclConfig 中的参数*/
//       bool use_map_topic_;

        double laser_min_range_;
        double laser_max_range_;
        int laser_max_beams_;

        int min_particles_;
        int max_particles_;

        double kld_err_;
        double kld_z_;

//  LaserModel laser_model = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
        double z_hit_;
        double z_rand_;
        double sigma_hit_;
        double laser_likelihood_max_dist_;
        bool do_beamskip_;
        double beam_skip_distance_;
        double beam_skip_threshold_;
        double beam_skip_error_threshold_;

//  OdomModel odom_model = ODOM_MODEL_OMNI;
        double odom_alpha1_;
        double odom_alpha2_;
        double odom_alpha3_;
        double odom_alpha4_;
        double odom_alpha5_;

        double update_min_d_;//d_thresh_
        double update_min_a_;//a_thresh_

        int resample_interval_;
        double recovery_alpha_slow_;
        double recovery_alpha_fast_;

        bool use_global_localization_;
        bool random_heading_;
        /*原来AmclConfig 中的参数*/


        //每次update函数最后置为false，若里程计测到的位移大于阈值，就不会专门置为true，
        bool laser_update_ = true;

        bool resampled_ = false;

        std::unique_ptr<SensorLaser> laser_model_ptr_;
        std::unique_ptr<SensorOdom> odom_model_ptr_;

        std::shared_ptr<AmclMap> map_ptr_;

        ParticleFilterPtr pf_ptr_;
        bool pf_init_ = true;

        static std::vector<std::pair<int, int> > free_space_indices;
        // 把map中free的cell的坐标放进来，
        // 2.17 好像随机生成粒子用，还不确定


        std::unique_ptr<AmclHyp> initial_pose_hyp_;

        std::mutex mutex_;

        //Publish flag
        bool publish_particle_pose_cloud_ = false;
        bool publish_pose_ = false;
        bool update_tf_ = false;

        Vec3d pf_odom_pose_;
        //updateLaser中得到的base_link在odom中的坐标，每次update函数调用都会更新

        bool force_publication_ = false;

        int resample_count_ = 0;

        uint8_t forceUpdateMode_ = 0;
        std::mutex forceUpdateMutex_;
        bool set_maxBeams_flag_ = false;
        ros::Time last_laser_update_time_;

    };
}


#endif //PROJECT_AMCL_H
