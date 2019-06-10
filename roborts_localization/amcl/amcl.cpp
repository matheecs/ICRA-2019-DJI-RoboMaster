//
// Created by cxn on 19-2-15.
//

#include "amcl.h"

namespace leonard_localization {
    std::vector<std::pair<int, int>> Amcl::free_space_indices;

    void Amcl::GetParamFromRos(ros::NodeHandle *nh) {
        //        nh->param<bool>("localization/use_map_topic", use_map_topic_, true);
        //        //当设置为true时，AMCL将仅仅使用订阅的第一个地图，而不是每次接收到新的时更新为一个新的地图
        nh->param<double>("localization/laser_min_range", laser_min_range_, 0.15);
        nh->param<double>("localization/laser_max_range", laser_max_range_, 8.0);
        nh->param<int>("localization/laser_max_beams", laser_max_beams_, 30);
        nh->param<int>("localization/min_particles", min_particles_, 500);
        nh->param<int>("localization/max_particles", max_particles_, 5000);
        nh->param<double>("localization/kld_err", kld_err_, 0.05); //原书中的erfa
        nh->param<double>("localization/kld_z", kld_z_, 0.99);     //原书中的z1-theta
        nh->param<double>("localization/z_hit", z_hit_, 0.5);
        nh->param<double>("localization/z_rand", z_rand_, 0.5);
        nh->param<double>("localization/sigma_hit", sigma_hit_, 0.2);
        nh->param<double>("localization/laser_likelihood_max_dist", laser_likelihood_max_dist_, 2.0);
        nh->param<bool>("localization/do_beamskip", do_beamskip_, true); //beamskip是啥？
        nh->param<double>("localization/beam_skip_distance", beam_skip_distance_, 0.5);
        nh->param<double>("localization/beam_skip_threshold", beam_skip_threshold_, 0.3);
        nh->param<double>("localization/beam_skip_error_threshold", beam_skip_error_threshold_, 0.9);
        nh->param<double>("localization/odom_alpha1", odom_alpha1_, 0.005);
        nh->param<double>("localization/odom_alpha2", odom_alpha2_, 0.005);
        nh->param<double>("localization/odom_alpha3", odom_alpha3_, 0.01);
        nh->param<double>("localization/odom_alpha4", odom_alpha4_, 0.005);
        nh->param<double>("localization/odom_alpha5", odom_alpha5_, 0.003);
        nh->param<double>("localization/update_min_d", update_min_d_, 0.2); //dji官方给值0.05(50mm)与0.03（1.7度）
        // 在执行滤波更新前平移运动的距离
        // (对于里程计模型有影响，模型中根据运动和地图求最终位姿的似然时丢弃了路径中的相关所有信息，
        // 已知的只有最终位姿，为了规避不合理的穿过障碍物后的非零似然，这个值建议不大于机器人半径。
        // 否则因更新频率的不同可能产生完全不同的结果)       网上摘的，未确定

        nh->param<double>("localization/update_min_a", update_min_a_, 0.5);
        nh->param<int>("localization/resample_interval", resample_interval_, 1); //重采样前需要滤波的次数,冲采样间隔
        nh->param<double>("localization/recovery_alpha_slow", recovery_alpha_slow_, 0.1);
        nh->param<double>("localization/recovery_alpha_fast", recovery_alpha_fast_, 0.001);
        nh->param<bool>("localization/use_global_localization", use_global_localization_, false);
        // 1.28：如果是true，那么粒子在初始化时，位姿会在整个地图使用均匀分布采样

        nh->param<bool>("localization/random_heading", random_heading_, false);
        // 1.28若为true，则在依照localization_config文件配置的初始位姿附近配置粒子

        CHECK_GT(laser_likelihood_max_dist_, 0);
    }

    void Amcl::SetForceUpdateMode(uint8_t forceUpdateMode) {
        // lock_guard析构时，解锁
        std::lock_guard<std::mutex> sample_lock(forceUpdateMutex_);
        forceUpdateMode_ = forceUpdateMode;
    }

    void Amcl::Init(const Vec3d &init_pose, const Vec3d &init_cov) {
        UpdatePoseFromParam(init_pose, init_cov);
    }

// 这个函数做了以下这些事：
// 将地图信息转换过成AmclMap类进行存储
// 标注地图中free的网格（方便生成全局随机粒子）
// 初始化pf，最少粒子，最大粒子，短期似然，长期似然，生成随机粒子函数，AmclMap指针，kld的两个参数
// 初始化init_pose_,init_cov_
// 全局初始化粒子or根据init_pose_,init_cov_高斯分布初始化粒子。初始化后会更新粒子滤波器（kd树插点，统计簇）
// 初始化传感器模型
// 若有initial_pose_hyp_，则根据其高斯分布重新初始化粒子
    void Amcl::HandleMapMessage(const nav_msgs::OccupancyGrid &map_msg,
                                const Vec3d &init_pose, const Vec3d &init_cov) {
        Reset();
        map_ptr_.reset(new AmclMap());
        map_ptr_->ConvertFromMsg(map_msg);

        // Index of free space
        Amcl::free_space_indices.resize(0);
        for (int i = 0; i < map_ptr_->GetSizeX(); i++) {
            for (int j = 0; j < map_ptr_->GetSizeY(); j++) {
                if (map_ptr_->CheckIndexFree(i, j)) {
                    this->free_space_indices.push_back(std::make_pair(i, j));
                }
            }
        }

        pf_ptr_.reset(new ParticleFilter(min_particles_,
                                         max_particles_,
                                         recovery_alpha_slow_,
                                         recovery_alpha_fast_,
                                         std::bind(&Amcl::UniformPoseGenerator, this),
                                         map_ptr_));
        pf_ptr_->SetKldParam(kld_err_, kld_z_);

        UpdatePoseFromParam(init_pose, init_cov);

        if (use_global_localization_) {
            GlobalLocalization();
        } else if (random_heading_) {
            RandomHeadingGlobalLocalization();
        } else {
            FixedHeadingGlobalLocalization();
        }

        odom_model_ptr_ = std::make_unique<SensorOdom>(odom_alpha1_,
                                                       odom_alpha2_,
                                                       odom_alpha3_,
                                                       odom_alpha4_,
                                                       odom_alpha5_);

        laser_model_ptr_ = std::make_unique<SensorLaser>(laser_max_beams_,
                                                         map_ptr_);
        LOG_INFO << "Initializing likelihood field model( this can take some time on large maps)";

        laser_model_ptr_->SetModelLikelihoodFieldProb(z_hit_,
                                                      z_rand_,
                                                      sigma_hit_,
                                                      laser_likelihood_max_dist_,
                                                      do_beamskip_,
                                                      beam_skip_distance_,
                                                      beam_skip_threshold_,
                                                      beam_skip_error_threshold_);
        LOG_INFO << "Done initializing likelihood field model.";

        ApplyInitPose();
    }

    void Amcl::Reset() {
        if (map_ptr_ != nullptr) {
            map_ptr_.reset();
        }
        if (pf_ptr_ != nullptr) {
            pf_ptr_.reset();
        }
    }

    void Amcl::SetLaserSensorPose(Vec3d laser_pose) {
        laser_update_ = true;
        laser_model_ptr_->SetLaserPose(laser_pose);
    }

    Vec3d Amcl::UniformPoseGenerator() {
        //drand48()会生成均匀分布在[0～1]的双浮点数字
        auto rand_index = static_cast<unsigned int>(drand48() * free_space_indices.size());
        std::pair<int, int> free_point = free_space_indices[rand_index];
        Vec3d p;
        map_ptr_->ConvertMapCoordsToWorldCoords(free_point.first, free_point.second, p[0], p[1]);
        p[2] = drand48() * 2 * M_PI - M_PI;
        return p;
    }

    void Amcl::UpdatePoseFromParam(const Vec3d &init_pose, const Vec3d &init_cov) {
        init_pose_[0] = 0.0;
        init_pose_[1] = 0.0;
        init_pose_[2] = 0.0;
        init_cov_[0] = 0.5 * 0.5;
        init_cov_[1] = 0.5 * 0.5;
        init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);

        if (!std::isnan(init_pose[0])) {
            init_pose_[0] = init_pose[0];
        } else {
            LOG_WARNING << "ignoring NAN in initial pose X position";
        }
        if (!std::isnan(init_pose[1])) {
            init_pose_[1] = init_pose[1];
        } else {
            LOG_WARNING << "ignoring NAN in initial pose Y position";
        }
        if (!std::isnan(init_pose[2])) {
            init_pose_[2] = init_pose[2];
        } else {
            LOG_WARNING << "ignoring NAN in initial pose Yaw";
        }
        if (!std::isnan(init_cov[0])) {
            init_cov_[0] = init_cov[0];
        } else {
            LOG_WARNING << "ignoring NAN in initial covariance XX";
        }
        if (!std::isnan(init_cov[1])) {
            init_cov_[1] = init_cov[1];
        } else {
            LOG_WARNING << "ignoring NAN in initial covariance YY";
        }
        if (!std::isnan(init_cov[2])) {
            init_cov_[2] = init_cov[2];
        } else {
            LOG_WARNING << "ignoring NAN in initial covariance AA";
        }

        DLOG_INFO << "Updated init pose " << init_pose_;
    }

    bool Amcl::GlobalLocalization() {
        if (map_ptr_ == nullptr) {
            return true;
        }

        LOG_INFO << "Initializing with uniform distribution";
        PfInitModelFunc UniformPoseGeneratorFunc = std::bind(&Amcl::UniformPoseGenerator, this);
        pf_ptr_->InitByModel(UniformPoseGeneratorFunc);

        LOG_INFO << "Global initialization done!";
        pf_init_ = false;
        return true;
    }

    bool Amcl::FixedHeadingGlobalLocalization() {
        if (map_ptr_ != nullptr) {
            Mat3d pf_init_pose_cov;
            pf_init_pose_cov.setZero();
            pf_init_pose_cov(0, 0) = init_cov_(0);
            pf_init_pose_cov(1, 1) = init_cov_(1);
            pf_init_pose_cov(2, 2) = init_cov_(2);

            pf_ptr_->InitByGuassian(init_pose_, pf_init_pose_cov);
            pf_init_ = false;
            return true;
        }
        return false;
    }

    bool Amcl::RandomHeadingGlobalLocalization() {
        if (map_ptr_ != nullptr) {
            Mat3d pf_init_pose_cov;
            pf_init_pose_cov.setZero();
            pf_init_pose_cov(0, 0) = init_cov_(0);
            pf_init_pose_cov(1, 1) = init_cov_(1);
            pf_init_pose_cov(2, 2) = init_cov_(2);

            pf_ptr_->InitByGuassianWithRandomHeading(init_pose_, pf_init_pose_cov);
            pf_init_ = false;
            return true;
        }
        return false;
    }

    void Amcl::ApplyInitPose() {
        if (initial_pose_hyp_ != nullptr && map_ptr_ != nullptr) {
            pf_ptr_->InitByGuassian(initial_pose_hyp_->pf_pose_mean,
                                    initial_pose_hyp_->pf_pose_cov);
            pf_init_ = false;
            initial_pose_hyp_.reset();
        }
    }

    void Amcl::HandleInitialPoseMessage(Vec3d pf_init_pose_mean,
                                        Mat3d pf_init_pose_cov) {
        initial_pose_hyp_.reset(new AmclHyp());
        initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
        initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
        ApplyInitPose();
    }

    int Amcl::NoUpdate(const Vec3d &pose,
                       const sensor_msgs::LaserScan &laser_scan,
                       const double &angle_min,
                       const double &angle_increment,
                       geometry_msgs::PoseArray &particle_cloud_pose_msg,
                       HypPose &hyp_pose) {
        auto set_ptr = pf_ptr_->GetCurrentSampleSetPtr();
        particle_cloud_pose_msg.poses.resize(set_ptr->sample_count);

        Vec3d poseMean;
        Vec3d poseCov;

        for (int i = 0; i < set_ptr->sample_count; i++) {
            tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set_ptr->samples_vec[i].pose[2]),
                                     tf::Vector3(set_ptr->samples_vec[i].pose[0],
                                                 set_ptr->samples_vec[i].pose[1],
                                                 0)),
                            particle_cloud_pose_msg.poses[i]);
            poseMean(0) += set_ptr->samples_vec[i].pose[0];
            poseMean(1) += set_ptr->samples_vec[i].pose[1];
            //            poseMean(2)+=set_ptr->samples_vec[i].pose[2];
        }
        poseMean(0) /= set_ptr->sample_count;
        poseMean(1) /= set_ptr->sample_count;
        //        poseMean(2)/=set_ptr->sample_count;

        for (int i = 0; i < set_ptr->sample_count; i++) {
            poseCov(0) +=
                    (set_ptr->samples_vec[i].pose[0] - poseMean(0)) * (set_ptr->samples_vec[i].pose[0] - poseMean(0));
            poseCov(1) +=
                    (set_ptr->samples_vec[i].pose[1] - poseMean(1)) * (set_ptr->samples_vec[i].pose[1] - poseMean(1));
            //            poseMean(2)+=set_ptr->samples_vec[i].pose[2];
        }
        poseCov(0) = std::sqrt(poseCov(0) / set_ptr->sample_count);
        poseCov(1) = std::sqrt(poseCov(1) / set_ptr->sample_count);

        std::cout << poseMean(0) << "," << poseMean(1) << std::endl;
        std::cout << poseCov(0) << "," << poseCov(1) << std::endl;

        return 1;
    }

    int Amcl::Update(const Vec3d &pose,
                     const sensor_msgs::LaserScan &laser_scan,
                     const double &angle_min,
                     const double &angle_increment,
                     geometry_msgs::PoseArray &particle_cloud_pose_msg,
                     HypPose &hyp_pose) {

        // lock_guard析构时，解锁
        std::lock_guard<std::mutex> sample_lock(mutex_);

        publish_pose_ = false;
        publish_particle_pose_cloud_ = false;
        update_tf_ = false;

        /** ↓原来void Amcl::UpdateOdomPoseData(leonard_localization::Vec3d pose)中的内容↓ **/
        Vec3d delta;
        delta.setZero();

        if (pf_init_) {
            // Compute change in pose
            delta[0] = pose[0] - pf_odom_pose_[0]; //上次更新时，
            delta[1] = pose[1] - pf_odom_pose_[1];
            delta[2] = angle_diff<double>(pose[2], pf_odom_pose_[2]);

            // See if we should update the filter
            bool update = std::fabs(delta[0]) > update_min_d_ ||
                          std::fabs(delta[1]) > update_min_d_ ||
                          std::fabs(delta[2]) > update_min_a_;

            // Set the laser update flags
            if (update) {
                laser_update_ = true;
            }
        }

        force_publication_ = false;

        if (!pf_init_) {
            // Pose at last filter update
            pf_odom_pose_ = pose;
            // Filter is now initialized"
            pf_init_ = true;
            // Set update sensor data flag
            laser_update_ = true;
            force_publication_ = true;
            resample_count_ = 0;
        } // If the robot has moved, update the filter
        else if (pf_init_ && laser_update_) {
            DLOG_INFO << "Robot has moved, update the filter";
            SensorOdomData odom_data;
            odom_data.pose = pose;
            odom_data.delta = delta;
            odom_model_ptr_->UpdateAction(pf_ptr_->GetCurrentSampleSetPtr(), odom_data);
        }
        /** ↑原来void Amcl::UpdateOdomPoseData(leonard_localization::Vec3d pose)中的内容↑ **/

        {
            //        forceUpdateMutex_.lock();
            std::lock_guard<std::mutex> guard(forceUpdateMutex_);
            if (forceUpdateMode_ == 1) {
                laser_update_ = true;
                // 在预测值附近半径30cm×半径30cm×±?度，撒粒子
                Vec3d mean;
                mean(0) = hyp_pose.pose_mean(0);
                mean(1) = hyp_pose.pose_mean(1);
                mean(2) = hyp_pose.pose_mean(2);
                Mat3d cov;
                cov.setZero();
                cov(0, 0) = 0.5;
                cov(1, 1) = 0.5;
                cov(2, 2) = 0.2;

                pf_ptr_->RandomByFixedPose(mean, cov);

                laser_model_ptr_->SensorMaxbeams(180);
                set_maxBeams_flag_ = true;
            } else if (forceUpdateMode_ == 0) {
                if (set_maxBeams_flag_ == true) {
                    laser_model_ptr_->SensorMaxbeams(laser_max_beams_);
                    set_maxBeams_flag_ = false;
                }
            } else if (forceUpdateMode_ == 2) {
                laser_update_ = true;
                // 在预测值附近半径30cm×半径30cm×±?度，撒粒子
                Vec3d mean;
                mean(0) = hyp_pose.pose_mean(0);
                mean(1) = hyp_pose.pose_mean(1);
                mean(2) = hyp_pose.pose_mean(2);
                Mat3d cov;
                cov.setZero();
                cov(0, 0) = 0.3;
                cov(1, 1) = 0.3;
                cov(2, 2) = 0.2;

                pf_ptr_->RandomByFixedPose(mean, cov);

                laser_model_ptr_->SensorMaxbeams(180);
                set_maxBeams_flag_ = true;
                forceUpdateMode_ = 0; //就一次
            }

            //如果没有更新，但是距离上一次更新已经过了5s，那么也更新
            if (forceUpdateMode_ == 0 &&
                (ros::Time().now() - last_laser_update_time_ > ros::Duration(5))) {
                laser_update_ = true;
                // 在预测值附近半径30cm×半径30cm×±?度，撒粒子
                Vec3d mean;
                mean(0) = hyp_pose.pose_mean(0);
                mean(1) = hyp_pose.pose_mean(1);
                mean(2) = hyp_pose.pose_mean(2);
                Mat3d cov;
                cov.setZero();
                cov(0, 0) = 0.3;
                cov(1, 1) = 0.3;
                cov(2, 2) = 0.2;

                pf_ptr_->RandomByFixedPose(mean, cov);

                laser_model_ptr_->SensorMaxbeams(180);
                set_maxBeams_flag_ = true;
            }
            //forceUpdateMutex_.unlock();
        }

        if (laser_update_) {
            // 若里程计测算到的位姿偏离上次pf得到的位姿一个阈值，就进入该条件
            // pose是bask-link在里程计坐标系中坐标（是从TF树中获取的），
            // particle_cloud_pose_msg是base-link在地图坐标系中坐标（是从下面的函数中获取的）

            /** ↓原来UpdateLaser(const sensor_msgs::LaserScan &laser_scan,
                           double angle_min,
                           double angle_increment,
                           const Vec3d &pose,
                           geometry_msgs::PoseArray &particle_cloud_pose_msg)中的内容↓ **/
            SensorLaserData laser_data;
            laser_data.range_count = laser_scan.ranges.size();

            // Apply range min/max thresholds, if the user supplied them
            if (laser_max_range_ > 0.0) {
                laser_data.range_max = std::min(laser_scan.range_max,
                                                static_cast<float>(laser_max_range_));
            } else {
                laser_data.range_max = laser_scan.range_max;
            }

            double range_min;
            if (laser_min_range_ > 0.0) {
                range_min = std::max(laser_scan.range_min, static_cast<float>(laser_min_range_));
            } else {
                range_min = laser_scan.range_min;
            }

            //            laser_data.ranges_mat.resize(laser_data.range_count, 2);
            //            laser_data.ranges_mat.setZero();
            //
            //            for (int i = 0; i < laser_data.range_count; i++) {
            //                // amcl doesn't (yet) have a concept of min range.  So we'll map short
            //                // readings to max range.
            //                if (laser_scan.ranges[i] <= range_min) {
            //                    laser_data.ranges_mat(i, 0) = laser_data.range_max;
            //                } else {
            //                    laser_data.ranges_mat(i, 0) = laser_scan.ranges[i];
            //                }
            //                // Compute bearing
            //                laser_data.ranges_mat(i, 1) = angle_min + (i * angle_increment);
            //            }

            /*****电脑外壳给我挡住了 0.15 0.18 以及 0.04 0.16*****/
            // 这个范围内的数据我们直接不要了，还能提高精度
            laser_data.ranges_mat.resize(laser_data.range_count, 2);
            laser_data.ranges_mat.setZero();

            int valid_count = 0;
            for (int i = 0; i < laser_data.range_count; i++) {
                double angle_temp = angle_min + (i * angle_increment);
                if (((angle_temp < 2.4468 || angle_temp > 3.3866) && angle_increment > 0) ||
                    ((angle_temp < -3.8364 || angle_temp > -2.8966) && angle_increment < 0)) {
                    laser_data.ranges_mat(valid_count, 1) = angle_temp;

                    if (laser_scan.ranges[valid_count] <= range_min) {
                        laser_data.ranges_mat(valid_count, 0) = laser_data.range_max;
                    } else {
                        laser_data.ranges_mat(valid_count, 0) = laser_scan.ranges[i];
                    }
                    valid_count++;
                }
            }
            laser_data.range_count = valid_count;

            laser_model_ptr_->UpdateSensor(pf_ptr_, &laser_data);

            /** ↑原来UpdateLaser(const sensor_msgs::LaserScan &laser_scan,
                           double angle_min,
                           double angle_increment,
                           const Vec3d &pose,
                           geometry_msgs::PoseArray &particle_cloud_pose_msg)中的内容↑ **/

            laser_update_ = false;
            last_laser_update_time_ = ros::Time().now();
            pf_odom_pose_ = pose;

            resampled_ = false;

            /** ↓原来geometry_msgs::PoseArray Amcl::ResampleParticles()中的内容↓ **/
            if (!(++resample_count_ % resample_interval_)) {
                DLOG_INFO << "Resample the particles";
                //                if (forceUpdateMode_) {
                pf_ptr_->UpdateResample(hyp_pose.pose_mean(2));
                //                } else {
                //                    pf_ptr_->UpdateResample();
                //                }
                //pf_ptr_->UpdateNoResample();
                resampled_ = true;
            }

            auto set_ptr = pf_ptr_->GetCurrentSampleSetPtr();
            DLOG_INFO << "Number of samples : " << set_ptr->sample_count;

            // Publish the resulting particle cloud
            particle_cloud_pose_msg.poses.resize(set_ptr->sample_count);
            for (int i = 0; i < set_ptr->sample_count; i++) {
                tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set_ptr->samples_vec[i].pose[2]),
                                         tf::Vector3(set_ptr->samples_vec[i].pose[0],
                                                     set_ptr->samples_vec[i].pose[1],
                                                     0)),
                                particle_cloud_pose_msg.poses[i]);
            }
            publish_particle_pose_cloud_ = true;

            /** ↑原来geometry_msgs::PoseArray Amcl::ResampleParticles()中的内容↑ **/
        }

        /** ↓原来void Amcl::UpdateFilter(HypPose &hyp_pose, ros::Time laser_msg_timestamp)中的内容↓ **/
        // 如果没有重采样，且没有强制发布信息，则没啥工作； update_tf_ = false; publish_pose_ = true;
        // 如果没有重采样，且有强制发布信息，则统计簇信息，并将权重最大的簇的均值、协方差当做后验结果 update_tf_ = true; publish_pose_ = true;
        // 有重采样，则将权重最大的簇的均值、协方差当做后验结果（重采样过程中已经统计过粗信息了）
        if (resampled_ || force_publication_) {
            //好像可以不要这一句，需要再确认
            if (!resampled_) {
                DLOG_INFO << "Recompute particle filter cluster statistics";
                pf_ptr_->ClusterStatistics();
            }
            // Read out the current hypotheses
            double max_weight = 0.0;
            int max_weight_hyp = -1;
            std::vector<AmclHyp> hyps;
            hyps.resize(pf_ptr_->GetCurrentSampleSetPtr()->cluster_count);
            for (int hyp_count = 0;
                 hyp_count < pf_ptr_->GetCurrentSampleSetPtr()->cluster_count;
                 hyp_count++) {
                double weight;
                Vec3d pose_mean;
                Mat3d pose_cov;

                if (!pf_ptr_->GetClusterStatistics(hyp_count,
                                                   &weight,
                                                   &pose_mean,
                                                   &pose_cov)) {
                    LOG_ERROR << "Couldn't get stats on cluster " << hyp_count;
                    break;
                }

                hyps[hyp_count].weight = weight;
                hyps[hyp_count].pf_pose_mean = pose_mean;
                hyps[hyp_count].pf_pose_cov = pose_cov;

                if (hyps[hyp_count].weight > max_weight) {
                    max_weight = hyps[hyp_count].weight;
                    max_weight_hyp = hyp_count;
                }
            }

            if (max_weight > 0.0) {

                DLOG_INFO << "Max weight: " << max_weight
                          << ", Pose: "
                          << hyps[max_weight_hyp].pf_pose_mean[0] << ", "
                          << hyps[max_weight_hyp].pf_pose_mean[1] << ", "
                          << hyps[max_weight_hyp].pf_pose_mean[2];

                auto set = pf_ptr_->GetCurrentSampleSetPtr();

                hyp_pose.pose_mean = hyps[max_weight_hyp].pf_pose_mean;
                hyp_pose.pose_set_cov = set->covariant;

                publish_pose_ = true;
                update_tf_ = true;
            } else {
                LOG_ERROR << "Max weight of clusters less than 0!";
                publish_pose_ = false;
                update_tf_ = false;
            }
        } else {
            // Nothing changed, republish the last transform.
            update_tf_ = false;
            publish_pose_ = true;
        }
        /** ↑原来void Amcl::UpdateFilter(HypPose &hyp_pose, ros::Time laser_msg_timestamp)中的内容↑ **/
    };

    bool Amcl::CheckTfUpdate() {
        return update_tf_;
    }

    const nav_msgs::OccupancyGrid &Amcl::GetDistanceMapMsg() {
        return map_ptr_->ConvertDistanMaptoMapMsg();
    }
} // namespace leonard_localization
