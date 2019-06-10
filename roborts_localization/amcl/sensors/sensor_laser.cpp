//
// Created by cxn on 19-2-15.
//
#include "sensor_laser.h"

namespace leonard_localization {

    void SensorLaser::SetLaserPose(const Vec3d &laser_pose) {
        laser_pose_ = laser_pose;
    }

    SensorLaser::SensorLaser(size_t max_beams,
                             const std::shared_ptr<AmclMap> &map_ptr) :
            max_samples_(0),
            max_obs_(0) {
        this->temp_obs_.clear();
        this->temp_obs_.shrink_to_fit();
        this->max_beams_ = max_beams;
        this->map_ptr_ = map_ptr;
    }

    void SensorLaser::SensorMaxbeams(size_t max_beams) {
        this->max_beams_ = max_beams;
    }

    void SensorLaser::SetModelLikelihoodFieldProb(double z_hit,
                                                  double z_rand,
                                                  double sigma_hit,
                                                  double max_occ_dist,//laser_likelihood_max_dist
                                                  bool do_beamskip,
                                                  double beam_skip_distance,
                                                  double beam_skip_threshold,
                                                  double beam_skip_error_threshold) {

        this->model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
        this->z_hit_ = z_hit;
        this->z_rand_ = z_rand;
        this->sigma_hit_ = sigma_hit;
        this->do_beamskip_ = do_beamskip;
        this->beam_skip_distance_ = beam_skip_distance;
        this->beam_skip_threshold_ = beam_skip_threshold;
        this->beam_skip_error_threshold_ = beam_skip_error_threshold;
        this->map_ptr_->UpdateCSpace(max_occ_dist);//2.13 这里会计算地图中每个格子距离最近障碍物块的距离
    }

    double SensorLaser::LikelihoodFieldModelProb(SensorLaserData *sensor_laser_data_ptr,
                                                 SampleSetPtr sample_set_ptr) {
        int i = 0, j = 0, step;
        double z, pz;
        double log_p;
        double obs_range, obs_bearing;
        double total_weight;
        Vec3d pose;
        Vec3d hit;

        total_weight = 0.0;
        step = std::ceil((sensor_laser_data_ptr->range_count) / static_cast<double>(this->max_beams_));

        // Step size must be at least 1
        if (step < 1) {
            step = 1;
        }

        double z_hit_denom = 2 * this->sigma_hit_ * this->sigma_hit_;
        double z_rand_mult = 1.0 / sensor_laser_data_ptr->range_max;
        auto max_occ_dist = this->map_ptr_->GetMaxOccDist();
        double max_dist_prob = std::exp(-(max_occ_dist * max_occ_dist) / z_hit_denom);

        /*为了方便起见，把所有默认参数列上
           z_hit                     : 0.5
           z_rand                    : 0.5
           sigma_hit                 : 0.2
           lambda_short              : 0.1
           laser_likelihood_max_dist : 5.0
           do_beamskip               : false
           beam_skip_distance        : 0.5
           beam_skip_threshold       : 0.3
           beam_skip_error_threshold : 0.9
         */

        //Beam skipping - ignores beams for which a majority of particles do not agree with the map
        //prevents correct particles from getting down weighted because of unexpected obstacles
        //such as humans
        bool do_beamskip = this->do_beamskip_;
        double beam_skip_distance = this->beam_skip_distance_;
        double beam_skip_threshold = this->beam_skip_threshold_;

        //we only do beam skipping if the filter has converged
        if (do_beamskip && !sample_set_ptr->converged) {
            do_beamskip = false;
            DLOG_INFO << "Filter not converged";
        }

        //we need a count the no of particles for which the beam agreed with the map
        //we also need a mask of which observations to integrate (to decide which beams to integrate to all particles)
        std::unique_ptr<int[]> obs_count(new int[this->max_beams_]);//默认值为30
        std::unique_ptr<bool[]> obs_mask(new bool[this->max_beams_]);

        int beam_ind = 0;

        //reset indicates if we need to reallocate the temp data structure needed to do beamskipping
        bool reset = false;

        if (do_beamskip) {

            if (this->max_obs_ < this->max_beams_) {
                reset = true;
            }

            if (this->max_samples_ < sample_set_ptr->sample_count) {
                reset = true;
            }

            if (reset) {
                this->ResetTempData(sample_set_ptr->sample_count, this->max_beams_);
                DLOG_INFO << "Reallocing temp weights " << this->max_samples_ << " - " << this->max_obs_;
            }
        }

        //Compute the sample weights

        for (j = 0; j < sample_set_ptr->sample_count; j++) {
            pose = sample_set_ptr->samples_vec.at(j).pose;
            pose = CoordAdd(this->laser_pose_, pose);
            log_p = 0;
            beam_ind = 0;

            for (i = 0; i < sensor_laser_data_ptr->range_count; i += step, beam_ind++) {
                obs_range = sensor_laser_data_ptr->ranges_mat(i, 0);
                obs_bearing = sensor_laser_data_ptr->ranges_mat(i, 1);

                // This model ignores max range readings
                if (obs_range >= sensor_laser_data_ptr->range_max) {
                    continue;
                }

                // Check for NaN
                if (obs_range != obs_range) {
                    continue;
                }

                pz = 0.0;

                // Compute the endpoint of the beam
                hit(0) = pose(0) + obs_range * std::cos(pose(2) + obs_bearing);
                hit(1) = pose(1) + obs_range * std::sin(pose(2) + obs_bearing);

                // Convert to map grid coords.
                int mi, mj;
                this->map_ptr_->ConvertWorldCoordsToMapCoords(hit(0), hit(1), mi, mj);

                // Part 1: Get distance from the hit to closest obstacle.
                // Off-map penalized as max distance
                if (!this->map_ptr_->CheckMapCoordsValid(mi, mj)) {
                    pz += this->z_hit_ * max_dist_prob;
                } else {

                    int cell_ind = this->map_ptr_->ComputeCellIndexByMap(mi, mj);
                    z = this->map_ptr_->GetCellOccDistByIndex(cell_ind);
                    if (z < beam_skip_distance) {
                        //只要不同粒子的情况下的同一束激光测出的到障碍物距离小于距离阈值，就会加1
                        obs_count[beam_ind] += 1;
                    }
                    pz += this->z_hit_ * std::exp(-(z * z) / z_hit_denom);
                }

                // Gaussian model
                // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)

                // Part 2: random measurements
                pz += this->z_rand_ * z_rand_mult;

                LOG_FATAL_IF(pz > 1.0 || pz < 0.0) << "pz error num = " << pz;

                if (!do_beamskip) {
                    log_p += log(pz);
                    //2.28 根据navigation包的代码骚一下，使距离差小的粒子权值更大
//                    log_p += 3 * log(pz);
                } else {
                    CHECK_GT(this->temp_obs_.size(), 0);
                    CHECK_GT(this->temp_obs_.at(j).size(), 0);
                    this->temp_obs_.at(j).at(beam_ind) = pz;
                    //2.28 根据navigation包的代码骚一下，使距离差小的粒子权值更大
//                    this->temp_obs_.at(j).at(beam_ind) = pz*pz*pz;
                }
            }
            if (!do_beamskip) {
                sample_set_ptr->samples_vec.at(j).weight *= exp(log_p);
                total_weight += sample_set_ptr->samples_vec.at(j).weight;
            }
        }

        //Skip part of beams
        if (do_beamskip) {
            int skipped_beam_count = 0;
            for (beam_ind = 0; beam_ind < this->max_beams_; beam_ind++) {
                if ((obs_count[beam_ind] / static_cast<double>(sample_set_ptr->sample_count)) > beam_skip_threshold) {
                    // 只要不同粒子的情况下的同一束激光测出的到障碍物距离小于距离阈值，就会加1
                    // 最终总和与总粒子数和之比大于某个阈值，就认为这一束激光测量准确，否则就忽略这一束激光
                    obs_mask[beam_ind] = true;
                } else {
                    obs_mask[beam_ind] = false;
                    skipped_beam_count++;
                }
            }
            DLOG(INFO) << "skipped_beam_count = " << skipped_beam_count << " max_beams = " << this->max_beams_;
            //we check if there is at least a critical number of beams that agreed with the map
            //otherwise it probably indicates that the filter converged to a wrong solution
            //if that's the case we integrate all the beams and hope the filter might converge to
            //the right solution
            bool error = false;
            if (skipped_beam_count >= (beam_ind * this->beam_skip_error_threshold_)) {
                //如果忽略激光数大于总激光数与某个阈值之积，就不对激光进行滤除
                LOG_ERROR << "Over " << (100 * this->beam_skip_error_threshold_)
                          << " of the observations were not in the map - "
                          << "pf may have converged to wrong pose - "
                          << "integrating all observations";
                error = true;
            }

            for (j = 0; j < sample_set_ptr->sample_count; j++) {
                pose = sample_set_ptr->samples_vec.at(j).pose;
                log_p = 0;
                for (beam_ind = 0; beam_ind < this->max_beams_; beam_ind++) {
                    if (error || obs_mask[beam_ind]) {
                        LOG_FATAL_IF(j > this->temp_obs_.size() - 1) << "temp_obs size = "
                                                                     << this->temp_obs_.size()
                                                                     << "j="
                                                                     << j;
                        LOG_FATAL_IF(beam_ind > this->temp_obs_.at(j).size() - 1) << "temp_obs at j size = "
                                                                                  << this->temp_obs_.at(j).size()
                                                                                  << "beam_ind = " << beam_ind;
                        log_p += std::log(this->temp_obs_.at(j).at(beam_ind));
                    }
                }
                sample_set_ptr->samples_vec.at(j).weight *= std::exp(log_p);
                total_weight += sample_set_ptr->samples_vec.at(j).weight;
            }
        }
        return (total_weight);
    }

    double
    SensorLaser::UpdateSensorLikelihoodFieldModelProb(ParticleFilterPtr pf_ptr, SensorLaserData *sensor_data_ptr) {
        if (this->max_beams_ < 2)
            return 0;
        auto set = pf_ptr->GetCurrentSampleSetPtr();
        // Apply the laser sensor model
        return LikelihoodFieldModelProb(sensor_data_ptr, set);
    }

    void SensorLaser::UpdateSensor(ParticleFilterPtr pf_ptr, SensorLaserData *sensor_data_ptr) {
        //如果max_beam比2小，则返回0
        double total_weight = UpdateSensorLikelihoodFieldModelProb(pf_ptr, sensor_data_ptr);

        //particle_filter.c中的UpdateOmega
        pf_ptr->UpdateOmega(total_weight);
    }


    void SensorLaser::ResetTempData(int new_max_samples, int new_max_obs) {

        max_obs_ = new_max_obs;
        max_samples_ = std::max(max_samples_, new_max_samples);
        DLOG_INFO << __FUNCTION__ << ": New max obs = " << max_obs_ << "New max samples = " << max_samples_;
        CHECK_GT(max_samples_, 0);
        temp_obs_.resize(max_samples_);
        for (int k = 0; k < max_samples_; k++) {
            CHECK_GT(max_obs_, 0);
            temp_obs_[k].resize(max_obs_, 0);
        }
    }
}