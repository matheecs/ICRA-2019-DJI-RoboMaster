//
// Created by cxn on 19-2-15.
//

#ifndef PROJECT_SENSOR_LASER_H
#define PROJECT_SENSOR_LASER_H

#include "map/amcl_map.h"
#include "particle_filter/particle_filter.h"
#include "localization_math.h"

namespace leonard_localization {

    enum LaserModel {
        LASER_MODEL_BEAM = 0,
        LASER_MODEL_LIKELIHOOD_FIELD = 1,
        LASER_MODEL_LIKELIHOOD_FIELD_PROB = 2
    };

    /**
     * @brief Laser data class
     */
    class SensorLaserData {
    public:
        /**
         * @brief Default constructor.
         */
        SensorLaserData() {
            range_count = 0;
            range_max = 0;
            ranges_mat.setZero();
        }

        /**
         * @brief Default destructor.
         */
        ~SensorLaserData() = default;
    public:

        /**
         * @brief Ranges count of laser
         */
        int range_count;

        /**
         * @brief Max range of laser
         */
        double range_max;

        /**
         * @brief Laser ranges data
         */
        MatX2d ranges_mat;//第一列是距离，第二列是角度
    };

    /**
     * @brief Laser sensor model class
     */
    class SensorLaser {
    public:
        /**
         * @brief Set laser pose
         * @param laser_pose Laser pose to set
         */
        void SetLaserPose(const Vec3d &laser_pose);

        /**
         * @brief Constructor function
         * @param max_beams Laser max beams number
         * @param map_ptr AmclMap object pointer
         */
        SensorLaser(size_t max_beams, const std::shared_ptr<AmclMap> &map_ptr);

        /**
         * @brief Initialize laser likelihood field model.
         * @param z_hit Measurement noise coefficient
         * @param z_rand Random measurement noise coefficient
         * @param sigma_hit Stddev of Gaussian model for laser hits
         * @param max_occ_dist Max distance at which we care about obstacles
         * @param do_beamskip Beam skipping option
         * @param beam_skip_distance Beam skipping distance
         * @param beam_skip_threshold Beam skipping threshold
         * @param beam_skip_error_threshold Threshold for the ratio of invalid beams -
         *                                  at which all beams are integrated to the likelihoods.
         *                                  This would be an error condition
         */
        void SetModelLikelihoodFieldProb(double z_hit,
                                         double z_rand,
                                         double sigma_hit,
                                         double max_occ_dist,
                                         bool do_beamskip,
                                         double beam_skip_distance,
                                         double beam_skip_threshold,
                                         double beam_skip_error_threshold);

        /**
         * @brief Determine the probability for the given pose. A more probabilistically correct model
         *        - also with the option to do beam skipping
         * @param sensor_laser_data_ptr Sensor data object pointer
         * @param sample_set_ptr Paticle sample set object pointer
         * @return Returns weight of given pose.
         */
        double LikelihoodFieldModelProb(SensorLaserData *sensor_laser_data_ptr,
                                        SampleSetPtr sample_set_ptr);

        /**
         * @brief Update the filter based on the sensor model.
         * @param pf_ptr Particle filter object pointer
         * @param sensor_data_ptr Sensor data object pointer
         * @return 如果权重为0，即max_beams<2，返回false
         */
        void UpdateSensor(ParticleFilterPtr pf_ptr, SensorLaserData *sensor_data_ptr);

        /**
         * @brief Update the filter based on the sensor model.
         * @param pf_ptr Particle filter object pointer
         * @param sensor_data_ptr Sensor data object pointer
         * @return Returns total weight of samples.
         */
        // 这个UpdateSensorLikelihoodFieldModelProb函数是官方开源的UpdateSensor，
        // 重新对UpdateSensor做了封装，原来particle_filter.c中的UpdateOmega加了进来，即对长期似然与短期似然的计算
        double UpdateSensorLikelihoodFieldModelProb(ParticleFilterPtr pf_ptr, SensorLaserData *sensor_data_ptr);

        void SensorMaxbeams(size_t max_beams);

    private:
        void ResetTempData(int new_max_samples, int new_max_obs);

    private:
        Vec3d laser_pose_;//laser在baselink上的坐标

        LaserModel model_type_;

        /**
         * @brief AmclMap object pointer
         */
        std::shared_ptr<AmclMap> map_ptr_;

        int max_beams_;
        bool do_beamskip_;
        double beam_skip_distance_;
        double beam_skip_threshold_;
        double beam_skip_error_threshold_;

        int max_samples_ = 0;
        int max_obs_ = 0;

        //二维数组，总粒子数*激光束数
        std::vector<std::vector<double>> temp_obs_;

        // Laser model params
        // Mixture params for the components of the model; must sum to 1
        double z_hit_;
        double z_rand_;

        /**
         * @brief Stddev of Gaussian model for laser hits.
         */
        double sigma_hit_;
    };
}
#endif //PROJECT_SENSOR_LASER_H
