//
// Created by cxn on 19-2-16.
//

#ifndef PROJECT_PARTICLE_FILTER_H
#define PROJECT_PARTICLE_FILTER_H

#include "map/amcl_map.h"
#include "particle_filter_sample.h"
#include "particle_filter_gaussian_pdf.h"


namespace leonard_localization {
    class ParticleFilter;
    using ParticleFilterPtr = std::shared_ptr<ParticleFilter>;
    using SampleSetPtr = std::shared_ptr<ParticleFilterSampleSet>;
    using PfInitModelFunc = std::function<Vec3d()>;

    /**
    * @brief Particle filter class
    */
    class ParticleFilter{
    public:
        /**
         * @brief Constructor of particle filter class
         * @param min_samples Min number of samples
         * @param max_samples Max number of samples
         * @param alpha_slow Decay rates for running averages
         * @param alpha_fast Decay rates for running averages
         * @param random_pose_func Function used to draw random pose samples
         * @param random_pose_data
         */
        ParticleFilter(int min_samples,
                       int max_samples,
                       double alpha_slow,
                       double alpha_fast,
                       const PfInitModelFunc &random_pose_func,//用于生成世界坐标系中的点，单位m
                       const std::shared_ptr<AmclMap> &map_ptr
        );
        /**
         * @brief Destructor
         */
        ~ParticleFilter();

        /**
         * @brief Initialize the filter using some model
         * @param init_fn Model function
         * @param init_data Model data
         */
        void InitByModel(PfInitModelFunc init_fn);

        SampleSetPtr GetCurrentSampleSetPtr() const;

        void SetKldParam(double pop_err, double pop_z);

        /**
         * @brief Resample the distribution
         */
        void UpdateResample();

        /**
         * @brief Compute the required number of samples,
         *        given that there are k bins with samples in them.
         *        This is taken directly from Fox et al.
         * @param k Bins with samples
         * @return Returns the required number of samples for resampling.
         */
        int ResampleLimit(int k);

        void UpdateOmega(double total_weight);

        /**
         * @brief Initialize the filter using a guassian pdf
         * @param mean Initial pose mean
         * @param cov Initial pose covariant
         */
        void InitByGuassian(const Vec3d &mean, const Mat3d &cov);

        void InitByGuassianWithRandomHeading(const Vec3d &mean, const Mat3d &cov);

        /**
         * @brief Compute the statistics for a particular cluster.
         * @param clabel Cluster label
         * @param weight Weight of the cluster
         * @param mean Mean of the cluster
         * @param cov Cov of the cluster
         * @return Returns 0 if there is no such cluster.
         */
        int GetClusterStatistics(int clabel, double *weight,
                                 Vec3d *mean, Mat3d *cov);


        void ClusterStatistics();

        /******************测试用code*********************/
        // 不重采样
        void UpdateNoResample();
        // 在某些地方生成随机粒子
        void RandomByFixedPose(const Vec3d &mean, const Mat3d &cov);
        // 随机生成粒子角度不能超过±M_PI/2，防止车子坐标反了
        void UpdateResample(double angle);

    private:
        void ClusterStatistics(const SampleSetPtr &sample_set_ptr);
        void InitConverged();
        bool UpdateConverged();
    private:

        //! This min and max number of samples
        int min_samples_ = 0, max_samples_ = 0;

        //! Population size parameters
        double pop_err_, pop_z_;

        //! The sample sets.  We keep two sets and use [current_set] to identify the active set.
        int current_set_;

        std::array<SampleSetPtr, 2> sample_set_ptr_array_;

        //! Running averages, slow and fast, of likelihood
        double w_slow_, w_fast_;

        //! Decay rates for running averages
        double alpha_slow_, alpha_fast_;

        //! Function used to draw random pose samples
        PfInitModelFunc random_pose_func_;
        std::shared_ptr<AmclMap> map_ptr_;

        //! Distance threshold in each axis over which the pf is considered to not be converged
        double dist_threshold_ = 0.5;
        //! Particle filter converged flag
        bool converged_ = false;
    };
}


#endif //PROJECT_PARTICLE_FILTER_H
