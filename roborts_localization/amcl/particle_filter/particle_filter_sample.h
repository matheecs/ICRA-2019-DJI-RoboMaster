//
// Created by cxn on 19-2-16.
//

#ifndef PROJECT_PARTICLE_FILTER_SAMPLE_H
#define PROJECT_PARTICLE_FILTER_SAMPLE_H

#include "particle_filter_kdtree.h"
#include "types.h"

namespace leonard_localization {
    /**
    * @brief Information for a single sample.
    */
    class ParticleFilterSample {
    public:
        ParticleFilterSample() {
            Reset();
        }

        /**
        * @brief Particle filter sample initialize
        */
        void Reset() {
            pose.setZero();
            weight = 0;
        };

    public:
        //! Pose represented by this sample
        Vec3d pose;

        //! Weight for this pose
        double weight;
    };

    /**
    * @brief Information for a cluster of samples
    */
    class ParticleFilterCluster {
    public:
        ParticleFilterCluster() {
            Reset();
        }
        void Reset() {
            count = 0;
            weight = 0;
            mean.setZero();
            cov.setZero();
            ws_vec.setZero();
            ws_mat.setZero();
        }
    public:
        //! Number of samples
        int count;
        //! Total weight of samples in this cluster
        double weight;
        //! Cluster statistics mean
        Vec3d mean;
        //! Cluster statistics cov
        Mat3d cov;
        //! Workspace 4 double vector
        Vec4d ws_vec;//源码中的m
        //! Workspace 2x2 double matrix
        Mat2d ws_mat;//源码中的c
    };

    /**
    * @brief Information for a set of samples
    */
    class ParticleFilterSampleSet {
    public:
        //! Number of samples
        int sample_count = 0;
        //! Vector of samples
        std::vector<ParticleFilterSample> samples_vec;
        //! Number of Clusters
        int cluster_count = 0;
        //! Max number of clusters
        int cluster_max_count = 0;
        //! Vector of clusters
        std::vector<ParticleFilterCluster> clusters_vec;
        //! A kdtree encoding the histogram
        std::unique_ptr<ParticleFilterKDTree> kd_tree_ptr;
        //! Filter statistics mean
        Vec3d mean;
        //! Filter statistics covariant
        Mat3d covariant;
        //! Filter statistics converged status
        bool converged = false;
    };
}
#endif //PROJECT_PARTICLE_FILTER_SAMPLE_H
