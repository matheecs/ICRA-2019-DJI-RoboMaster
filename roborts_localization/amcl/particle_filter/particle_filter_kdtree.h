//
// Created by cxn on 19-2-16.
//

#ifndef PROJECT_PARTICLE_FILTER_KDTREE_H
#define PROJECT_PARTICLE_FILTER_KDTREE_H

#include "types.h"
#include "log.h"
#include <memory>
namespace leonard_localization {
    /**
    * @brief Information for a particle filter K-D tree node
    */
    class ParticleFilterKDTreeNode {
    public:
        //! Left child node
        ParticleFilterKDTreeNode *left_ptr;
        //! Right child node
        ParticleFilterKDTreeNode *right_ptr;
        //! Leaf node flag
        bool leaf = false;
        //! Depth in the tree
        int depth = 0;
        //! The cluster label
        int cluster = -1;
        //! Pivot dimension
        int pivot_dim = 0;
        //! Pivot value
        double pivot_value = 0;
        //! The key for this node
        Vec3d key = {0, 0, 0};
        //! The value for this node
        double value = 0;
    };

    /**
    * @brief K-D tree class
    */
    class ParticleFilterKDTree {
    public:
        /**
        * @brief Create a K-D tree by max sample size
        * @param max_size Max sample size
        */
        void InitializeByMaxSize(int max_size);
        /**
        * @brief Clear all entries from the tree
        */
        void Clear();
        /**
         * @brief Insert a pose into the tree
         * @param pose Pose to insert
         * @param value Weight of pose
         */
        void InsertPose(Vec3d pose, double value);
        /**
        * @brief Destuctor
        */
        ~ParticleFilterKDTree();
        /**
        * @brief Get number of leaves
        * @return Returns number of leaves.
        */
        const int &GetLeafCount() const;
        /**
         * @brief Cluster the leaves in the tree
         */
        void Cluster();
        /**
         * @brief Determine the cluster label for the given pose
         * @param pose Pose
         * @return Returns the cluster label of given pose if exist.
         */
        int GetCluster(Vec3d pose);

    private:
        ParticleFilterKDTreeNode *InsertNode(
                ParticleFilterKDTreeNode *parent_node_ptr,
                ParticleFilterKDTreeNode *node_ptr,
                const Vec3d &key,
                double value);

        ParticleFilterKDTreeNode *FindNode(
                ParticleFilterKDTreeNode *node_ptr, Vec3d key);

        inline bool IsEqualKey(const Vec3d &key_a, const Vec3d &key_b);
        void ClusterNode(ParticleFilterKDTreeNode *node_ptr, int depth);

    private:
        //! Cell size
        double size_[3] = {0.5,0.5,(10.0 * M_PI / 180.0)};
        //! The root node of the tree
        ParticleFilterKDTreeNode *root_ptr_ = nullptr;
        //! The number of nodes in the tree
        int node_count_ = 0;
        //! Max number of nodes in the tree
        int node_max_count_ = 0;
        //! Vector of nodes
        std::vector<ParticleFilterKDTreeNode *> nodes_ptr_vec_;
        //! The number of leaf nodes in the tree
        int leaf_count_ = 0;
    };
}

#endif //PROJECT_PARTICLE_FILTER_KDTREE_H
