//
// Created by cxn on 19-2-16.
//

#include "particle_filter.h"

namespace leonard_localization {

    ParticleFilter::ParticleFilter(int min_samples,
                                   int max_samples,
                                   double alpha_slow,
                                   double alpha_fast,
                                   const PfInitModelFunc &random_pose_func,
                                   const std::shared_ptr<AmclMap> &map_ptr) {
        min_samples_ = min_samples;
        max_samples_ = max_samples;
        alpha_slow_ = alpha_slow;
        alpha_fast_ = alpha_fast;
        pop_err_ = 0.01;
        pop_z_ = 3;
        dist_threshold_ = 0.5;
        current_set_ = 0;

        random_pose_func_ = random_pose_func;
        map_ptr_ = map_ptr;

        for (auto &sample_set_it : sample_set_ptr_array_) {
            sample_set_it = std::make_shared<ParticleFilterSampleSet>();
            sample_set_it->sample_count = max_samples;
            sample_set_it->samples_vec.resize(max_samples);
            for (int i = 0; i < sample_set_it->sample_count; i++) {
                sample_set_it->samples_vec[i].weight = 1.0 / max_samples;
            }

            sample_set_it->kd_tree_ptr = std::make_unique<ParticleFilterKDTree>();
            sample_set_it->kd_tree_ptr->InitializeByMaxSize(3 * max_samples);
            sample_set_it->cluster_count = 0;
            sample_set_it->cluster_max_count = max_samples_;
            sample_set_it->clusters_vec.resize(sample_set_it->cluster_max_count);
            sample_set_it->mean.setZero();
            sample_set_it->covariant.setZero();
        }

        w_slow_ = 0;
        w_fast_ = 0;

        alpha_slow_ = alpha_slow;
        alpha_fast_ = alpha_fast;

        InitConverged();

    }

    void ParticleFilter::InitConverged() {
        this->sample_set_ptr_array_[current_set_]->converged = false;
        this->converged_ = false;
    }

    SampleSetPtr ParticleFilter::GetCurrentSampleSetPtr() const {
        return this->sample_set_ptr_array_[current_set_];
    }

    ParticleFilter::~ParticleFilter() {
        int i;
        for (i = 0; i < 2; i++) {
            this->sample_set_ptr_array_[i]->kd_tree_ptr.reset();
            this->sample_set_ptr_array_[i]->clusters_vec.clear();
            this->sample_set_ptr_array_[i]->clusters_vec.shrink_to_fit();
            this->sample_set_ptr_array_[i]->samples_vec.clear();
            this->sample_set_ptr_array_[i]->samples_vec.shrink_to_fit();
        }
        LOG_INFO << "Delete pf";
    }

    void ParticleFilter::InitByModel(PfInitModelFunc init_fn) {
        int i;
        auto set = this->sample_set_ptr_array_[current_set_];
        // Create the kd tree for adaptive sampling
        set->kd_tree_ptr->Clear();
        set->sample_count = max_samples_;

        // Compute the new sample poses
        for (i = 0; i < set->sample_count; i++) {
            set->samples_vec[i].weight = 1.0 / this->max_samples_;

            set->samples_vec[i].pose = init_fn();
            // Add sample to histogram
            set->kd_tree_ptr->InsertPose(set->samples_vec[i].pose, set->samples_vec[i].weight);
        }

        this->w_slow_ = this->w_fast_ = 0.0;
        ClusterStatistics(set);
        // Re-compute cluster statistics
        InitConverged();
        //set converged to 0
    }

    void ParticleFilter::ClusterStatistics(const SampleSetPtr &sample_set_ptr) {
        int i, j, k;
        int cluster_index;
        ParticleFilterCluster *cluster;

        //work space
        Mat2d tmp_ws_mat;
        Vec4d tmp_ws_vec;
        size_t count;
        double weight;

        // Cluster the samples
        sample_set_ptr->kd_tree_ptr->Cluster();//给kd树的叶子节点聚类

        // Initialize cluster stats
        sample_set_ptr->cluster_count = 0;

        for (i = 0; i < sample_set_ptr->cluster_max_count; i++) {
            sample_set_ptr->clusters_vec[i].Reset();//这个reset是写的，不是标准库里的
        }
        // Initialize overall filter stats
        count = 0;
        weight = 0.0;
        sample_set_ptr->mean.setZero();
        sample_set_ptr->covariant.setZero();
        tmp_ws_mat.setZero();
        tmp_ws_vec.setZero();

        // Compute cluster stats
        for (i = 0; i < sample_set_ptr->sample_count; i++) {
            auto sample_it = sample_set_ptr->samples_vec[i];
            // Get the cluster label for this sample
            cluster_index = sample_set_ptr->kd_tree_ptr->GetCluster(sample_it.pose);
            if (cluster_index < 0) {
                LOG_ERROR << "Cluster not found";
                return;
            }
            if (cluster_index >= sample_set_ptr->cluster_max_count) {
                continue;
            }
            if (cluster_index + 1 > sample_set_ptr->cluster_count) {
                sample_set_ptr->cluster_count = cluster_index + 1;
            }

            sample_set_ptr->clusters_vec[cluster_index].count += 1;
            sample_set_ptr->clusters_vec[cluster_index].weight += sample_it.weight;

            count += 1;
            weight += sample_it.weight;

            //compute mean
            sample_set_ptr->clusters_vec[cluster_index].ws_vec(0) += sample_it.weight * sample_it.pose(0);
            sample_set_ptr->clusters_vec[cluster_index].ws_vec(1) += sample_it.weight * sample_it.pose(1);
            sample_set_ptr->clusters_vec[cluster_index].ws_vec(2) += sample_it.weight * cos(sample_it.pose(2));
            sample_set_ptr->clusters_vec[cluster_index].ws_vec(3) += sample_it.weight * sin(sample_it.pose(2));

            /* 2.14 leonard做的修改
            tmp_ws_vec(0) += sample_it.weight * sample_set_ptr->clusters_vec[cluster_index].ws_vec(0);
            tmp_ws_vec(1) += sample_it.weight * sample_set_ptr->clusters_vec[cluster_index].ws_vec(1);
            */
            tmp_ws_vec(0) += sample_it.weight * sample_it.pose(0);
            tmp_ws_vec(1) += sample_it.weight * sample_it.pose(1);

            tmp_ws_vec(2) += sample_it.weight * std::cos(sample_it.pose(2));
            tmp_ws_vec(3) += sample_it.weight * std::sin(sample_it.pose(2));

            // Compute covariance in linear components
            /* 2.14 leonard做的修改
            for (int j = 0; j < 2; j++) {
              for (int k = 0; k < 2; k++) {
                cluster = &sample_set_ptr->clusters_vec[cluster_index];
                cluster->cov(j, k) = cluster->ws_mat(j, k) / cluster->weight -
                    cluster->mean(j) * cluster->mean(k);
              }
            }*/
            for (int j = 0; j < 2; j++) {
                for (int k = 0; k < 2; k++) {
                    cluster = &sample_set_ptr->clusters_vec[cluster_index];
                    cluster->ws_mat(j, k) += sample_it.weight * sample_it.pose(j) * sample_it.pose(k);
                    tmp_ws_mat(j, k) += sample_it.weight * sample_it.pose(j) * sample_it.pose(k);
                }
            }
        }

        // Normalize
        for (i = 0; i < sample_set_ptr->cluster_count; i++) {

            cluster = &sample_set_ptr->clusters_vec[i];

            cluster->mean(0) = cluster->ws_vec(0) / cluster->weight;
            cluster->mean(1) = cluster->ws_vec(1) / cluster->weight;
            cluster->mean(2) = std::atan2(cluster->ws_vec(3), cluster->ws_vec(2));

            cluster->cov.setZero();

            // Covariance in linear components
            for (j = 0; j < 2; j++) {
                for (k = 0; k < 2; k++) {
                    //协方差计算公式
                    //cov(x,y)=E{[x-E(x)][y-E(y)]}=E(xy)-E(x)E(y)
                    cluster->cov(j, k) = cluster->ws_mat(j, k) / cluster->weight -
                                         cluster->mean(j) * cluster->mean(k);
                }
            }

            // Covariance in angular components
            cluster->cov(2, 2) = -2 * std::log(
                    std::sqrt(
                            cluster->ws_vec(2) * cluster->ws_vec(2) +
                            cluster->ws_vec(3) * cluster->ws_vec(3)
                    ));
            DLOG_INFO << "cluster: " << cluster->count
                      << "," << cluster->weight
                      << "," << cluster->mean(0)
                      << "," << cluster->mean(1) << "," << cluster->mean(2);
        }

        // Compute overall filter stats
        sample_set_ptr->mean(0) = tmp_ws_vec(0) / weight;
        sample_set_ptr->mean(1) = tmp_ws_vec(1) / weight;
        sample_set_ptr->mean(2) = std::atan2(tmp_ws_vec(3), tmp_ws_vec(2));

        // Covariance in linear components
        for (j = 0; j < 2; j++) {
            for (k = 0; k < 2; k++) {
                sample_set_ptr->covariant(j, k) =
                        tmp_ws_mat(j, k) / weight - sample_set_ptr->mean(j) * sample_set_ptr->mean(k);
            }
        }

        // Covariance in angular components;
        sample_set_ptr->covariant(2, 2) =
                -2 * std::log(std::sqrt(tmp_ws_vec(2) * tmp_ws_vec(2) + tmp_ws_vec(3) * tmp_ws_vec(3)));

    }

    void ParticleFilter::UpdateResample() {

        double w_diff = 0;
        double total = 0;
        int i = 0;

        ParticleFilterSample *sample_a, *sample_b;

        auto set_a = this->sample_set_ptr_array_[current_set_];
        auto set_b = this->sample_set_ptr_array_[(current_set_ + 1) % 2];

        // Build up cumulative probability table for resampling.
        // TODO: Replace this with a more efficient procedure
        // (e.g., http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)
        const int c_size = set_a->sample_count + 1;
        std::vector<double> c;
        c.resize(c_size);
        c[0] = 0.0;
        for (i = 0; i < set_a->sample_count; i++) {
            c[i + 1] = c[i] + set_a->samples_vec[i].weight;
        }

        set_b->kd_tree_ptr->Clear();

        // Draw samples from set a to create set b.
        total = 0;
        set_b->sample_count = 0;

        w_diff = 1.0 - this->w_fast_ / this->w_slow_;
        if (w_diff < 0.0) {
            w_diff = 0.0;
        }

        while (set_b->sample_count < this->max_samples_) {
            sample_b = &set_b->samples_vec[set_b->sample_count++];

            if (drand48() < w_diff)
                sample_b->pose = (random_pose_func_)();
            else {
                double r;
                r = drand48();

                //好像是转盘采样
                for (i = 0; i < set_a->sample_count; i++) {
                    if ((c[i] <= r) && (r < c[i + 1]))
                        break;
                }
                assert(i < set_a->sample_count);
                sample_a = &(set_a->samples_vec[i]);

                assert(sample_a->weight > 0);
                sample_b->pose = sample_a->pose;
            }

            sample_b->weight = 1.0;
            total += sample_b->weight;

            // Add sample to histogram
            set_b->kd_tree_ptr->InsertPose(sample_b->pose, sample_b->weight);

            // See if we have enough samples yet
            DLOG_INFO << "Histogram bins num: " << set_b->kd_tree_ptr->GetLeafCount();
            auto kld_resample_num = ResampleLimit(set_b->kd_tree_ptr->GetLeafCount());
            if (set_b->sample_count > kld_resample_num) {
                LOG_INFO << "KLD-Resample num : " << kld_resample_num;
                break;
            }
        }

        // Reset averages, to avoid spiraling off into complete randomness.
        // 为了便面一直生成随机粒子
        if (w_diff > 0.0)
            this->w_slow_ = this->w_fast_ = 0.0;

        // Normalize weights
        for (i = 0; i < set_b->sample_count; i++) {
            sample_b = &(set_b->samples_vec[i]);
            sample_b->weight /= total;
        }

        // Re-compute cluster statistics
        // 这个函数将kd树中的叶子节点分簇，即计算后三个值差在-1 0 1之间为一簇，
        // 会计算每一簇pose的均值，协方差矩阵，以及滤波后所有粒子的均值、协方差
        ClusterStatistics(set_b);
        current_set_ = (current_set_ + 1) % 2;
        // x,y都收敛到dist_threshold_内算收敛
        UpdateConverged();
        //  c.reset();

    }

    void ParticleFilter::UpdateResample(double angle) {

        double w_diff = 0;
        double total = 0;
        int i = 0;

        ParticleFilterSample *sample_a, *sample_b;

        auto set_a = this->sample_set_ptr_array_[current_set_];
        auto set_b = this->sample_set_ptr_array_[(current_set_ + 1) % 2];

        // Build up cumulative probability table for resampling.
        // TODO: Replace this with a more efficient procedure
        // (e.g., http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)
        const int c_size = set_a->sample_count + 1;
        std::vector<double> c;
        c.resize(c_size);
        c[0] = 0.0;
        for (i = 0; i < set_a->sample_count; i++) {
            c[i + 1] = c[i] + set_a->samples_vec[i].weight;
        }

        set_b->kd_tree_ptr->Clear();

        // Draw samples from set a to create set b.
        total = 0;
        set_b->sample_count = 0;

        w_diff = 1.0 - this->w_fast_ / this->w_slow_;
        if (w_diff < 0.0) {
            w_diff = 0.0;
        }

        while (set_b->sample_count < this->max_samples_) {
            sample_b = &set_b->samples_vec[set_b->sample_count++];

            if (drand48() < w_diff) {
                sample_b->pose = (random_pose_func_)();
                while (fabs(angle_diff<double>(sample_b->pose(2), angle)) > M_PI / 2
//                       && fabs(angle_diff<double>(sample_b->pose(2), angle2)) > M_PI / 2
                        ) {
                    sample_b->pose = (random_pose_func_)();
                }
            } else {
                double r;
                r = drand48();

                //好像是转盘采样
                for (i = 0; i < set_a->sample_count; i++) {
                    if ((c[i] <= r) && (r < c[i + 1]))
                        break;
                }
                assert(i < set_a->sample_count);
                sample_a = &(set_a->samples_vec[i]);

                assert(sample_a->weight > 0);
                sample_b->pose = sample_a->pose;
            }

            sample_b->weight = 1.0;
            total += sample_b->weight;

            // Add sample to histogram
            set_b->kd_tree_ptr->InsertPose(sample_b->pose, sample_b->weight);

            // See if we have enough samples yet
            DLOG_INFO << "Histogram bins num: " << set_b->kd_tree_ptr->GetLeafCount();
            auto kld_resample_num = ResampleLimit(set_b->kd_tree_ptr->GetLeafCount());
            if (set_b->sample_count > kld_resample_num) {
                LOG_INFO << "KLD-Resample num : " << kld_resample_num;
                break;
            }
        }

        // Reset averages, to avoid spiraling off into complete randomness.
        // 为了便面一直生成随机粒子
        if (w_diff > 0.0)
            this->
                    w_slow_ = this->w_fast_ = 0.0;

        // Normalize weights
        for (
                i = 0;
                i < set_b->
                        sample_count;
                i++) {
            sample_b = &(set_b->samples_vec[i]);
            sample_b->weight /=
                    total;
        }

// Re-compute cluster statistics
// 这个函数将kd树中的叶子节点分簇，即计算后三个值差在-1 0 1之间为一簇，
// 会计算每一簇pose的均值，协方差矩阵，以及滤波后所有粒子的均值、协方差
        ClusterStatistics(set_b);
        current_set_ = (current_set_ + 1) % 2;

// x,y都收敛到dist_threshold_内算收敛
        UpdateConverged();
//  c.reset();

    }

    void ParticleFilter::UpdateNoResample() {
        double w_diff = 0;
        double total = 0;
        int i = 0;

        ParticleFilterSample *sample_a, *sample_b;

        auto set_a = this->sample_set_ptr_array_[current_set_];
        auto set_b = this->sample_set_ptr_array_[(current_set_ + 1) % 2];

        set_b->kd_tree_ptr->Clear();
        set_b->sample_count = 0;
        assert(set_b->sampels_vec.size() == this->max_samples_);
        assert(set_a->sampels_vec.size() == this->max_samples_);
        while (set_b->sample_count < this->max_samples_) {
            sample_b = &set_b->samples_vec[set_b->sample_count++];
            sample_a = &(set_a->samples_vec[set_b->sample_count - 1]);
            sample_b->pose = sample_a->pose;
            sample_b->weight = 1.0;
            total += sample_b->weight;
            // Add sample to histogram
            set_b->kd_tree_ptr->InsertPose(sample_b->pose, sample_b->weight);
        }

        // Normalize weights
        for (i = 0; i < set_b->sample_count; i++) {
            sample_b = &(set_b->samples_vec[i]);
            sample_b->weight /= total;
        }

        // Re-compute cluster statistics
        // 这个函数将kd树中的叶子节点分簇，即计算后三个值差在-1 0 1之间为一簇，
        // 会计算每一簇pose的均值，协方差矩阵，以及滤波后所有粒子的均值、协方差
        ClusterStatistics(set_b);
        current_set_ = (current_set_ + 1) % 2;
        // x,y都收敛到dist_threshold_内算收敛
        UpdateConverged();
        //  c.reset();

    }


    int ParticleFilter::ResampleLimit(int k) {
        double a, b, c, x;
        int n;

        if (k <= 1) {
            LOG_WARNING_IF(k != 1) << "K < 1 (kd_tree leaf count < 1)";
            return this->max_samples_;
        }

        a = 1.0;
        b = 2.0 / (9.0 * ((double) k - 1.0));
        c = std::sqrt(2.0 / (9.0 * ((double) k - 1))) * this->pop_z_;
        x = a - b + c;

        n = (int) std::ceil((k - 1.0) / (2.0 * this->pop_err_) * x * x * x);

        //  LOG_INFO << "Required number of samples = "<< n;

        if (n < this->min_samples_) {
            return this->min_samples_;
        }
        if (n > this->max_samples_) {
            return this->max_samples_;
        }

        return n;
    }

    bool ParticleFilter::UpdateConverged() {
        auto sample_set_ptr = sample_set_ptr_array_[current_set_];
        double mean_x = 0, mean_y = 0;

        int i;

        for (i = 0; i < sample_set_ptr->sample_count; i++) {
            mean_x += sample_set_ptr->samples_vec[i].pose(0);
            mean_y += sample_set_ptr->samples_vec[i].pose(1);
        }
        mean_x /= sample_set_ptr->sample_count;
        mean_y /= sample_set_ptr->sample_count;

        for (i = 0; i < sample_set_ptr->sample_count; i++) {
            if (std::fabs(sample_set_ptr->samples_vec[i].pose(0) - mean_x) > this->dist_threshold_ ||
                std::fabs(sample_set_ptr->samples_vec[i].pose(1) - mean_y) > this->dist_threshold_) {
                sample_set_ptr->converged = false;
                this->converged_ = false;
                return false;
            }
        }

        sample_set_ptr->converged = true;
        this->converged_ = true;
        return true;
    }

    void ParticleFilter::UpdateOmega(double total_weight) {
        int i = 0;
        auto set = GetCurrentSampleSetPtr();

        if (total_weight > 0.0) {
            // Normalize weights
            // 2.27leonard做的修改
            // double w_avg = 0.0;

            for (i = 0; i < set->sample_count; i++) {
                // 2.27leonard做的修改
                // w_avg += set->samples_vec[i].weight;
                set->samples_vec[i].weight /= total_weight;
            }

            DLOG_INFO << "Update running averages of likelihood of samples"; //(Probabilistic Robotics p258)

            // 2.27leonard做的修改
            // w_avg /= set->sample_count;
            double w_avg = total_weight / set->sample_count;

            if (this->w_slow_ == 0.0)
                this->w_slow_ = w_avg;
            else
                this->w_slow_ += this->alpha_slow_ * (w_avg - this->w_slow_);
            if (this->w_fast_ == 0.0)
                this->w_fast_ = w_avg;
            else
                this->w_fast_ += this->alpha_fast_ * (w_avg - this->w_fast_);
        } else {
            for (i = 0; i < set->sample_count; i++) {
                auto sample = &set->samples_vec[i];
                sample->weight = 1.0 / set->sample_count;
            }
        }
    }

    void ParticleFilter::SetKldParam(double pop_err, double pop_z) {
        this->pop_err_ = pop_err;
        this->pop_z_ = pop_z;
    }

    void ParticleFilter::InitByGuassian(const Vec3d &mean, const Mat3d &cov) {
        int i;
        auto sample_set_ptr = sample_set_ptr_array_[current_set_];

        // Create the kd tree for adaptive sampling
        sample_set_ptr->kd_tree_ptr->Clear();

        sample_set_ptr->sample_count = max_samples_;

        auto pf_gaussian_pdf = ParticleFilterGaussianPdf(mean, cov);

        // Compute the new sample poses
        for (i = 0; i < sample_set_ptr->sample_count; i++) {
            sample_set_ptr->samples_vec[i].weight = 1.0 / max_samples_;
            sample_set_ptr->samples_vec[i].pose = pf_gaussian_pdf.GenerateSample();

            // Add sample to histogram
            sample_set_ptr->kd_tree_ptr->InsertPose(sample_set_ptr->samples_vec[i].pose,
                                                    sample_set_ptr->samples_vec[i].weight);
        }

        this->w_slow_ = this->w_fast_ = 0.0;

        // Re-compute cluster statistics
        ClusterStatistics(sample_set_ptr);

        //set converged to 0
        InitConverged();
    }


    void ParticleFilter::InitByGuassianWithRandomHeading(const Vec3d &mean, const Mat3d &cov) {
        int i;
        auto sample_set_ptr = sample_set_ptr_array_[current_set_];

        LOG_INFO << "Init with random heading!";

        sample_set_ptr->kd_tree_ptr->Clear();

        sample_set_ptr->sample_count = max_samples_;

        auto pf_gaussian_pdf = ParticleFilterGaussianPdf(mean, cov);

        for (i = 0; i < sample_set_ptr->sample_count; i++) {
            sample_set_ptr->samples_vec[i].weight = 1.0 / max_samples_;
            sample_set_ptr->samples_vec[i].pose = pf_gaussian_pdf.GenerateSample();
            sample_set_ptr->samples_vec[i].pose[2] = drand48() * 2 * M_PI - M_PI;
            sample_set_ptr->kd_tree_ptr->InsertPose(sample_set_ptr->samples_vec[i].pose,
                                                    sample_set_ptr->samples_vec[i].weight);
        }

        this->w_slow_ = this->w_fast_ = 0.0;

        ClusterStatistics(sample_set_ptr);
        InitConverged();

    }


    int ParticleFilter::GetClusterStatistics(int clabel,
                                             double *weight,
                                             Vec3d *mean,
                                             Mat3d *cov) {
        auto set = this->sample_set_ptr_array_[current_set_];
        if (clabel >= set->sample_count) {
            return 0;
        }
        *weight = set->clusters_vec[clabel].weight;
        *mean = set->clusters_vec[clabel].mean;
        *cov = set->clusters_vec[clabel].cov;

        return 1;
    }


    void ParticleFilter::RandomByFixedPose(const Vec3d &mean, const Mat3d &cov) {
        int i;
        auto sample_set_ptr = sample_set_ptr_array_[current_set_];

        auto pf_gaussian_pdf = ParticleFilterGaussianPdf(mean, cov);

        int samples_num = std::min(max_samples_, sample_set_ptr->sample_count * 10);

        for (i = sample_set_ptr->sample_count; i < samples_num; i++) {
            sample_set_ptr->samples_vec[i].weight = 1.0 / max_samples_;
            sample_set_ptr->samples_vec[i].pose = pf_gaussian_pdf.GenerateSample();
        }
        sample_set_ptr->sample_count = samples_num;

//        static int32_t cnt;
//        cnt++;
//        std::cout << "Random by FixedPose" << cnt << "," << samples_num << std::endl;

        LOG_INFO << "Random by FixedPose, "<< samples_num;
    }

    void ParticleFilter::ClusterStatistics() {
        ClusterStatistics(this->sample_set_ptr_array_[0]);
    }
}