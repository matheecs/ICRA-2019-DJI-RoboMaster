//
// Created by cxn on 19-2-15.
//

#include "amcl_map.h"
namespace leonard_localization {
    void AmclMap::ConvertFromMsg(const nav_msgs::OccupancyGrid &map_msg) {
        this->size_x_ = map_msg.info.width; //163
        this->size_y_ = map_msg.info.height;  //103
        this->scale_ = map_msg.info.resolution; //0.05
        this->origin_x_ = map_msg.info.origin.position.x + (this->size_x_ / 2) * this->scale_; //4.05
        this->origin_y_ = map_msg.info.origin.position.y + (this->size_y_ / 2) * this->scale_; //2.55
        this->cells_vec_.resize(this->size_x_ * this->size_y_);
        LOG_INFO << "max x " << (this->size_x_) * scale_ << " max y " << (this->size_y_) * scale_;

        //有无障碍标注
        for (int i = 0; i < this->size_x_ * this->size_y_; i++) {
            auto tmp_msg = static_cast<int>(map_msg.data[i]);
            if (tmp_msg == 0) {
                this->cells_vec_[i].occ_state = -1;
            } else if (tmp_msg == 100) {
                this->cells_vec_[i].occ_state = +1;
            } else {
                this->cells_vec_[i].occ_state = 0;
            }
        }
    }

    void AmclMap::BuildDistanceMap(double scale, double max_dist) {
        cached_distance_map_.reset();// 智能指针引用次数减1，且cached_distance_map_置为nullptr
        if (cached_distance_map_ == nullptr || cached_distance_map_->scale_ != scale || cached_distance_map_->max_dist_) {
            if (cached_distance_map_ != nullptr) {
                cached_distance_map_.reset();
                //讲道理，怎么可能走到这一步？
                std::cout<<"leonard is a dog"<<std::endl;
            }
            cached_distance_map_ = std::make_unique<CachedDistanceMap>(scale, max_dist);
        }
    }

    void AmclMap::Enqueue(int i, int j, int src_i, int src_j, CellDataPriorityQueue &Q) {

        auto index = ComputeCellIndexByMap(i, j);
        if (mark_vec_->at(index)) {
            return;
        }

        int di = std::abs(i - src_i);
        int dj = std::abs(j - src_j);
        double distance = cached_distance_map_->distances_mat_[di][dj];

        if (distance > cached_distance_map_->cell_radius_) {
            return;
        }

        this->cells_vec_[index].occ_dist = distance * this->scale_;

        CellData cell_data(static_cast<unsigned int>(i),
                           static_cast<unsigned int>(j),
                           static_cast<unsigned int>(src_i),
                           static_cast<unsigned int>(src_j),
                           std::bind(&AmclMap::GetCellOccDistByCoord, this, std::placeholders::_1, std::placeholders::_2));
        Q.push(cell_data);
        mark_vec_->at(index) = 1;

    }

    int AmclMap::ComputeCellIndexByMap(const int &i, const int &j) {
        return i + j * this->size_x_;
    };

    const double &AmclMap::GetCellOccDistByIndex(int cell_index) {
        return cells_vec_[cell_index].occ_dist;
    }

    double AmclMap::GetCellOccDistByCoord(unsigned i, unsigned j) {
        return GetCellOccDistByIndex(ComputeCellIndexByMap(i, j));
    }

    void AmclMap::UpdateCSpace(double max_occ_dist) {

        mark_vec_ = std::make_unique<std::vector<unsigned char>>();
        mark_vec_->resize(this->size_x_ * this->size_y_);
        CellDataPriorityQueue Q;//通过格子的障碍物距离，从小到大排序
        this->max_occ_dist_ = max_occ_dist;
        BuildDistanceMap(this->scale_, this->max_occ_dist_);
        //cached_distance_map_指向了一个新的CachedDistanceMap对象

        // Enqueue all the obstacle cells
        CellData cell(0,
                      0,
                      0,
                      0,
                      std::bind(&AmclMap::GetCellOccDistByCoord, this, std::placeholders::_1, std::placeholders::_2));
        //绑定的回调是这个格子的障碍物距离

        for (int i = 0; i < this->size_x_; i++) {
            cell.src_i_ = cell.i_ = i;
            for (int j = 0; j < size_y_; j++) {
                auto map_index_tmp = ComputeCellIndexByMap(i, j);
                if (this->cells_vec_[map_index_tmp].occ_state == +1) {
//				cell.occ_dist_ = this->cells_vec_[map_index_tmp].occ_dist = 0.0;
                    this->cells_vec_[map_index_tmp].occ_dist = 0.0;
                    cell.src_j_ = cell.j_ = j;
                    mark_vec_->at(map_index_tmp) = 1;
                    Q.push(cell);
                }
                else {
//				cell.occ_dist_ = this->cells_vec_[map_index_tmp].occ_dist = max_occ_dist;
                    this->cells_vec_[map_index_tmp].occ_dist = max_occ_dist;
                }
            }
        }

        while (!Q.empty()) {
            CellData current_cell_data = Q.top();
            if (current_cell_data.i_ > 0) {
                Enqueue(current_cell_data.i_ - 1, current_cell_data.j_,
                        current_cell_data.src_i_, current_cell_data.src_j_,
                        Q);
            }
            if (current_cell_data.j_ > 0) {
                Enqueue(current_cell_data.i_, current_cell_data.j_ - 1,
                        current_cell_data.src_i_, current_cell_data.src_j_,
                        Q);
            }
            if (current_cell_data.i_ < this->size_x_ - 1) {
                Enqueue(current_cell_data.i_ + 1, current_cell_data.j_,
                        current_cell_data.src_i_, current_cell_data.src_j_,
                        Q);
            }
            if (current_cell_data.j_ < this->size_y_ - 1) {
                Enqueue(current_cell_data.i_, current_cell_data.j_ + 1,
                        current_cell_data.src_i_, current_cell_data.src_j_,
                        Q);
            }
            Q.pop();
        }

        cached_distance_map_.reset();
        mark_vec_.reset();
    }

    AmclMap::~AmclMap() {
        cells_vec_.clear();
        cells_vec_.shrink_to_fit();
    }

    const nav_msgs::OccupancyGrid &AmclMap::ConvertDistanMaptoMapMsg() {
        if (!distance_map_init_) {
            distance_map_msg_.header.frame_id = "map";
            distance_map_msg_.info.width = this->size_x_;
            distance_map_msg_.info.height = this->size_y_;
            distance_map_msg_.info.resolution = this->scale_;
            distance_map_msg_.info.origin.position.x = this->origin_x_ - (this->size_x_ / 2) * this->scale_;
            distance_map_msg_.info.origin.position.y = this->origin_y_ - (this->size_y_ / 2) * this->scale_;
            distance_map_msg_.data.resize(this->size_x_ * this->size_y_);
            for (int i = 0; i < this->size_x_ * this->size_y_; i++) {
                distance_map_msg_.data[i] =
                        (static_cast<int8_t >((cells_vec_[i].occ_dist) / (max_occ_dist_) * 100.0 / this->scale_));
                // 2m/2m*100/0.05=2000mm
                // 1m/2m*100/0.05=1000mm
            }
            distance_map_init_ = true;
        }

        return distance_map_msg_;
    }

    const double &AmclMap::GetMaxOccDist() const {
        return max_occ_dist_;
    }


    void AmclMap::ConvertWorldCoordsToMapCoords(const double &x,
                                                const double &y,
                                                int &mx,
                                                int &my) {
        mx = (std::floor((x - this->origin_x_) / this->scale_ + 0.5) + this->size_x_ / 2);
        my = (std::floor((y - this->origin_y_) / this->scale_ + 0.5) + this->size_y_ / 2);
    }

    bool AmclMap::CheckMapCoordsValid(const int &i, const int &j) {
        return ((i >= 0) && (i < this->size_x_) && (j >= 0) && (j < this->size_y_));
    };

    bool AmclMap::CheckIndexFree(int i, int j) {
        if (this->cells_vec_[ComputeCellIndexByMap(i, j)].occ_state == -1) {
            return true;
        } else {
            return false;
        }
    }

    void AmclMap::ConvertMapCoordsToWorldCoords(const int &x,
                                                const int &y,
                                                double &wx,
                                                double &wy) {
        wx = (this->origin_x_ + ((x) - this->size_x_ / 2) * this->scale_);
        wy = (this->origin_y_ + ((y) - this->size_y_ / 2) * this->scale_);
    }

    CachedDistanceMap::CachedDistanceMap(double scale,
                                         double max_dist) {
        scale_ = scale;//默认值为0.05
        max_dist_ = max_dist;//默认值为2.0
        cell_radius_ = static_cast<int>(max_dist / scale);
        distances_mat_.resize(cell_radius_ + 2);
        for (auto it = distances_mat_.begin(); it != distances_mat_.end(); ++it) {
            it->resize(cell_radius_ + 2, max_dist_);
        }

        //若cell_radius为40
        //distances_mat_长度42*42，每个值为2.0

        for (int i = 0; i < cell_radius_ + 2; i++) {
            for (int j = 0; j < cell_radius_ + 2; j++) {
                distances_mat_[i][j] = std::sqrt(i * i + j * j);
            }
        }
    }
}