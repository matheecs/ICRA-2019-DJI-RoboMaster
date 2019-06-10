//
// Created by cxn on 19-2-15.
//
#ifndef PROJECT_AMCL_MAP_H
#define PROJECT_AMCL_MAP_H

#include <boost/function.hpp>
#include <queue>
#include <nav_msgs/OccupancyGrid.h>
#include "log.h"

namespace leonard_localization{
    class Cell{
    public:
        //! Occupancy state (-1 = free, 0 = unknown, +1 = occ)
        int occ_state = 0;
        //! Distance to the nearest occupied cell
        double occ_dist = 0;
    };

    class CellData {
    public:
        CellData(unsigned int i,
                 unsigned int j,
                 unsigned int src_i,
                 unsigned int src_j,
                 const std::function<double(unsigned int, unsigned int)> &GetOccDistFunc)
                :i_(i),j_(j),src_i_(src_i),src_j_(src_j),GetOccDist(GetOccDistFunc){
        }

    public:
        std::function<double(unsigned int, unsigned int)> GetOccDist;
        unsigned int i_ = 0;
        unsigned int j_ = 0;
        unsigned int src_i_ = 0;
        unsigned int src_j_ = 0;
    };

    class CachedDistanceMap {
    public:
        CachedDistanceMap(double scale, double max_dist);
    public:
        double scale_ = 0, max_dist_ = 0;
        int cell_radius_ = 0;
        std::vector<std::vector<double>> distances_mat_;
    };

    class CompareByOccDist {
    public:
        bool operator()(const CellData &a, const CellData &b) {
            return a.GetOccDist(a.i_, a.j_) > b.GetOccDist(b.i_, b.j_);
        };
    };

    using CellDataPriorityQueue = std::priority_queue<CellData, std::vector<CellData>, CompareByOccDist>;

    class AmclMap{
    public:
        /**
        * @brief Convert static map message to AmclMap
        * @param map_msg Static map message
        */
        void ConvertFromMsg(const nav_msgs::OccupancyGrid &map_msg);

        /**
        * @brief Update the cspace distance values, used by LikelihoodFiledProbe model
        * @param max_occ_dist Max distance at which we care about obstacles
        */
        void UpdateCSpace(double max_occ_dist);

        /**
        * @brief Compute cell index by map coords
        * @param i Map coords i
        * @param j Map coords j
        * @return Returns cell index
        */
        int ComputeCellIndexByMap(const int &i, const int &j);

        const double &GetCellOccDistByIndex(int cell_index);

        double GetCellOccDistByCoord(unsigned i, unsigned j);

        ~AmclMap();

        /**
        * @brief Get map size x
        * @return Returns map size x
        */
        int GetSizeX() const {
            return size_x_;
        }

        /**
        * @brief Get map size y
        * @return Returns map size y
        */
        int GetSizeY() const {
            return size_y_;
        }

        /**
         * @brief Get max distance at which we care about obstacles
         * @return Max distance at which we care about obstacles
         */
        const double &GetMaxOccDist() const;

        /**
         * @brief Convert world coords to map coords
         * @param x World coords x
         * @param y World coords y
         * @param mx Map coords x
         * @param my Map coords y
         */
        void ConvertWorldCoordsToMapCoords(const double &x, const double &y, int &mx, int &my);

        /**
         * @brief Check map coords valid
         * @param i Map coords i
         * @param j Map coords j
         * @return
         */
        bool CheckMapCoordsValid(const int &i, const int &j);

        const nav_msgs::OccupancyGrid &ConvertDistanMaptoMapMsg();

        /**
         * @brief Check cell is free (occ_state == -1)
         * @param i Map coords i
         * @param j Map coords j
         * @return
         */
        bool CheckIndexFree(int i, int j);

        /**
         * @brief Convert world coords to map coords
         * @param x Map coords x
         * @param y Map coords y
         * @param wx World coords x
         * @param wy World coords y
         */
        void ConvertMapCoordsToWorldCoords(const int &x, const int &y,
                                           double &wx, double &wy);


    private:

        void Enqueue(int i, int j, int src_i, int src_j, CellDataPriorityQueue &Q);

        void BuildDistanceMap(double scale, double max_dist);

    private:
        /**
        * @brief The map data, stored as a grid
        */
        std::vector<Cell> cells_vec_;

        /**
         * @brief Map origin; the map is a viewport onto a conceptual larger map.
         */
        double origin_x_ = 0, origin_y_ = 0;

        /**
         * @brief Map scale (m/cell)
         */
        double scale_ = 0;

        /**
         * @brief Map dimensions (number of cells)
         */
        int size_x_ = 0, size_y_ = 0;

        /**
        * @brief The static distance map data
        */
        std::unique_ptr<CachedDistanceMap> cached_distance_map_;

        std::unique_ptr<std::vector<unsigned char>> mark_vec_;

        /**
        * @brief Max distance at which we care about obstacles, for constructing likelihood field.
        */
        double max_occ_dist_ = 0;

        /**
        * @brief The static distance map messages data
        */
        nav_msgs::OccupancyGrid distance_map_msg_;

        bool distance_map_init_ = false;
    };
}
#endif //PROJECT_AMCL_MAP_H
