//
// Created by cxn on 19-2-15.
//

#ifndef PROJECT_SENSOR_ODOM_H
#define PROJECT_SENSOR_ODOM_H

#include "log.h"
#include "particle_filter/particle_filter.h"
#include "localization_math.h"

namespace leonard_localization {
    enum OdomModel{
        ODOM_MODEL_DIFF = 0,
        ODOM_MODEL_OMNI = 1
    };

    class SensorOdomData{
    public:
        Vec3d pose;
        Vec3d delta;
    };

    class SensorOdom{
    public:
        SensorOdom(double alpha1,
                   double alpha2,
                   double alpha3,
                   double alpha4,
                   double alpha5);

        bool UpdateAction(SampleSetPtr sample_set_ptr, const SensorOdomData &odom_data);

        void setSensorOdom(double alpha1,
                        double alpha2,
                        double alpha3,
                        double alpha4,
                        double alpha5);

    private:
        /**
        * @brief Model type
        */
        OdomModel odom_model_type_;

        /**
         * @brief Drift parameters
         */
        double alpha1_;
        double alpha2_;
        double alpha3_;
        double alpha4_;
        double alpha5_;
    };
}


#endif //PROJECT_SENSOR_ODOM_H
