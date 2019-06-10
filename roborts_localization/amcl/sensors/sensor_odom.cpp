//
// Created by cxn on 19-2-15.
//

#include "sensor_odom.h"

namespace leonard_localization {
    SensorOdom::SensorOdom(double alpha1,
                           double alpha2,
                           double alpha3,
                           double alpha4,
                           double alpha5) {
        alpha1_ = alpha1;
        alpha2_ = alpha2;
        alpha3_ = alpha3;//可以看做是标准差是移动距离的sqrt(alpha3)倍
        alpha4_ = alpha4;
        alpha5_ = alpha5;
        odom_model_type_ = ODOM_MODEL_OMNI;
    }

    void SensorOdom::setSensorOdom(double alpha1,
                                   double alpha2,
                                   double alpha3,
                                   double alpha4,
                                   double alpha5) {
        alpha1_ = alpha1;
        alpha2_ = alpha2;
        alpha3_ = alpha3;//可以看做是标准差是移动距离的sqrt(alpha3)倍
        alpha4_ = alpha4;
        alpha5_ = alpha5;
    }


// 机器人用的运动模型是ODOM_MODEL_OMNI，《概率机器人》中程序5.6介绍的是ODOM_MODEL_DIFF
    bool SensorOdom::UpdateAction(SampleSetPtr sample_set_ptr, const SensorOdomData &odom_data) {

        DLOG_INFO << "Compute the new sample poses by motion model";

        Vec3d old_pose = (odom_data.pose) - (odom_data.delta);
        double delta_bearing;
        double delta_trans_hat, delta_rot_hat, delta_strafe_hat;
        double delta_trans = std::sqrt(
                odom_data.delta[0] * odom_data.delta[0] + odom_data.delta[1] * odom_data.delta[1]);
        double delta_rot = odom_data.delta[2];

        double trans_hat_stddev = std::sqrt(alpha3_ * (delta_trans * delta_trans) + alpha1_ * (delta_rot * delta_rot));
        double rot_hat_stddev = std::sqrt(alpha4_ * (delta_rot * delta_rot) + alpha2_ * (delta_trans * delta_trans));
        double strafe_hat_stddev = std::sqrt(alpha1_ * (delta_rot * delta_rot) + alpha5_ * (delta_trans * delta_trans));

        for (int i = 0; i < sample_set_ptr->sample_count; i++) {

            delta_bearing = angle_diff<double>(std::atan2(odom_data.delta(1),
                                                          odom_data.delta(0)),
                                               old_pose(2)) + sample_set_ptr->samples_vec[i].pose(2);

            double cs_bearing = std::cos(delta_bearing);
            double sn_bearing = std::sin(delta_bearing);

            delta_trans_hat = delta_trans + RandomGaussianNumByStdDev<double>(trans_hat_stddev);
            delta_rot_hat = delta_rot + RandomGaussianNumByStdDev<double>(rot_hat_stddev);
            delta_strafe_hat = 0 + RandomGaussianNumByStdDev<double>(strafe_hat_stddev);

            sample_set_ptr->samples_vec[i].pose[0] += (delta_trans_hat * cs_bearing + delta_strafe_hat * sn_bearing);
            sample_set_ptr->samples_vec[i].pose[1] += (delta_trans_hat * sn_bearing - delta_strafe_hat * cs_bearing);
            sample_set_ptr->samples_vec[i].pose[2] += delta_rot_hat;

        }
        return true;
    }
}