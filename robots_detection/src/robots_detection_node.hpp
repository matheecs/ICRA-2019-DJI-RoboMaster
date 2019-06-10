#ifndef ROBOTS_DETECTION_HPP
#define ROBOTS_DETECTION_HPP
#include "armor.hpp"
#ifdef RELEASE
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/EnemyDetect.h"
#endif
#ifdef RELEASE
void main_f(cv::Mat &Image, roborts_msgs::GimbalAngle &ptz_ct, Pos &pos_ct)
#endif
#ifndef RELEASE
    void main_f(cv::Mat &Image)
#endif
{
    if (Image.empty())
    {
        std::cout << "EMPTY IMAGE!" << std::endl;
        return;
    }
    armor.armor_img() = Image.clone();
    armor.armor_ipp();
    armor.armor_glb();
    armor.armor_gpl();
    armor.armor_pos();
    armor.armor_pre();
    //armor.armor_mode();
#ifdef RELEASE
    ptz_ct.yaw_mode = armor.inform.yaw_mode;
    ptz_ct.pitch_mode = armor.inform.pitch_mode;
    ptz_ct.yaw_angle = armor.inform.yaw;
    //ptz_ct.yaw_angle = armor.yaw_pd.yaw_pd_mode ? armor.inform.yaw + armor.yaw_pd.yaw_pd * 0.5 : armor.inform.yaw;
    ptz_ct.pitch_angle = armor.inform.pitch;
    ptz_ct.distance = armor.inform.distance;
    pos_ct.pos_mode = armor.pos.pos_mode;
    pos_ct.pos_x = armor.pos.pos_x;
    pos_ct.pos_y = armor.pos.pos_y;
#endif
#ifdef DEBUG
    std::cout << "pos_decesion----\n"
              << "pos_mode\t" << armor.pos.pos_mode
              << "\npos_depth\t" << armor.pos.pos_x << std::endl;
#endif
#if defined(TEST) || defined(DEBUG)
    cv::imshow("armor", Image0d);
#endif
#ifdef DEBUG
    cv::imshow("armor_", Imaged);
#endif
#if defined(TEST) || defined(DEBUG)
    if (cv::waitKey(1) == 27)
    {
        cv::waitKey(0);
    }
#endif
}
#endif