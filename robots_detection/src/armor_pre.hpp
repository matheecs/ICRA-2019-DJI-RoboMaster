#ifndef ARMOR_PRE_HPP
#define ARMOR_PRE_HPP
#include <opencv2/opencv.hpp>
struct Yaw_pd
{
    Yaw_pd() : yaw_pd(0), yaw_pd_mode(0), yaw_delay(0) {}
    float yaw_pd;
    bool yaw_pd_mode;
    int yaw_delay;
};
/**
*  @Function : float yaw_pre(std::deque<float> &deque_yaw) 
*  @Description : calcullate the motion offset of yaw              
*/
float yaw_pre(std::deque<float> &deque_yaw)
{
    size_t size = deque_yaw.size();
    float delta_yaw = 0;
    for (auto yaw : deque_yaw)
    {
        delta_yaw += yaw;
    }
    return delta_yaw /= deque_yaw.size();
}
#endif