#ifndef SUB_CAMERA_NODE_HPP
#define SUB_CAMERA_NODE_HPP
#include <opencv2/opencv.hpp>
#include "sub_cam_param.hpp"
bool judge_v(const float &angle, const cv::Size2f &size)
{
    bool sign = 0;
    if (-angle > Param.angle_max)
    {
        size.width > size.height ? sign = 1 : sign = 0;
    }
    else if (-angle < Param.angle_min)
    {
        size.width < size.height ? sign = 1 : sign = 0;
    }
    return sign;
}
bool main_f(cv::Mat &Image)
{
#if defined(TEST) || defined(DEBUG)
    cv::imshow("sub_cam", Image);
#endif
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 5));
    cv::Mat Image_;
    cv::Mat Gray, Ass;
    cv::cvtColor(Image, Gray, cv::COLOR_BGR2GRAY);
    Gray = (Gray >= Param.gray_min);
    //std::array<cv::Mat, 3> channels;
    std::vector<cv::Mat> channels(3);
    cv::split(Image, channels);
    if (Param.enemy_color)
    {
        Image_ = ((channels[2] - channels[1]) > Param.color_min);
        Ass = ((channels[0] - channels[1]) > Param.color_min);
    }
    else
    {
        Image_ = ((channels[0] - channels[1]) > Param.color_min);
        Ass = ((channels[2] - channels[1]) > Param.color_min);
    }
    //process enemy color
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(Image_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point> scontour;
    for (auto contour : contours)
    {
        scontour.insert(scontour.end(), contour.begin(), contour.end());
    }
    cv::Rect rect = cv::boundingRect(scontour);
    cv::Mat ROI_OUT = Image_(rect);
    Gray(rect).copyTo(ROI_OUT);
    //elimate self color
    contours.clear();
    cv::findContours(Ass, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (!contours.empty())
    {
        scontour.clear();
        for (auto contour : contours)
        {
            scontour.insert(scontour.end(), contour.begin(), contour.end());
        }
        cv::Rect rect_ass = cv::boundingRect(scontour);
        if (rect_ass.area() < rect.area())
        {
            ROI_OUT = Image_(rect_ass) > 255;
        }
    }
    contours.clear();
#ifdef DEBUG
    cv::imshow("sub_cam_", Image_);
#endif
    cv::findContours(Image_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::RotatedRect> lampbars;
    if (!contours.empty())
    {
        float area;
        for (size_t i = 0; i < contours.size(); ++i)
        {
            area = cv::contourArea(contours[i]);
            if (area > Param.area_min && area < Param.area_max)
            {
                cv::RotatedRect box = cv::minAreaRect(contours[i]);
                if (judge_v(box.angle, box.size))
                {
                    return 1;
                }
            }
        }
    }
    return 0;
}
#endif