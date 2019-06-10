#ifndef ARMOR_IP_HPP
#define ARMOR_IP_HPP
#include <opencv2/opencv.hpp>
#include "armor_param.hpp"
#include "armor_debug.hpp"

//armor image preprocess

/**
*  @Function : void ipp(cv::Mat &InputImage, cv::Mat &OutputImage)
*  @Description : pick the right target by channels subtraction, Gray and 
*                 mask ROI                 
*/
void ipp(cv::Mat &InputImage, cv::Mat &OutputImage)
{
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 5));
    cv::Mat Gray, Ass;
    cv::cvtColor(InputImage, Gray, cv::COLOR_BGR2GRAY);
    cv::blur(Gray, Gray, cv::Size(2, 5));
    std::vector<cv::Mat> channels(3);
    cv::split(InputImage, channels);
    if (Param.enemy_color)
    {
        Gray = (Gray >= Param.red_gray_min);
        OutputImage = ((channels[2] - channels[1] / 2 - channels[0] / 2) > Param.red_color_min);
        Ass = ((channels[0] - channels[1] / 2 - channels[2] / 2) > Param.blue_color_min);
    }
    else
    {
        Gray = (Gray >= Param.blue_gray_min);
        OutputImage = ((channels[0] - channels[1] / 2 - channels[2] / 2) > Param.blue_color_min);
        Ass = ((channels[2] - channels[1] / 2 - channels[0] / 2) > Param.red_color_min);
    }
    //process enemy color
    cv::dilate(OutputImage, OutputImage, element);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(OutputImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point> scontour;
    for (auto contour : contours)
    {
        scontour.insert(scontour.end(), contour.begin(), contour.end());
    }
    cv::Rect rect = cv::boundingRect(scontour);
    cv::Mat ROI_OUT = OutputImage(rect);
    Gray(rect).copyTo(ROI_OUT);
#ifdef TEST
    cv::rectangle(Image0d, rect, cv::Scalar(255, 0, 0), 10);
#endif
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
            ROI_OUT = OutputImage(rect_ass) > 255;
        }
    }
}

//armor get lampbar
/**
*  @Function : bool judge_h(const float &angle, const cv::Size2f &size)
*  @Description : judge the RotatedRect vertically or horizontally
*                 samll function with extraoridinary effection 
*  @return : judge_h returns true when the RotatedRect is horizontal
*            judge_v returns true when the RotatedRect is vertically
*/
bool judge_h(const float &angle, const cv::Size2f &size)
{
    if (-angle > Param.angle_max)
    {
        return size.width < size.height;
    }
    if (-angle < Param.angle_min)
    {
        return size.width > size.height;
    }
    return 0;
}
bool judge_v(const float &angle, const cv::Size2f &size)
{
    if (-angle > Param.angle_max)
    {
        return size.width > size.height;
    }
    if (-angle < Param.angle_min)
    {
        return size.width < size.height;
    }
    return 0;
}
/**
*  @Function : float getRatio(cv::RotatedRect &box)
*  @Description : compute ratio between length and width, judge weather the box is horizontal
*  @Return : return the ratio if the box is horizontal, else return 0
*/
float getRatio(cv::RotatedRect &box)
{
    if (-box.angle > Param.angle_max)
    {
        return box.size.height / box.size.width;
    }
    if (-box.angle < Param.angle_min)
    {
        return box.size.width / box.size.height;
    }
    return 0;
}

struct Weight
{
    Weight() = default;
    Weight(size_t i, size_t j, float w) : index1(i), index2(j), weight(w) {}
    size_t index1;
    size_t index2;
    float weight;
};
/**
*  @Function : bool glb(cv::Mat &InputImage, std::vector<cv::RotatedRect> &boxs, std::vector<Weight> &boxs_weight)
*  @Description : select the right lampbars and boxs from the binaryzation image
*  @return : return possible armors and the sort value of these armors
*/

bool glb(cv::Mat &InputImage, std::vector<cv::RotatedRect> &boxs, std::vector<Weight> &boxs_weight, bool &cr_mode)
{
#ifdef DDBUG
    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "get lampbar:";
#endif
    boxs.clear();
    boxs_weight.clear();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(InputImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::RotatedRect> lampbars;
    cr_mode = 0;
    //get the correct lampbars
    if (contours.size() > 1)
    {
        float area;
        for (size_t i = 0; i < contours.size(); ++i)
        {
#ifdef DDBUG
            std::cout << "\n\t" << i + 1 << "  "
                      << "contoursArea  " << cv::contourArea(contours[i]);
#endif
            area = cv::contourArea(contours[i]);
            if (area > Param.area_min && area < Param.area_max)
            {
                cv::RotatedRect box = cv::minAreaRect(contours[i]);
#ifdef DDBUG
                std::cout << "\tangle  " << box.angle << std::endl;
                circle(InputImage, box.center, 3, cv::Scalar(155), -1, 8, 0);
                //putText(Image0d, "dis " +std::to_string(i) + std::to_string(box.center.y), cv::Point(25, 50+20*i), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 0));
#endif
                if (!cr_mode && box.center.y > 400)
                {
                    cr_mode = 1;
                }
                if (judge_v(box.angle, box.size))
                {
                    if (-box.angle > Param.angle_max)
                    {
                        box.angle = box.angle + 91; //in order to distinguish -90 and -0, plus 1
                        lampbars.push_back(std::move(box));
                    }
                    else
                    {
                        lampbars.push_back(std::move(box));
                    }
                }
            }
        }
    }
    if (!lampbars.empty())
    {
#ifdef DDBUG
        std::cout << "\nget the pre_plat:";
#endif
        bool exist;
        float ratio, ratio_g, ratio_l; //box ratio of length and width / two lampbars' ratio of gradient / two lampbars' ratio of length
        std::array<cv::Point, 4> v_lampbar;
        for (size_t i = 0; i < lampbars.size(); ++i)
        {
            exist = 0;
            for (size_t j = i + 1; j < lampbars.size(); ++j)
            {
                //attempt to save the RAM and improve running rate
                ratio = std::max(lampbars[i].size.width, lampbars[i].size.height);
                ratio_g = std::max(lampbars[j].size.width, lampbars[j].size.height);
                ratio_l = std::max(ratio_g, ratio) / std::min(ratio_g, ratio);
                if (ratio_l > Param.ratio_l)
                {
                    continue;
                }
                ratio_g = fabs(lampbars[i].angle - lampbars[j].angle);
#ifdef DDBUG
                std::cout << "\n\t* angle:  " << lampbars[i].angle << "," << lampbars[j].angle;
                std::cout << "\t\t ratio_g:  " << ratio_g;
                std::cout << "\t\t ratio_l:  " << ratio_l;
#endif
                if (ratio_g < Param.ratio_g)
                {
                    float sin_, cos_, theta;
                    if (lampbars[i].angle > 0)
                    {
                        theta = (lampbars[i].angle - 1) * CV_PI / 180; //eliminate the plus of 1
                        sin_ = lampbars[i].size.width * sinf(theta) / 2;
                        cos_ = lampbars[i].size.width * cosf(theta) / 2;
                        v_lampbar[0] = (cv::Point(round(lampbars[i].center.x + sin_), round(lampbars[i].center.y - cos_)));
                        v_lampbar[1] = (cv::Point(round(lampbars[i].center.x - sin_), round(lampbars[i].center.y + cos_)));
                    }
                    else
                    {
                        theta = (90 + lampbars[i].angle) * CV_PI / 180;
                        sin_ = lampbars[i].size.height * sinf(theta) / 2;
                        cos_ = lampbars[i].size.height * cosf(theta) / 2;
                        v_lampbar[0] = (cv::Point(round(lampbars[i].center.x + cos_), round(lampbars[i].center.y + sin_)));
                        v_lampbar[1] = (cv::Point(round(lampbars[i].center.x - cos_), round(lampbars[i].center.y - sin_)));
                    }
                    if (lampbars[j].angle > 0)
                    {
                        theta = (lampbars[j].angle - 1) * CV_PI / 180; //eliminate the plus of 1
                        sin_ = lampbars[j].size.width * sinf(theta) / 2;
                        cos_ = lampbars[j].size.width * cosf(theta) / 2;
                        v_lampbar[2] = (cv::Point(round(lampbars[j].center.x + sin_), round(lampbars[j].center.y - cos_)));
                        v_lampbar[3] = (cv::Point(round(lampbars[j].center.x - sin_), round(lampbars[j].center.y + cos_)));
                    }
                    else
                    {
                        theta = (90 + lampbars[j].angle) * CV_PI / 180;
                        sin_ = lampbars[j].size.height * sinf(theta) / 2;
                        cos_ = lampbars[j].size.height * cosf(theta) / 2;
                        v_lampbar[2] = (cv::Point(round(lampbars[j].center.x + cos_), round(lampbars[j].center.y + sin_)));
                        v_lampbar[3] = (cv::Point(round(lampbars[j].center.x - cos_), round(lampbars[j].center.y - sin_)));
                    }
                    cv::RotatedRect box = cv::minAreaRect(v_lampbar);
                    ratio = getRatio(box);
#ifdef DDBUG
                    std::cout << "\t\tratio:  " << ratio;
#endif
                    if (ratio && ratio > Param.ratio_min && ratio < Param.ratio_max)
                    {
#ifdef DDBUG
                        draw_rect(InputImage, box, cv::Scalar(155));
                        std::cout << "\tweight: " << (ratio_g / Param.ratio_g) * 0.75 + ((ratio_l - 1) / (Param.ratio_l - 1)) * (1 - 0.75)
                                  << "\t^_^\n";
#endif
                        ratio = (ratio_g / Param.ratio_g) * 0.8 + ((ratio_l - 1) / (Param.ratio_l - 1)) * (1 - 0.8);
                        if (exist)
                        {
                            if (ratio < boxs_weight.back().weight)
                            {
                                boxs.back() = box;
                                boxs_weight.back() = Weight(i, j, ratio);
                            }
                        }
                        else
                        {
                            exist = 1;
                            boxs.push_back(std::move(box));
                            boxs_weight.push_back(Weight(i, j, ratio));
                        }
                    }
                }
            }
        }
#ifdef DDBUG
        std::cout << std::endl;
#endif
    }
    else
    {
        return 0;
    }
    return 1;
}

//armor get plat

/**
*  @Function : float getDistance2(const cv::Point2f &point1, const cv::Point2f &point2)
*  @Description : compute the distance^2 between two points
*/
inline float getDistance2(const cv::Point2f &point1, const cv::Point2f &point2)
{
    return powf((point1.x - point2.x), 2) + powf((point1.y - point2.y), 2);
}
/**
*  @Function : int gpl(std::vector<cv::RotatedRect> &boxs, std::vector<Weight> &boxs_weight, cv::Point &point)
*  @Description : select the right boxs from the set of the boxs 
*                 and sort the right boxs in hash vector
*/
int gpl(std::vector<cv::RotatedRect> &boxs, std::vector<Weight> &boxs_weight, cv::Point &point)
{
    bool exist;
    std::vector<size_t> hash;
    for (size_t i = 0; i < boxs.size(); ++i)
    {
        exist = 0;
        if (boxs_weight[i].weight <= Param.weight_max)
        {
            for (size_t j = i + 1; j < boxs.size(); ++j)
            {
                if (boxs_weight[j].weight <= Param.weight_max)
                {
                    if (boxs_weight[i].index1 == boxs_weight[j].index1 || boxs_weight[i].index1 == boxs_weight[j].index2 || boxs_weight[i].index2 == boxs_weight[j].index1 || boxs_weight[i].index2 == boxs_weight[j].index2)
                    {
                        //exist = 1;
                        //
                        /*
                        if (boxs_weight[i].weight < boxs_weight[j].weight)
                        {
                            hash.push_back(i);
                            boxs_weight[j].weight = 1;
                        }
                        else
                        {
                            hash.push_back(j);
                            boxs_weight[i].weight = 1;
                            break;
                        }
                        */
                        //
                        if (boxs_weight[i].weight < boxs_weight[j].weight)
                        {
                            exist = 0;
                            boxs_weight[j].weight = 1;
                        }
                        else
                        {
                            exist = 1;
                            break;
                        }
                    }
                }
            }
        }
        else
        {
            exist = 1;
        }
        if (!exist)
        {
            hash.push_back(i);
        }
    }
    if (!hash.empty())
    {
#if defined(TEST) || defined(DEBUG)
        for (size_t i = 0; i < hash.size(); ++i)
        {
            draw_rect(Image0d, boxs[hash[i]], cv::Scalar(0, 255, 0));
        }
#endif
        size_t order = 0;
        if (point.x)
        {
            float distacne = getDistance2(boxs[hash[order]].center, point);
            float distance_;
            for (size_t i = 1; i < hash.size(); ++i)
            {
                distance_ = getDistance2(boxs[hash[i]].center, point);
                if (distance_ < distacne)
                {
                    order = i;
                    distacne = distance_;
                }
            }
            point = boxs[hash[order]].center;
            return hash[order];
        }
        else
        {
            for (size_t i = 1; i < hash.size(); ++i)
            {
                if (boxs[hash[i]].size.area() > boxs[hash[order]].size.area())
                {
                    order = i;
                }
            }
            point = boxs[hash[order]].center;
            return hash[order];
        }
    }
    else
    {
        point = cv::Point(0, 0);
        return -1;
    }
}
#endif