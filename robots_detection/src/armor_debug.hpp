#ifndef ARMOR_DEBUG_HPP
#define ARMOR_DEBUG_HPP
#include <opencv2/opencv.hpp>
cv::Mat Image0d;
cv::Mat Imaged;

//std::ofstream data("/home/zhanglixuan/data.txt");
/**
*  @Function : void draw_rect(cv::Mat &Image,cv::RotatedRect &box, cv::Scalar scalar)
*  @Description : particularly designed for DEBUG
*                draw rectangle the RotatedRect box
*/
void draw_rect(cv::Mat &Image, cv::RotatedRect &box, cv::Scalar scalar)
{
    cv::Point2f vertex[4];
    box.points(vertex);
    for (int k = 0; k < 4; ++k)
    {
        cv::line(Image, vertex[k], vertex[(k + 1) % 4], scalar);
    }
}
#endif