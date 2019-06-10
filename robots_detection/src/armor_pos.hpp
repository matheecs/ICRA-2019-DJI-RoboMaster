#ifndef ARMOR_POS_HPP
#define ARMOR_POS_HPP
#include <opencv2/opencv.hpp>
#include "armor_param.hpp"
#include "armor_debug.hpp"
/**
*  @Function : void pos_ident(cv::RotatedRect &box, cv::Point3f &position)
*  @Description : measure the distance between self and enemy armor
*                 by using cv::solvePnP
*                 a new edition ,add the offset 
*  @return : distance between self and enemy armor
*/
void pos_ident(cv::RotatedRect &box, cv::Point3f &position, std::vector<cv::Point3f> &pnp_max, std::vector<cv::Point3f> &pnp_min, size_t pnp_method)
{
    cv::Point2f vertex[4]; //four points
    std::array<cv::Point2f, 4> vertices;
    cv::Mat TVEC = cv::Mat_<float>(3, 1);
    cv::Mat RVEC = cv::Mat_<float>(3, 3);
    box.points(vertex);
    if (-box.angle > Param.angle_max)
    {
        vertices[0] = vertex[0];
        vertices[1] = vertex[1];
        vertices[2] = vertex[2];
        vertices[3] = vertex[3];
        solvePnP(pnp_max, vertices, Param.camera.IntrinsicMatrix, Param.camera.Distortion, RVEC, TVEC, false, pnp_method);
    }
    else if (-box.angle < Param.angle_min)
    {
        vertices[0] = vertex[0];
        vertices[1] = vertex[1];
        vertices[2] = vertex[2];
        vertices[3] = vertex[3];
        solvePnP(pnp_min, vertices, Param.camera.IntrinsicMatrix, Param.camera.Distortion, RVEC, TVEC, false, pnp_method);
    }
    else
    {
        return;
    }
    position = cv::Point3f(TVEC);
#ifdef DEBUG
    std::cout << "position estimate:"
              << "\n\tx         " << position.x
              << "\n\ty         " << position.y
              << "\n\tz         " << position.z
              << "\n\tdistacne  "
              << sqrtf(powf(position.x, 2) + powf(position.y, 2) + powf(position.z, 2))
              << std::endl;
    putText(Image0d, "DISTANCE " + std::to_string(sqrtf(powf(position.x, 2) + powf(position.y, 2) + powf(position.z, 2))) + "m", cv::Point(25, 25), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 0));
#endif
}

struct Inform
{
  public:
    Inform() : pitch_mode(0), yaw_mode(0), pitch(0), yaw(0), distance(0) {}
    bool pitch_mode;
    bool yaw_mode;
    float pitch;
    float yaw;
    float distance;
};

/**
*  @Function : void ptz_ct(cv::Point2f &position,Inform &inform)
*  @Description : commpute the pitch and yaw of gimbal based on the position(TVEC)
*  @return : return picth and yaw by reference type
*/

void ptz_ct(cv::Point3f &position, Inform &inform)
{
    inform.pitch = -(float)(atan2(-(position.y + Param.ptz.y_os), position.z + Param.ptz.z_os)) + (float)(Param.ptz.pitch_os * CV_PI / 180);
    inform.yaw = -(float)(atan2(position.x + Param.ptz.x_os, position.z + Param.ptz.z_os)) + (float)(Param.ptz.yaw_os * CV_PI / 180);
#if defined(DEBUG)
    std::cout << "gimbal_angle:\n\tpitch\t" << inform.pitch
              << "\n\tyaw\t" << inform.yaw << std::endl;
#endif
}
struct Pos
{
    Pos() : pos_x(), pos_y(0), pos_mode(0) {}
    float pos_x;
    float pos_y;
    bool pos_mode;
};

#endif