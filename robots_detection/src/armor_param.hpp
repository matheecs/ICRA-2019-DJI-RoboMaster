#ifndef ARMOR_PARAM_HPP
#define ARMOR_PARAM_HPP
#include <opencv2/opencv.hpp>
#include <fstream>

// release mode
#define RELEASE

// test mode
//#define TEST

// debug mode
//#define DEBUG

// deep debug mode
//#define DDBUG

// record video mode
//#define RECORD

// service_image mode
//#define SERVICE_I

// service_gimbal mode
//#define SERVICE_G

struct Ptz
{
public:
  float x_os;
  float y_os;
  float z_os;
  float pitch_os;
  float yaw_os;
};
struct Camera
{
public:
  cv::Mat IntrinsicMatrix;
  cv::Mat Distortion;
  std::vector<cv::Point3f> points_object_max;
  std::vector<cv::Point3f> points_object_min;
  std::vector<cv::Point3f> points_pos_max;
  std::vector<cv::Point3f> points_pos_min;
};
class Parameters
{
public:
  Parameters() = default;
  /**
     * @BLUE : 0
     * @RED : 1 
     */
  int enemy_color;
  int video;
  float area_max;
  float area_min;
  float angle_max;
  float angle_min;
  float ratio_max;
  float ratio_min;
  float ratio_g;
  float ratio_l;
  float armor_plat_width;
  float armor_plat_height;
  float armor_pos_width;
  float armor_pos_height;
  float weight_max;
  int blue_gray_min;
  int red_gray_min;
  int blue_color_min;
  int red_color_min;
  Ptz ptz;
  Camera camera;
};
Parameters Param;
#endif
