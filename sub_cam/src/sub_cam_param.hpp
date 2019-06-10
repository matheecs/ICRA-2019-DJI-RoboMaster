#ifndef SUB_CAM_PARAM_HPP
#define SUM_CAM_PARAM_HPP
#include <opencv2/opencv.hpp>
#include <fstream>

// release mode
#define RELEASE

// test mode
//#define TEST

// debug mode
//#define DEBUG

// record video mode
#define RECORD

// service_image mode
#define SERVICE_I
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
  int gray_min;
  int color_min;
  int red_ap_min;
  int blue_ap_min;
};

Parameters Param;
#endif
