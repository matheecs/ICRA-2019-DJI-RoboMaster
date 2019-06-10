#ifndef ARMOR_HPP
#define ARMOR_HPP
#include <opencv2/opencv.hpp>
#include "armor_param.hpp"
#include "armor_ip.hpp"
#include "armor_pos.hpp"
#include "armor_pre.hpp"
#include "armor_mode.hpp"
#if defined(TEST) || defined(DEBUG)
#include "armor_debug.hpp"
#endif
class Armor
{
public:
  Armor() = default;
  cv::Mat &armor_img() { return Image0; }
  void armor_ipp();
  void armor_glb();
  void armor_gpl();
  void armor_pos();
  void armor_pre();
  void armor_mode();
  Inform inform;
  Pos pos;
  double time_d;
  Yaw_pd yaw_pd;

private:
  cv::Mat Image0;
  cv::Mat Image;
  std::vector<cv::RotatedRect> boxs;
  std::vector<Weight> boxs_weight;
  int index;
  cv::Point point;
  cv::Point3f position;
  cv::Point3f position_map;
  size_t pos_count;
  // close range mode
  bool cr_mode;
  std::deque<float> deque_yaw;
  Deque_mode deque_mode;
};

Armor armor;

void Armor::armor_ipp()
{
  ipp(this->Image0, this->Image);
#if defined(TEST) || defined(DEBUG)
  Image0d = this->Image0;
  Imaged = this->Image;
#endif
}
void Armor::armor_glb()
{
  if (glb(this->Image, this->boxs, this->boxs_weight, this->cr_mode))
  {
    this->pos.pos_mode = 1;
  }
  else
  {
    this->pos.pos_mode = 0;
  }
}
void Armor::armor_gpl()
{
  this->index = gpl(this->boxs, this->boxs_weight, this->point);
#if defined(TEST) || defined(DEBUG)
  if (this->index >= 0)
  {
    circle(Image0d, this->boxs[this->index].center, 3, cv::Scalar(0, 0, 255), -1, 8, 0);
#ifdef DEBUG
    std::cout << "weight:  " << boxs_weight[this->index].weight << std::endl;
#endif
  }
#endif
}
void Armor::armor_pos()
{
  if (this->index >= 0)
  {
    pos_ident(this->boxs[this->index], this->position, Param.camera.points_object_max, Param.camera.points_object_min, 2);
    ptz_ct(this->position, this->inform);
    pos_ident(this->boxs[this->index], this->position_map, Param.camera.points_pos_max, Param.camera.points_pos_min, 5);
    this->inform.pitch_mode = 0;
    this->inform.yaw_mode = 1;
    this->inform.distance = sqrtf(powf(this->position_map.x, 2) + powf(this->position_map.y, 2) + powf(this->position_map.z, 2));
    if (fabs(this->inform.yaw) > 1)
    {
      this->inform.yaw = 0;
    }
    if (fabs(this->inform.pitch) > 1)
    {
      this->inform.pitch = 0;
      this->inform.pitch_mode = 1;
    }
    this->pos.pos_mode = 1;
    if (this->position_map.z < 8)
    {
      this->pos.pos_x = this->position_map.z;
      this->pos.pos_y = -this->position_map.x;
      this->pos_count = 0;
    }

    if (!this->pos.pos_x)
    {
      this->pos.pos_mode = 0;
      this->pos_count++;
    }
  }
  else
  {
    this->inform.pitch = 0;
    this->inform.yaw = 0;
    this->inform.pitch_mode = 1;
    this->inform.yaw_mode = 1;
    this->inform.distance = sqrtf(powf(this->position_map.x, 2) + powf(this->position_map.y, 2) + powf(this->position_map.z, 2));
    if (this->cr_mode)
    {
      this->pos.pos_mode = 1;
      this->pos.pos_x = 0.5;
      this->pos.pos_y = 0;
    }
    else
    {
      if (this->position_map.z)
      {
        if (this->pos_count++ < 10) // 10 pos_decsion delay
        {
          this->pos.pos_mode = 1;
          this->pos.pos_x = this->position_map.z;
          this->pos.pos_y = -this->position_map.x;
        }
        else
        {
          this->pos.pos_mode = 0;
          this->pos.pos_x = 0;
          this->pos.pos_y = 0;
          this->position_map = cv::Point3f(0, 0, 0);
        }
      }
      else
      {
        this->pos.pos_mode = 0;
        this->pos.pos_x = 0;
        this->pos.pos_y = 0;
      }
    }
  }
}
void Armor::armor_pre()
{
  if (this->yaw_pd.yaw_delay && this->yaw_pd.yaw_delay++ < 20)
  {
    this->yaw_pd.yaw_pd_mode = 0;
  }
  else
  {
    if (fabs(this->inform.yaw) > 0.085)
    {
      this->deque_yaw.clear();
      this->yaw_pd.yaw_pd_mode = 0;
      this->yaw_pd.yaw_delay = 1;
      this->yaw_pd.yaw_pd = 0;
    }
    else
    {
      this->yaw_pd.yaw_pd_mode = 1;
      this->deque_yaw.push_back(this->inform.yaw);
      if (this->deque_yaw.size() == 5)
      {
        this->yaw_pd.yaw_pd += yaw_pre(this->deque_yaw);
        deque_yaw.clear();
      }
    }
  }
}
void Armor::armor_mode()
{
  this->deque_mode.push(this->inform.yaw);
  if (this->deque_mode.deque_mode.size() == this->deque_mode.max_size)
  {
    // this->deque_mode.circle_mode();
    //this->deque_mode.twist_mode();
  }
}
#endif //ARMOR_HPP
