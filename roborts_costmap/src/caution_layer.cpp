#include "caution_layer.h"
#include "caution_layer_setting.pb.h"

namespace roborts_costmap{

void CautionLayer::OnInitialize()
{
  ros::NodeHandle nh;
  ParaCautionLayer para_caution_layer;
  std::string caution_layer = ros::package::getPath("roborts_costmap") + \
      "/config/caution_layer_config.prototxt";
  roborts_common::ReadProtoFromTextFile(caution_layer.c_str(), &para_caution_layer);
  
  global_frame_ = layered_costmap_-> GetGlobalFrameID();
  caution_cost_value_ = roborts_costmap::CAUTION_ZONE;
  caution_cost_value_ = para_caution_layer.caution_cost_value();
  caution_center_x_ = para_caution_layer.caution_zone().center_x();
  caution_center_y_ = para_caution_layer.caution_zone().center_y();
  caution_width_ = para_caution_layer.caution_zone().width();
  caution_height_ = para_caution_layer.caution_zone().height();
  is_world_ = para_caution_layer.is_world();
  initial_pose_x_ = para_caution_layer.initial_pose_x();
  initial_pose_y_ = para_caution_layer.initial_pose_y(); 
  initial_pose_a_ = para_caution_layer.initial_pose_a();

}
void CautionLayer::changeInitPose(tf::Stamped<tf::Pose> const & initial_pose){
  initial_pose_x_ = initial_pose.getOrigin().getX();
  initial_pose_y_ = initial_pose.getOrigin().getY();
  cv::Mat R_3f, V_3f;
  auto mat = initial_pose.getBasis();
  R_3f = (cv::Mat_<float>(3,3) << mat.getRow(0).getX(), mat.getRow(0).getY(), mat.getRow(0).getZ(), 
                              mat.getRow(1).getX(), mat.getRow(1).getY(), mat.getRow(1).getZ(), 
                              mat.getRow(2).getX(), mat.getRow(2).getY(), mat.getRow(2).getZ() );
  Rodrigues(R_3f, V_3f);
  initial_pose_a_ = V_3f.at<float>(2); // * 180 / 3.1415926;
  std::cout<<"change!"<<initial_pose_x_<<" "<<initial_pose_y_<<" "<<initial_pose_a_<<std::endl;
}
void CautionLayer::MatchSize()
{
  if (!layered_costmap_->IsRolling()) 
  {
    Costmap2D* master = layered_costmap_->GetCostMap();
    ResizeMap(master->GetSizeXCell(), master->GetSizeYCell(), 
              master->GetResolution(), master->GetOriginX(), 
              master->GetOriginY() );
  }
}

void CautionLayer::UpdateCosts(Costmap2D& master_grid, int min_i, 
                               int min_j, int max_i, int max_j)
{
  unsigned int cx, cy, mx, my, ax, ay;
  /*if(master_grid.World2Map(caution_center_x_, caution_center_y_, cx, cy)){
      ROS_INFO("%d, %d", cx, cy);
    master_grid.SetCost(cx, cy, 255);
  }*/
  if (master_grid.World2Map(caution_min_x_, caution_min_y_, mx, my) &&
      master_grid.World2Map(caution_max_x_, caution_max_y_, ax, ay) &&
      master_grid.World2Map(mark_x_, mark_y_, cx, cy))
  {
    //std::cout<<"mxy"<<mx<<" "<<my<<" "<<cx<<" "<<cy<<std::endl;
    cv::Mat R = (cv::Mat_<float>(2,2) << cos(initial_pose_a_), sin(initial_pose_a_),
                                        -sin(initial_pose_a_), cos(initial_pose_a_));
    //ROS_INFO("%f, %f, %f, %f", caution_min_x_, caution_max_x_, caution_min_y_, caution_max_y_);
    //ROS_INFO("ax:%d, ay:%d", ax, ay);
    //ROS_INFO("mx:%d, my:%d", mx, my);
    //for (int i = std::max(min_i, mx); i < std::min(max_i, ax); i++)
    for (int i = mx; i < ax; i++)
    {
      //for (int j = std::max(min_j, my); j < std::min(max_j, ay); j++)
      for (int j = my; j < ay; j++)
      {
        cv::Mat k = (cv::Mat_<float>(2,1) << float(int(i)-int(cx)), float(int(j)-int(cy)));
        //std::cout<<"R"<<R<<" "<<k<<std::endl;
        k = R*k;
        //std::cout<<k<<std::endl;
        int ki, kj;
        ki = int(k.at<float>(0)+cx);
        kj = int(k.at<float>(1)+cy);
        //std::cout<<"center ("<<cx<<", "<<cy<<")"<<std::endl;
        //std::cout<<"point ("<<ki<<", "<<kj<<") ("<<i<<", "<<j<<")"<<std::endl;
        
        if(ki <= master_grid.GetSizeXCell() && ki >= 0 &&
           kj <= master_grid.GetSizeYCell() && kj >= 0)
          master_grid.SetCost(ki, kj, std::max(master_grid.GetCost(ki, kj), caution_cost_value_));
        //master_grid.SetCost(i, j, std::max(master_grid.GetCost(i, j), caution_cost_value_));
      }
    }
  }

  /*
  //mark_x, mark_y_是robot前方1m的障碍物
  //unsigned int mx;
  //unsigned int my;
  if(master_grid.World2Map(mark_x_, mark_y_, mx, my)){
    master_grid.SetCost(mx, my, LETHAL_OBSTACLE);
  }
  */
  
}

void CautionLayer::UpdateBounds(double robot_x, double robot_y,
                               double robot_yaw, double *min_x,
                               double *min_y, double *max_x, double *max_y) 
{
  mark_x_ = cos(initial_pose_a_) * (caution_center_x_ - initial_pose_x_) + 
            sin(initial_pose_a_) * (caution_center_y_ - initial_pose_y_);
  mark_y_ = -sin(initial_pose_a_) * (caution_center_x_ - initial_pose_x_) + 
            cos(initial_pose_a_) * (caution_center_y_ - initial_pose_y_);

  //float new_x1_ = -cos(initial_pose_a_)*caution_width_/2 + sin(initial_pose_a_)*caution_height_/2;
  //float new_x2_ = -cos(initial_pose_a_)*caution_width_/2 - sin(initial_pose_a_)*caution_height_/2;
  //float new_x3_ = cos(initial_pose_a_)*caution_width_/2 - sin(initial_pose_a_)*caution_height_/2;
  //float new_x4_ = cos(initial_pose_a_)*caution_width_/2 + sin(initial_pose_a_)*caution_height_/2;
  caution_min_x_ = mark_x_ - caution_width_/2;
  caution_max_x_ = mark_x_ + caution_width_/2;
  caution_min_y_ = mark_y_ - caution_height_/2;
  caution_max_y_ = mark_y_ + caution_height_/2;

  double wx, wy;
  if(!layered_costmap_->IsRollingWindow()) {
    //return;
  }
  //just make sure the value is normal
  //UseExtraBounds(min_x, min_y, max_x, max_y);

  /*
  //mark_x, mark_y_是robot前方1m的障碍物
  mark_x_ = caution_center_x_;//robot_x + cos(robot_yaw);
  mark_y_ = caution_center_y_;//robot_y + sin(robot_yaw);
  ROS_INFO("ronot obstacle: %f, %f", mark_x_, mark_y_);
 
  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
  */

  if (is_world_)
  {
    *min_x = std::min(caution_min_x_, *min_x);
    *min_y = std::min(caution_min_y_, *min_y);
    *max_x = std::max(*max_x, caution_max_x_);
    *max_y = std::max(*max_y, caution_max_y_);
  }
  else
  {
    double wx, wy;
    Map2World(caution_min_x_, caution_min_y_, wx, wy);
    *min_x = std::min(wx, *min_x);
    *min_y = std::min(wy, *min_y);
    Map2World(caution_max_x_, caution_max_y_, wx, wy);
    *max_x = std::max(*max_x, wx);
    *max_y = std::max(*max_y, wy);
  }

}

void CautionLayer::Activate() 
{
  OnInitialize();
}

void CautionLayer::Deactivate() 
{
//    delete cost_map_;
}

}