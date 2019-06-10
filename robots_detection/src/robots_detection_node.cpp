#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
//function
#include "robots_detection_node.hpp"
//message
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/EnemyDetect.h"
#include "roborts_msgs/RobotStatus.h"
//service_image
#ifdef SERVICE_G
#include "roborts_msgs/GimbalMode.h"
#endif
#ifdef SERVICE_I
#include <cv_bridge/cv_bridge.h>
#include <robots_detection/StartVision.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
namespace VISION_MODE
{
bool vision_mode = 0;
bool vision_mode_back(robots_detection::StartVision::Request &req, robots_detection::StartVision::Response &res)
{
    vision_mode = req.start_vision;
    res.received = 1;
    return true;
}
} // namespace VISION_MODE
#endif
namespace ROBOT_STATUS
{
void RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &msg)
{
    if ((msg->id == 3 || msg->id == 4) && Param.enemy_color != 0)
    {
        Param.enemy_color = 0;
    }
    else if ((msg->id == 13 || msg->id == 14) && Param.enemy_color != 1)
    {
        Param.enemy_color = 1;
    }
}
} // namespace ROBOT_STATUS

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robots_detection");
    ros::NodeHandle main_cam;
#ifdef SERVICE_I
    image_transport::ImageTransport main_cam_image(main_cam);
#endif
    //topic
    ros::Publisher ser_pub = main_cam.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1);
    ros::Publisher enemy_pub = main_cam.advertise<roborts_msgs::EnemyDetect>("test_enemy_pose", 1);
    ros::Subscriber robot_status_sub_ = main_cam.subscribe("robot_status", 2, &ROBOT_STATUS::RobotStatusCallback);
#ifdef SERVICE_I
    image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("image_raw", 1);
    //service
    ros::ServiceServer vison_mode_ser = main_cam.advertiseService("start_vision", VISION_MODE::vision_mode_back);
#endif
#ifdef SERVICE_G
    ros::ServiceClient gimbal_mode_client = main_cam.serviceClient<roborts_msgs::GimbalMode>("set_gimbal_mode");
    bool gimbal_mode_h = 0;
    roborts_msgs::GimbalMode gimbal_mode_msg;
    gimbal_mode_msg.request.gimbal_mode = static_cast<uint8_t>(1);
    gimbal_mode_h = 1;
    gimbal_mode_client.call(gimbal_mode_msg);
#endif
    //param load
    main_cam.param("robots_detection/enemy_color_", Param.enemy_color, 0);
    main_cam.param("robots_detection/video_", Param.video, 0);
    main_cam.param("robots_detection/area_max_", Param.area_max, (float)1000);
    main_cam.param("robots_detection/area_min_", Param.area_min, (float)0);
    main_cam.param("robots_detection/angle_max_", Param.angle_max, (float)50);
    main_cam.param("robots_detection/angle_min_", Param.angle_min, (float)40);
    main_cam.param("robots_detection/ratio_max_", Param.ratio_max, (float)5);
    main_cam.param("robots_detection/ratio_min_", Param.ratio_min, (float)1);
    main_cam.param("robots_detection/ratio_g_", Param.ratio_g, (float)20);
    main_cam.param("robots_detection/ratio_l_", Param.ratio_l, (float)1.5);
    main_cam.param("robots_detection/armor_plat_width_", Param.armor_plat_width, (float)0.130);
    main_cam.param("robots_detection/armor_plat_height_", Param.armor_plat_height, (float)0.55);
    main_cam.param("robots_detection/weight_max_", Param.weight_max, (float)0.6);
    main_cam.param("robots_detection/blue_gray_min_", Param.blue_gray_min, 200);
    main_cam.param("robots_detection/red_gray_min_", Param.red_gray_min, 200);
    main_cam.param("robots_detection/blue_color_min_", Param.blue_color_min, 100);
    main_cam.param("robots_detection/red_color_min_", Param.red_color_min, 100);
    main_cam.param("robots_detection/x_os_", Param.ptz.x_os, (float)0);
    main_cam.param("robots_detection/y_os_", Param.ptz.y_os, (float)0.09);
    main_cam.param("robots_detection/z_os_", Param.ptz.z_os, (float)0.09);
    main_cam.param("robots_detection/pitch_os_", Param.ptz.pitch_os, (float)0);
    main_cam.param("robots_detection/yaw_os_", Param.ptz.yaw_os, (float)0);
    //param set
    Param.armor_pos_height = Param.armor_plat_height / 10;
    Param.armor_pos_width = Param.armor_plat_width;
    Param.camera.points_pos_max = {cv::Point3f(Param.armor_pos_width / 2, -Param.armor_pos_height / 2, 0), cv::Point3f(-Param.armor_pos_width / 2, -Param.armor_pos_height / 2, 0), cv::Point3f(-Param.armor_pos_width / 2, Param.armor_pos_height / 2, 0), cv::Point3f(Param.armor_pos_width / 2, Param.armor_pos_height / 2, 0)};
    Param.camera.points_pos_min = {cv::Point3f(-Param.armor_pos_width / 2, -Param.armor_pos_height / 2, 0), cv::Point3f(-Param.armor_pos_width / 2, Param.armor_pos_height / 2, 0), cv::Point3f(Param.armor_pos_width / 2, Param.armor_pos_height / 2, 0), cv::Point3f(Param.armor_pos_width / 2, -Param.armor_pos_height / 2, 0)};
    Param.camera.points_object_max = {cv::Point3f(Param.armor_plat_width / 2, -Param.armor_plat_height / 2, 0), cv::Point3f(-Param.armor_plat_width / 2, -Param.armor_plat_height / 2, 0), cv::Point3f(-Param.armor_plat_width / 2, Param.armor_plat_height / 2, 0), cv::Point3f(Param.armor_plat_width / 2, Param.armor_plat_height / 2, 0)};
    Param.camera.points_object_min = {cv::Point3f(-Param.armor_plat_width / 2, -Param.armor_plat_height / 2, 0), cv::Point3f(-Param.armor_plat_width / 2, Param.armor_plat_height / 2, 0), cv::Point3f(Param.armor_plat_width / 2, Param.armor_plat_height / 2, 0), cv::Point3f(Param.armor_plat_width / 2, -Param.armor_plat_height / 2, 0)};
    Param.camera.IntrinsicMatrix = (cv::Mat_<float>(3, 3) << 753.269757, 0.000000, 368.357886, 0.000000, 752.711266, 232.078338, 0.000000, 0.000000, 1.000000);
    Param.camera.Distortion = (cv::Mat_<float>(4, 1) << -0.003080, 0.015896, 0, 0);
    //camera set
    cv::VideoCapture main_cap(cv::CAP_DSHOW); //(cv::CAP_V4L2); //DSHOW
    main_cap.open(Param.video);
    if (!main_cap.isOpened())
    {
        std::cout << "ERROR IN OPEN MAIN CAMERA!" << std::endl;
        return -1;
    }
    main_cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); //CV_CAP_PROP_FOURCC
    main_cap.set(cv::CAP_PROP_FRAME_WIDTH, 480);                                    //CV_CAP_PROP_FRAME_WIDTH
    main_cap.set(cv::CAP_PROP_FRAME_HEIGHT, 640);                                   //CV_CAP_PROP_FRAME_HEIGHT
    main_cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);                                 //(cv::CAP_PROP_AUTO_EXPOSURE
    main_cap.set(cv::CAP_PROP_EXPOSURE, -12);                                       //CV_CAP_PROP_EXPOSURE
    //===================
#ifdef RECORD
    std::ifstream numi("num.txt");
    int num;
    numi >> num;
    numi.close();
    num += 1;
    std::ofstream numo("num.txt");
    numo << num;
    numo.close();
    cv::VideoWriter frame_writer("/home/teliute/detection0520" + std::to_string(num) + ".mkv", CV_FOURCC('M', 'J', 'P', 'G'), 60, cv::Size(640, 480));
#endif
    //===================
    //variable declaration
    roborts_msgs::GimbalAngle gimbal_angle;
    tf::TransformListener tf_listener_;
    tf::Stamped<tf::Pose> enemy_to_map;
    roborts_msgs::EnemyDetect enemy_to_map_msg;
    Pos pos_ct;
#ifdef SERVICE_I
    sensor_msgs::Image image_msg;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info;
    cam_info.reset(new camera_info_manager::CameraInfoManager(main_cam, "main_camera", ""));
    if (!cam_info->isCalibrated())
    {
        cam_info->setCameraName("/dev/video0");
        sensor_msgs::CameraInfo cam_info_;
        cam_info_.header.frame_id = "main_camera";
        cam_info_.width = 480;
        cam_info_.height = 640;
        cam_info->setCameraInfo(cam_info_);
    }
    sensor_msgs::CameraInfoPtr camera_info;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
#endif
    cv::Mat image;
    float yaw_speed = 0, pitch_speed = 0;
    double time0 = static_cast<double>(cv::getTickCount());
    float pitch_angle_last = 0;
    ros::spinOnce();
    while (ros::ok())
    {
#ifdef SERVICE_I
        if (VISION_MODE::vision_mode)
        {
            main_cap >> cv_ptr->image;
            image_msg = *(cv_ptr->toImageMsg());
            image_msg.header.stamp = ros::Time::now();
            image_msg.header.frame_id = "main_camera";
            camera_info = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(cam_info->getCameraInfo()));
            camera_info->header.frame_id = image_msg.header.frame_id;
            camera_info->header.stamp = image_msg.header.stamp;
            image_pub.publish(image_msg, *camera_info);
        }
        else
        {
#endif
#ifdef RECORD
            frame_writer << image;
#endif
            main_cap >> image;
            //std::cout << "exposure:" << main_cap.get(cv::CAP_PROP_EXPOSURE) << std::endl;
            armor.time_d = (double)(cv::getTickCount() - time0) / cv::getTickFrequency();
            //std::cout << "frames differ: " << armor.time_d << std::endl;
            time0 = static_cast<double>(cv::getTickCount());
            gimbal_angle.time = time0;
            main_f(image, gimbal_angle, pos_ct);
            gimbal_angle.yaw_spd = gimbal_angle.yaw_angle * 180 / (CV_PI * armor.time_d);
            gimbal_angle.pitch_spd = (gimbal_angle.pitch_angle - pitch_angle_last) * 180 / (CV_PI * armor.time_d);
            ser_pub.publish(gimbal_angle);
            pitch_angle_last = gimbal_angle.pitch_angle;
            try
            {
                tf::Transform tmp_tf(tf::createQuaternionFromYaw(0), tf::Vector3(pos_ct.pos_x, pos_ct.pos_y, 0.0));
                tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf, ros::Time(0), "gimbal");
                tf_listener_.transformPose("map", tmp_tf_stamped, enemy_to_map);
                tf::poseStampedTFToMsg(enemy_to_map, enemy_to_map_msg.enemy_pos);
                enemy_to_map_msg.detected = pos_ct.pos_mode;
            }
            catch (tf::TransformException &e)
            {
                // std::cout << "Failed to subtract enemy to map transform" << e.what() << std::endl;
            }
            //topic publisher
            enemy_pub.publish(enemy_to_map_msg);
#ifdef SERVICE_G
            if (gimbal_mode_h != pos_ct.pos_mode)
            {
                if (pos_ct.pos_mode)
                {
                    gimbal_mode_msg.request.gimbal_mode = static_cast<uint8_t>(2);
                    gimbal_mode_h = pos_ct.pos_mode;
                }
                else
                {
                    gimbal_mode_msg.request.gimbal_mode = static_cast<uint8_t>(1);
                    gimbal_mode_h = pos_ct.pos_mode;
                }
                gimbal_mode_client.call(gimbal_mode_msg);
            }
#endif
#ifdef SERVICE_I
        }
#endif
        ros::spinOnce();
    }
    return 0;
}
