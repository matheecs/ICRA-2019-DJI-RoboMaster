#include "ros/ros.h"
#include "std_msgs/Bool.h"
//function
#include "sub_cam_node.hpp"
#include "roborts_msgs/RobotStatus.h"
#ifdef SERVICE_I
#include <cv_bridge/cv_bridge.h>
#include <sub_cam/StartVision.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
namespace VISION_MODE
{
bool vision_mode = 0;
bool vision_mode_back(sub_cam::StartVision::Request &req, sub_cam::StartVision::Response &res)
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
    ros::init(argc, argv, "sub_camera");
    ros::NodeHandle sub_cam;
#ifdef SERVICE_I
    image_transport::ImageTransport sub_cam_image(sub_cam);
#endif
    ros::Publisher back_pub = sub_cam.advertise<std_msgs::Bool>("enemy_back", 1);
#ifdef SERVICE_I
    image_transport::CameraPublisher image_pub = sub_cam_image.advertiseCamera("image_raw", 1);
    //service
    ros::ServiceServer vison_mode_ser = sub_cam.advertiseService("start_vision", VISION_MODE::vision_mode_back);
#endif
    ros::Subscriber robot_status_sub_ = sub_cam.subscribe("robot_status", 2, &ROBOT_STATUS::RobotStatusCallback);

    //param load
    sub_cam.param("sub_cam/enemy_color_", Param.enemy_color, 0);
    sub_cam.param("sub_cam/video_", Param.video, 0);
    sub_cam.param("sub_cam/area_max_", Param.area_max, (float)1000);
    sub_cam.param("sub_cam/area_min_", Param.area_min, (float)0);
    sub_cam.param("sub_cam/angle_max_", Param.angle_max, (float)50);
    sub_cam.param("sub_cam/angle_min_", Param.angle_min, (float)40);
    sub_cam.param("sub_cam/gray_min_", Param.gray_min, 200);
    sub_cam.param("sub_cam/color_min_", Param.color_min, 100);
    sub_cam.param("sub_cam/red_ap_min_", Param.red_ap_min, 40);
    sub_cam.param("sub_cam/blue_ap_min_", Param.blue_ap_min, 30);
    cv::VideoCapture sub_cap(700); //DSHOW
    sub_cap.open(Param.video);
    if (!sub_cap.isOpened())
    {
        std::cout << "ERROR IN OPEN MAIN CAMERA!" << std::endl;
        return -1;
    }
    sub_cap.set(6, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); //CV_CAP_PROP_FOURCC
    sub_cap.set(3, 480);                                         //CV_CAP_PROP_FRAME_WIDTH
    sub_cap.set(4, 640);
    //variable declaration
#ifdef RECORD
    std::ifstream numi("num.txt");
    int num;
    numi >> num;
    numi.close();
    num += 1;
    std::ofstream numo("num.txt");
    numo << num;
    numo.close();
    cv::VideoWriter frame_writer("/home/teliute/sub_cam_" + std::to_string(num) + ".mkv", CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(640, 480));
#endif
#ifdef SERVICE_I
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info;
    sensor_msgs::Image image_msg;
    cam_info.reset(new camera_info_manager::CameraInfoManager(sub_cam, "main_camera", ""));
    if (!cam_info->isCalibrated())
    {
        cam_info->setCameraName("/dev/video0");
        sensor_msgs::CameraInfo cam_info_;
        cam_info_.header.frame_id = image_msg.header.frame_id;
        cam_info_.width = 480;
        cam_info_.height = 640;
        cam_info->setCameraInfo(cam_info_);
    }
    sensor_msgs::CameraInfoPtr camera_info;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
#endif
    cv::Mat image;
    ros::spinOnce();
    while (ros::ok())
    {
#ifdef SERVICE_I
        if (VISION_MODE::vision_mode)
        {
            sub_cap >> image;
#ifdef RECORD
            frame_writer << image;
#endif
#ifdef TEST
            cv::imshow("XX", image);
#endif
            cv::GaussianBlur(image, image, cv::Size(3, 3), 1);
            std::vector<cv::Mat> channels(3);
            cv::split(image, channels);
            cv::Mat gray;
            if (Param.enemy_color)
            {
                gray = 0.3 * channels[1] + 0.7 * channels[0];
                gray = gray > Param.red_ap_min;
            }
            else
            {
                gray = 0.3 * channels[1] + 0.7 * channels[2];
                cv::equalizeHist(gray, gray);
                cv::threshold(gray, gray, Param.blue_ap_min, 255, cv::THRESH_TRUNC);
                //gray = gray > Param.blue_ap_min;
            }
            cv::cvtColor(gray, image, cv::COLOR_GRAY2BGR);
#ifdef TEST
            cv::imshow("YY", image);
#endif
            cv_ptr->image = image;
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
            sub_cap >> image;
            if (!image.empty())
            {
                std_msgs::Bool msg_back;
                msg_back.data = main_f(image);
                back_pub.publish(msg_back);
            }
#ifdef SERVICE_I
        }
#endif
#if defined(DEBUG) || defined(TEST)
        cv::waitKey(20);
#endif
        ros::spinOnce();
    }

    return 0;
}