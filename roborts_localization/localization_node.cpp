//
// Created by cxn on 19-2-15.
//
#include "localization_node.h"

namespace leonard_localization {

    static Mat3d MsgCovarianceToMat3d(const boost::array<double, 36> &msg_cov) {
        Mat3d pose_cov;
        pose_cov.setZero();

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                pose_cov(i, j) = msg_cov[6 * i + j];
            }
        }
        pose_cov(2, 2) = msg_cov[6 * 5 + 5];
        return pose_cov;
    }

    LocalizationNode::LocalizationNode(std::string name) {
        CHECK(Init()) << "Module " << name << " initialized failed!";
        initialized_ = true;
    }

    bool LocalizationNode::Init() {

        //从参数服务器中将参数提取出来，并存放在odom_frame_id中，若参数服务器中没有相应参数，则加载默认值"odom"
        //参数服务器这一块可以参考博客https://blog.csdn.net/u014695839/article/details/78348600
        nh_.param<std::string>("localization/odom_frame_id", odom_frame_, "odom");
        nh_.param<std::string>("localization/base_frame_id", base_frame_, "base_link");
        nh_.param<std::string>("localization/global_frame_id", global_frame_, "map");
        nh_.param<std::string>("localization/laser_topic_name", laser_topic_, "scan");

        double transform_tolerance;
        nh_.param<double>("localization/transform_tolerance", transform_tolerance, 0.1);
        this->transform_tolerance_ = ros::Duration(transform_tolerance);

        nh_.param<double>("localization/initial_pose_x", init_pose_(0), 1);
        nh_.param<double>("localization/initial_pose_y", init_pose_(1), 1);
        nh_.param<double>("localization/initial_pose_a", init_pose_(2), 0);

        nh_.param<double>("localization/initial_cov_xx", init_cov_(0), 0.1);//用于初始化高斯分布滤波器
        nh_.param<double>("localization/initial_cov_yy", init_cov_(1), 0.1);
        nh_.param<double>("localization/initial_cov_aa", init_cov_(2), 0.1);

        nh_.param<bool>("localization/publish_visualize", publish_visualize_, true);

        tf_broadcaster_ptr_ = std::make_unique<tf::TransformBroadcaster>();
        tf_listener_ptr_ = std::make_unique<tf::TransformListener>();

        // Use message filter for time synchronizer (laser scan topic and tf between odom and base frame)
        laser_scan_sub_ =
                std::make_shared<message_filters::Subscriber<sensor_msgs::LaserScan >>(nh_, laser_topic_, 100);
        // 收到laser_topic信息会缓存，直到tf信息也收到，进入回调处理
        laser_scan_filter_ = std::make_unique<tf::MessageFilter<sensor_msgs::LaserScan >>(*laser_scan_sub_,
                                                                                          *tf_listener_ptr_,
                                                                                          odom_frame_,
                                                                                          100);
        laser_scan_filter_->registerCallback(boost::bind(&LocalizationNode::LaserScanCallback, this, _1));

        //接收到init_pose_的topic
        initial_pose_sub_ = nh_.subscribe("initialpose", 2, &LocalizationNode::InitialPoseCallback, this);

        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("amcl_pose", 2, true);
        distance_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("distance_map", 1, true);
        particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);

        amcl_ptr_ = std::make_unique<Amcl>();
        amcl_ptr_->GetParamFromRos(&nh_);
        amcl_ptr_->Init(init_pose_, init_cov_);

        map_init_ = GetStaticMap();//map_server服务
        laser_init_ = GetLaserPose();

        force_update_mode_srv_ = nh_.advertiseService("set_force_update_mode",
                                                      &LocalizationNode::SetResamplelModeService, this);


        // Use message filter for time synchronizer (laser scan topic and tf between odom and base frame)
//        tag_detections_sub_ptr_ = std::make_shared<message_filters::Subscriber<apriltags2_ros::AprilTagDetectionArray>>(
//                nh_, "tag_detections", 1);
//        // 收到laser_topic信息会缓存，直到tf信息也收到，进入回调处理
//        tag_detections_filter_ = std::make_unique<tf::MessageFilter<apriltags2_ros::AprilTagDetectionArray >>(
//                *tag_detections_sub_,
//                *tf_listener_ptr_,
//                odom_frame_,
//                1);
//        tag_detections_filter_->registerCallback(boost::bind(&LocalizationNode::TagCallback, this, _1));


        return map_init_ && laser_init_;
    }

    bool LocalizationNode::SetResamplelModeService(roborts_msgs::ForceUpdateAmcl::Request &req,
                                                   roborts_msgs::ForceUpdateAmcl::Response &res) {
        amcl_ptr_->SetForceUpdateMode(req.mode);
        res.received = true;
        return true;
    }

    void LocalizationNode::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg_ptr) {
        last_laser_msg_timestamp_ = laser_scan_msg_ptr->header.stamp;

        Vec3d pose_in_odom;//base_link(0,0)在odom中坐标
        if (!GetPoseFromTf(odom_frame_, base_frame_, last_laser_msg_timestamp_, pose_in_odom)) {
            LOG_ERROR << "Couldn't determine robot's pose";
            return;
        }

        double angle_min = 0, angle_increment = 0;
        sensor_msgs::LaserScan laser_scan_msg = *laser_scan_msg_ptr;
        // 计算激光扫描初始角与扫描角间隔在Base_link中的角度值
        TransformLaserscanToBaseFrame(angle_min, angle_increment, laser_scan_msg);

        amcl_ptr_->Update(pose_in_odom,
                          laser_scan_msg,
                          angle_min,
                          angle_increment,
                          particlecloud_msg_,
                          hyp_pose_);

        // Tf树
        LOG_ERROR_IF(!PublishTf()) << "Publish Tf Error!";
        // 话题topic
        if (publish_visualize_) {
            PublishVisualize();
        }

//        amcl_ptr_->NoUpdate(pose_in_odom,
//                          laser_scan_msg,
//                          angle_min,
//                          angle_increment,
//                          particlecloud_msg_,
//                          hyp_pose_);
//        if (particlecloud_pub_.getNumSubscribers() > 0) {
//            particlecloud_msg_.header.stamp = ros::Time::now();
//            particlecloud_msg_.header.frame_id = global_frame_;
//            particlecloud_pub_.publish(particlecloud_msg_);
//        }
    }

    bool LocalizationNode::PublishTf() {
        ros::Time transform_expiration = (last_laser_msg_timestamp_ + transform_tolerance_);
        if (amcl_ptr_->CheckTfUpdate()) {
            // Subtracting base to odom from map to base and send map to odom instead
            tf::Stamped<tf::Pose> odom_to_map;
            try {
                tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyp_pose_.pose_mean[2]),
                                     tf::Vector3(hyp_pose_.pose_mean[0],
                                                 hyp_pose_.pose_mean[1],
                                                 0.0));

                tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(),
                                                     last_laser_msg_timestamp_,
                                                     base_frame_);

                this->tf_listener_ptr_->transformPose(odom_frame_,
                                                      tmp_tf_stamped,
                                                      odom_to_map);

            } catch (tf::TransformException &e) {
                LOG_ERROR << "Failed to subtract base to odom transform" << e.what();
                return false;
            }

            latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                       tf::Point(odom_to_map.getOrigin()));
            latest_tf_valid_ = true;

            // 2.18 以下分析不一定对
            // 为什么不直接发送base_link到map？
            // 其实就是tf树的组织问题，已有base到odom的tf树，若发送base到map的tf树，那么odom到map就不在同一棵树上了，它两个的关系就容易带来错误，
            // 而若发送odom到base，map到base，是不是就可以了呢？有空试一下

            // 发送base到map报的错
            /* 报错如下，好像是costmap_interface.cpp里的GetRobotPose方法里抛出的错误
              [ERROR] [1550157766.089043893, 7.600000000]: Extrapolation Error looking up robot pose:
              Lookup would require extrapolation into the future.
              Requested time 7.650000000 but the latest data is at time 7.600000000,
              when looking up transform from frame [base_link] to frame [odom]
              [ERROR] [1550157766.283086281, 7.800000000]: Extrapolation Error looking up robot pose:
              Lookup would require extrapolation into the future.
              Requested time 7.850000000 but the latest data is at time 7.800000000,
              when looking up transform from frame [base_link] to frame [odom]
            */

            tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                                transform_expiration,
                                                global_frame_,
                                                odom_frame_);

            this->tf_broadcaster_ptr_->sendTransform(tmp_tf_stamped);

            return true;
        } else if (latest_tf_valid_) {
            // Nothing changed, so we'll just republish the last transform
            tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                                transform_expiration,
                                                global_frame_,
                                                odom_frame_);
            this->tf_broadcaster_ptr_->sendTransform(tmp_tf_stamped);
            return true;
        } else {
            return false;
        }
    }

    void LocalizationNode::PublishVisualize() {
        if (pose_pub_.getNumSubscribers() > 0) {
            pose_msg_.header.stamp = ros::Time::now();
            pose_msg_.header.frame_id = global_frame_;
            pose_msg_.pose.position.x = hyp_pose_.pose_mean[0];
            pose_msg_.pose.position.y = hyp_pose_.pose_mean[1];
            pose_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(hyp_pose_.pose_mean[2]);
            pose_pub_.publish(pose_msg_);
        }

        if (particlecloud_pub_.getNumSubscribers() > 0) {
            particlecloud_msg_.header.stamp = ros::Time::now();
            particlecloud_msg_.header.frame_id = global_frame_;
            particlecloud_pub_.publish(particlecloud_msg_);
        }

        if (!publish_first_distance_map_) {
            distance_map_pub_.publish(amcl_ptr_->GetDistanceMapMsg());
            publish_first_distance_map_ = true;
        }
    }


    bool LocalizationNode::GetStaticMap() {
        static_map_srv_ = nh_.serviceClient<nav_msgs::GetMap>("static_map");
        ros::service::waitForService("static_map", -1);
        nav_msgs::GetMap::Request req;
        nav_msgs::GetMap::Response res;
        if (static_map_srv_.call(req, res)) {
            LOG_INFO << "Received Static Map";
            amcl_ptr_->HandleMapMessage(res.map, init_pose_, init_cov_);
            return true;
        } else {
            LOG_ERROR << "Get static map failed";
            return false;
        }
    }

    bool LocalizationNode::GetLaserPose() {
        auto laser_scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(laser_topic_);

        Vec3d laser_pose;
        GetPoseFromTf(base_frame_, laser_scan_msg->header.frame_id, ros::Time(), laser_pose);
        laser_pose[2] = 0; // No need for rotation, or will be error
        LOG_INFO << "Received laser's pose wrt robot: " <<
                 laser_pose[0] << ", " <<
                 laser_pose[1] << ", " <<
                 laser_pose[2];

        amcl_ptr_->SetLaserSensorPose(laser_pose);
        return true;
    }

    bool LocalizationNode::GetPoseFromTf(const std::string &target_frame,
                                         const std::string &source_frame,
                                         const ros::Time &timestamp,
                                         Vec3d &pose) {
        tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),
                                                  tf::Vector3(0, 0, 0)),
                                    timestamp,
                                    source_frame);
        //求source_frame中的原点位姿在target_frame中的位姿
        tf::Stamped<tf::Pose> pose_stamp;
        try {
            this->tf_listener_ptr_->transformPose(target_frame,
                                                  ident,
                                                  pose_stamp);
        } catch (tf::TransformException &e) {
            LOG_ERROR << "Couldn't transform from "
                      << source_frame
                      << "to "
                      << target_frame;
            return false;
        }

        pose[0] = pose_stamp.getOrigin().x();
        pose[1] = pose_stamp.getOrigin().y();
        pose[2] = 0;
        double yaw, pitch, roll;
        pose_stamp.getBasis().getEulerYPR(yaw, pitch, roll);
        pose[2] = yaw;
        return true;
    }

    void LocalizationNode::TransformLaserscanToBaseFrame(double &angle_min,
                                                         double &angle_increment,
                                                         const sensor_msgs::LaserScan &laser_scan_msg) {

        // To account for lasers that are mounted upside-down, we determine the
        // min, max, and increment angles of the laser in the base frame.
        // Construct min and max angles of laser, in the base_link frame.
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, laser_scan_msg.angle_min);
        tf::Stamped<tf::Quaternion> min_q(q, laser_scan_msg.header.stamp,
                                          laser_scan_msg.header.frame_id);
        q.setRPY(0.0, 0.0, laser_scan_msg.angle_min
                           + laser_scan_msg.angle_increment);
        tf::Stamped<tf::Quaternion> inc_q(q, laser_scan_msg.header.stamp,
                                          laser_scan_msg.header.frame_id);

        try {
            tf_listener_ptr_->transformQuaternion(base_frame_,
                                                  min_q,
                                                  min_q);
            tf_listener_ptr_->transformQuaternion(base_frame_,
                                                  inc_q,
                                                  inc_q);
        }
        catch (tf::TransformException &e) {
            LOG_WARNING << "Unable to transform min/max laser angles into base frame: " << e.what();
            return;
        }

        angle_min = tf::getYaw(min_q);
        angle_increment = (tf::getYaw(inc_q) - angle_min);

        // Wrapping angle to [-pi .. pi]
        // std::fmod 返回浮点余数 std::fmod(18.5,4.2)=1.7
        angle_increment = (std::fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI);
        // 若雷达x轴与车正相反，则两个数字为：0.0158609,0.0174532
    }

    void
    LocalizationNode::InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &init_pose_msg) {

        if (init_pose_msg->header.frame_id == "") {
            LOG_WARNING << "Received initial pose with empty frame_id.";
        } // Only accept initial pose estimates in the global frame
        else if (tf_listener_ptr_->resolve(init_pose_msg->header.frame_id) !=
                 tf_listener_ptr_->resolve(global_frame_)) {
            LOG_ERROR << "Ignoring initial pose in frame \" "
                      << init_pose_msg->header.frame_id
                      << "\"; initial poses must be in the global frame, \""
                      << global_frame_;
            return;
        }

        // In case the client sent a pose estimate in the past, integrate the
        // intervening odometric change.
        tf::StampedTransform tx_odom;
        try {
            //获取now在时间戳坐标系中的变换tx_odom
            ros::Time now = ros::Time::now();
            tf_listener_ptr_->waitForTransform(base_frame_,
                                               init_pose_msg->header.stamp,
                                               base_frame_,
                                               now,
                                               odom_frame_,
                                               ros::Duration(0.5));
            tf_listener_ptr_->lookupTransform(base_frame_,
                                              init_pose_msg->header.stamp,
                                              base_frame_,
                                              now,
                                              odom_frame_, tx_odom);
        }
        catch (tf::TransformException &e) {
            tx_odom.setIdentity();
        }
        tf::Pose pose_new;
        tf::Pose pose_old;
        tf::poseMsgToTF(init_pose_msg->pose.pose, pose_old);

        //时间戳下base到map的变换*now下base在时间戳下base中的变换 = now下base到map的变换
        pose_new = pose_old * tx_odom;

        // Transform into the global frame
        DLOG_INFO << "Setting pose " << ros::Time::now().toSec() << ", "
                  << pose_new.getOrigin().x() << ", " << pose_new.getOrigin().y();

        Vec3d init_pose_mean;
        Mat3d init_pose_cov;
        init_pose_mean.setZero();
        init_pose_cov.setZero();
        double yaw, pitch, roll;
        init_pose_mean(0) = pose_new.getOrigin().x();
        init_pose_mean(1) = pose_new.getOrigin().y();
        pose_new.getBasis().getEulerYPR(yaw, pitch, roll);
        init_pose_mean(2) = yaw;
        init_pose_cov = MsgCovarianceToMat3d(init_pose_msg->pose.covariance);

        amcl_ptr_->HandleInitialPoseMessage(init_pose_mean, init_pose_cov);
    }
}

int main(int argc, char **argv) {
    leonard_localization::GLogWrapper gLogWrapper(argv[0]);
    ros::init(argc, argv, "localization_node");
    leonard_localization::LocalizationNode localization_node("localization_node");
    ros::AsyncSpinner async_spinner(THREAD_NUM);
    async_spinner.start();
//    ros::spin();
    ros::waitForShutdown();
//    std::vector <std::string> cc = {"age", "home", "cxn", "zlx"};
//    leonard_filewrite::CsvWriter a("/home/cxn/data.csv", cc);
//    a.write(1.34555);
//    a.write(34341);
//    a.write(1.676);
//    a.write(61);
//    a.write('a');
//    a.write("dfaslfs");
//    a.write("dsad");
    return 0;
}
