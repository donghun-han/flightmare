#include "flightros/racing_simulator.hpp"

RacingSimulator::RacingSimulator() : nh_(""), pnh_("~"), it_(nh_) {
    quad_ptr_ = std::make_shared<flightlib::Quadrotor>();
    rgb_camera_ = std::make_shared<flightlib::RGBCamera>();

    flightlib::Vector<3> B_r_BC(0.0, 0.0, 0.3);
    flightlib::Matrix<3, 3> R_BC = flightlib::Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
    rgb_camera_->setFOV(90);
    rgb_camera_->setWidth(640);
    rgb_camera_->setHeight(480);
    rgb_camera_->setRelPose(B_r_BC, R_BC);
    rgb_camera_->setPostProcessing(std::vector<bool>{false, false, false});  // depth, segmentation, optical flow
    quad_ptr_->addRGBCamera(rgb_camera_);

    quad_state_.setZero();
    quad_ptr_->reset(quad_state_);


    state_est_sub_ = nh_.subscribe("flightmare/uav_state", 1, &RacingSimulator::stateEstCallback, this);
    image_pub_timer_ = nh_.createTimer(ros::Duration(1 / 30), &RacingSimulator::imagePubCallback, this);
    rgb_pub_ = it_.advertise("flightmare/rgb", 1);
    
    ros::Duration(5.0).sleep();

    if (unity_bridge_ptr_ == nullptr) {
        unity_bridge_ptr_ = flightlib::UnityBridge::getInstance();
        unity_bridge_ptr_->addQuadrotor(quad_ptr_);
        ROS_INFO("Unity bridge created");
    }

    if (unity_bridge_ptr_ != nullptr) {
        is_unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
        ROS_INFO("Unity connected");
    }
}

RacingSimulator::~RacingSimulator() {}

void RacingSimulator::stateEstCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    quad_state_.x[flightlib::QS::POSX] = (flightlib::Scalar)msg->pose.pose.position.x;
    quad_state_.x[flightlib::QS::POSY] = (flightlib::Scalar)msg->pose.pose.position.y;
    quad_state_.x[flightlib::QS::POSZ] = (flightlib::Scalar)msg->pose.pose.position.z;
    quad_state_.x[flightlib::QS::ATTW] = (flightlib::Scalar)msg->pose.pose.orientation.w;
    quad_state_.x[flightlib::QS::ATTX] = (flightlib::Scalar)msg->pose.pose.orientation.x;
    quad_state_.x[flightlib::QS::ATTY] = (flightlib::Scalar)msg->pose.pose.orientation.y;
    quad_state_.x[flightlib::QS::ATTZ] = (flightlib::Scalar)msg->pose.pose.orientation.z;

    quad_ptr_->setState(quad_state_);

    if (is_unity_ready_) {
        unity_bridge_ptr_->getRender(0);
        unity_bridge_ptr_->handleOutput();
    }
}

void RacingSimulator::imagePubCallback(const ros::TimerEvent &event) {
    if (is_unity_ready_) {
        cv::Mat rgb_image;
        ros::Time timestamp = ros::Time::now();

        rgb_camera_->getRGBImage(rgb_image);
        sensor_msgs::ImagePtr rgb_msg = 
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_image).toImageMsg();
        rgb_msg->header.stamp = timestamp;
        rgb_pub_.publish(rgb_msg);
    }
}