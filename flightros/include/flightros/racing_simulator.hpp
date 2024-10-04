#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>

#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

class RacingSimulator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RacingSimulator();
    ~RacingSimulator();

private:
    void stateEstCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void imagePubCallback(const ros::TimerEvent &event);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;

    ros::Subscriber state_est_sub_;
    ros::Timer image_pub_timer_;
    image_transport::Publisher rgb_pub_;

    std::shared_ptr<flightlib::Quadrotor> quad_ptr_;
    std::shared_ptr<flightlib::RGBCamera> rgb_camera_;
    flightlib::QuadState quad_state_;

    std::shared_ptr<flightlib::UnityBridge> unity_bridge_ptr_;
    flightlib::SceneID scene_id_{flightlib::UnityScene::WAREHOUSE};

    bool is_unity_ready_{false};
};