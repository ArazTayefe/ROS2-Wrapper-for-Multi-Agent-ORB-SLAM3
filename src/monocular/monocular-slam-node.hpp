#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include <cv_bridge/cv_bridge.h>
#include "System.h"

#include <fstream>
#include <unordered_map>

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);
    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using RigidBodiesMsg = mocap4r2_msgs::msg::RigidBodies;

    void GrabImage(const ImageMsg::SharedPtr msg);
    void MocapCallback(const RigidBodiesMsg::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;
    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<ImageMsg>::SharedPtr m_image_subscriber;
    rclcpp::Subscription<RigidBodiesMsg>::SharedPtr m_mocap_subscriber;

    std::unordered_map<std::string, std::ofstream> mocap_logs_;
    std::unordered_map<std::string, std::string> id_to_name_;

    double last_frame_time_;
};

#endif
