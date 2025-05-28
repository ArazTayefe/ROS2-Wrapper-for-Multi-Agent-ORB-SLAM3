#include "monocular-slam-node.hpp"
#include <opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
    : Node("ORB_SLAM3_ROS2"), m_SLAM(pSLAM), last_frame_time_(0.0)
{
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera", 10, std::bind(&MonocularSlamNode::GrabImage, this, _1));

    m_mocap_subscriber = this->create_subscription<RigidBodiesMsg>(
        "/rigid_bodies", 10, std::bind(&MonocularSlamNode::MocapCallback, this, _1));

    id_to_name_ = {
        {"1", "Alpha"},
        {"2", "Bravo"},
        {"3", "Charlie"}
    };

    for (const auto& [id, name] : id_to_name_) {
        mocap_logs_[name].open("MocapTrajectory_" + name + ".txt");
    }

    RCLCPP_INFO(this->get_logger(), "Monocular SLAM node initialized");
}

MonocularSlamNode::~MonocularSlamNode()
{
    m_SLAM->Shutdown();

    std::string node_name = this->get_name();  // e.g., "alpha"
    std::string output_file = "KeyFrameTrajectory_" + node_name + ".txt";
    m_SLAM->SaveKeyFrameTrajectoryTUM(output_file);

    for (auto& [name, file] : mocap_logs_) {
        file.close();
    }
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    try {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    double timestamp = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
    last_frame_time_ = timestamp;

    m_SLAM->TrackMonocular(m_cvImPtr->image, timestamp);
}

void MonocularSlamNode::MocapCallback(const RigidBodiesMsg::SharedPtr msg)
{
    for (const auto& rb : msg->rigidbodies) {
        auto it = id_to_name_.find(rb.rigid_body_name);
        if (it != id_to_name_.end()) {
            const std::string& name = it->second;
            std::ofstream& out = mocap_logs_[name];

            double t = last_frame_time_;  // Use synchronized timestamp from last image frame
            const auto& pos = rb.pose.position;
            const auto& q = rb.pose.orientation;

            out << std::fixed << t << " "
                << pos.x << " " << pos.y << " " << pos.z << " "
                << q.x << " " << q.y << " " << q.z << " " << q.w << "\n";
        }
    }
}
