#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"
#include "System.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string vocab_path;
    std::string config_path;

    // CLI usage: ros2 run orbslam3 mono <vocab> <yaml>
    if (argc == 3) {
        vocab_path = argv[1];
        config_path = argv[2];
    } else {
        // ROS2 launch file usage with parameters
        auto temp_node = std::make_shared<rclcpp::Node>("orbslam3_param_loader");

        temp_node->declare_parameter("vocab_path", "");
        temp_node->declare_parameter("config_path", "");

        temp_node->get_parameter("vocab_path", vocab_path);
        temp_node->get_parameter("config_path", config_path);

        if (vocab_path.empty() || config_path.empty()) {
            RCLCPP_ERROR(temp_node->get_logger(), "Missing vocab_path or config_path as parameters.");
            return 1;
        }
    }

    bool visualization = true;
    ORB_SLAM3::System SLAM(vocab_path, config_path, ORB_SLAM3::System::MONOCULAR, visualization);
    auto node = std::make_shared<MonocularSlamNode>(&SLAM);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

