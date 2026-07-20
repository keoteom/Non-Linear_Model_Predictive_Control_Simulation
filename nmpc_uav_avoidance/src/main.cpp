/**
 * @file main.cpp
 * @brief Entry point for the NMPC UAV obstacle avoidance node.
 */

#include "nmpc_uav_avoidance/nmpc_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<nmpc_uav::NMPCNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}