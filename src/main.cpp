//
// Created by tomek on 7/7/22.
//

#include <rclcpp/rclcpp.hpp>
#include "swift_nav_ros2/swift_nav_driver_node.hpp"

#include <iostream>
#include <stdexcept>
#include <string>


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions nodeOptions;

    auto swiftNavDriverNode = std::make_shared<swift_nav::SwiftNavDriverNode>(nodeOptions);
    exec.add_node(swiftNavDriverNode);

    while (rclcpp::ok()) {
        swiftNavDriverNode->process();
        exec.spin_once(std::chrono::nanoseconds(10000));
    }

    rclcpp::shutdown();

    return 0;
}