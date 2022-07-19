// Copyright 2022 Perception for Physical Interaction Laboratory at Poznan University of Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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