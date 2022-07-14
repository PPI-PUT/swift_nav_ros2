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

#include "swift_nav_ros2/swift_nav_driver_node.hpp"


namespace swift_nav {

    SwiftNavDriverNode::SwiftNavDriverNode(const rclcpp::NodeOptions &options)
            : Node("swift_nav_driver", options) {
        const auto baudrate = declare_parameter("baudrate").get<int>();
        const auto port = declare_parameter("port").get<std::string>();
        const auto frame_id = declare_parameter("frame_id").get<std::string>();

        m_swift_nav = std::make_unique<swift_nav::SwiftNavDriver>(port.c_str(), frame_id, baudrate);

        // Publishers
        createPublishers();

    }

    void SwiftNavDriverNode::process() {
      m_swift_nav->process();
      std::pair<void *, std::string> data = m_swift_nav->getOutput();
      if (data.second == "nav") {
        (*static_cast<sensor_msgs::msg::NavSatFix *>(data.first)).header.stamp = Node::now();
        m_gnss_publisher->publish((*static_cast<sensor_msgs::msg::NavSatFix *>(data.first)));
      }

      else if (data.second == "mov") {
        (*static_cast<geometry_msgs::msg::TwistWithCovarianceStamped *>(data.first)).header.stamp = Node::now();
        m_speed_publisher->publish(
                (*static_cast<geometry_msgs::msg::TwistWithCovarianceStamped *>( data.first)));
      }

      else if (data.second == "time") {
        (*static_cast<sensor_msgs::msg::TimeReference *>(data.first)).header.stamp = Node::now();
        m_gps_time_publisher->publish(
                (*static_cast<sensor_msgs::msg::TimeReference *>( data.first)));
      }

      else if (data.second == "imu") {
        (*static_cast<sensor_msgs::msg::Imu *>(data.first)).header.stamp = Node::now();
        m_imu_publisher->publish(
                (*static_cast<sensor_msgs::msg::Imu *>( data.first)));
      }
    }

    void SwiftNavDriverNode::createPublishers() {

      m_gnss_publisher =
              this->create_publisher<sensor_msgs::msg::NavSatFix>(
                      "/gnss/nav_sat_fix",
                      1);

      m_speed_publisher =
              this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
                      "/gnss/nav_speed",
                      1);

      m_gps_time_publisher =
              this->create_publisher<sensor_msgs::msg::TimeReference>(
                      "/gnss/nav_time",
                      1);

      m_imu_publisher =
              this->create_publisher<sensor_msgs::msg::Imu>(
                      "/gnss/imu",
                      1);
    }

}  // namespace swift_nav

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(swift_nav::SwiftNavDriverNode)
