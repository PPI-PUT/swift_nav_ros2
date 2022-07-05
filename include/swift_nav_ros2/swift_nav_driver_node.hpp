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

#ifndef SWIFT_NAV_ROS2__SWIFT_NAV_ROS2_NODE_HPP_
#define SWIFT_NAV_ROS2__SWIFT_NAV_ROS2_NODE_HPP_

#include <swift_nav_ros2/swift_nav_driver.hpp>

#include <rclcpp/rclcpp.hpp>


namespace swift_nav
{

using SwiftNavDriverPtr = std::unique_ptr<swift_nav::SwiftNavDriver>;

class SWIFT_NAV_ROS2_PUBLIC SwiftNavDriverNode : public rclcpp::Node
{
public:
  explicit SwiftNavDriverNode(const rclcpp::NodeOptions & options);
  SwiftNavDriverPtr m_swift_nav{nullptr};
private:
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr m_gnss_publisher;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_subscribers;
  rclcpp::TimerBase::SharedPtr m_timer;
  void onTimer();

};
}  // namespace swift_nav

#endif  // SWIFT_NAV_ROS2__SWIFT_NAV_ROS2_NODE_HPP_
