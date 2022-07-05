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


namespace swift_nav
{

SwiftNavDriverNode::SwiftNavDriverNode(const rclcpp::NodeOptions & options)
:  Node("swift_nav_driver", options)
{
  const auto baudrate = declare_parameter("baudrate").get<int64_t>();
  const auto update_rate = declare_parameter("update_rate").get<int64_t>();
  m_swift_nav = std::make_unique<swift_nav::SwiftNavDriver>();
  m_swift_nav->set_parameters(baudrate, update_rate);

  // Publisher
  m_gnss_publisher =
    this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "gnss/nav_sat_fix",
      1);

  m_callback_group_subscribers = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto timer_callback = std::bind(&SwiftNavDriverNode::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / static_cast<double>(update_rate)));

  m_timer = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(m_timer, m_callback_group_subscribers);

}

void SwiftNavDriverNode::onTimer()
{
  m_swift_nav->check_param();
}

}  // namespace swift_nav

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(swift_nav::SwiftNavDriverNode)
