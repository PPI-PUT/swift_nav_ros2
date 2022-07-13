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

#ifndef SWIFT_NAV_ROS2__SWIFT_NAV_ROS2_HPP_
#define SWIFT_NAV_ROS2__SWIFT_NAV_ROS2_HPP_

#include <swift_nav_ros2/visibility_control.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <swift_nav_ros2/sbp_serial_reader.hpp>
#include <swift_nav_ros2/gnss_handler.hpp>

#include <string>


namespace swift_nav {

    class SWIFT_NAV_ROS2_PUBLIC SwiftNavDriver {
    public:
        SwiftNavDriver(const char *serial_name, std::string frame_id);
        void set_parameters(int64_t baudrate);
        int32_t check_param() const;
        void init();
        void process();
        std::pair<void *, std::string> getOutput();

    private:
        sbp::State m_s;
        SbpSerialReader m_reader;
        GNSSHandler* m_handler;
        // Default parameters
        int64_t m_baudrate;
    };

}  // namespace swift_nav

#endif  // SWIFT_NAV_ROS2__SWIFT_NAV_ROS2_HPP_
