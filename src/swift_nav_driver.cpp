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

#include "swift_nav_ros2/swift_nav_driver.hpp"
#include <iostream>
#include <utility>
#include <map>


namespace swift_nav {

    SwiftNavDriver::SwiftNavDriver(const char *serial_name, std::string frame_id, int baudrate) : m_reader(serial_name,
                                                                                                           baudrate) {
        this->m_handler = new GNSSHandler(&m_s, frame_id);
        SwiftNavDriver::init();
    }

    void SwiftNavDriver::init() {
        m_s.set_reader(&m_reader);
    }

    void SwiftNavDriver::process() {
        while (!m_handler->get_msg_flag()) {
            m_s.process();
        }
        m_handler->set_msg_flag(false);
    }

    std::pair<void *, std::string> SwiftNavDriver::getOutput() {
        return m_handler->getOutput();
    }

}  // namespace swift_nav
