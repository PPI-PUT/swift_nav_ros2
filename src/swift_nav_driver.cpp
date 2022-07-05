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


namespace swift_nav
{

SwiftNavDriver::SwiftNavDriver()
{

}

void SwiftNavDriver::set_parameters(int64_t baudrate,
                                    int64_t update_rate)
{
  m_baudrate = baudrate;
  m_update_rate = update_rate;
}

int32_t SwiftNavDriver::check_param() const
{
    std::cout << "Baudrate: " << m_baudrate << std::endl;
    std::cout << "Update rate: " << m_update_rate << std::endl;
    return 0;
}

}  // namespace swift_nav
