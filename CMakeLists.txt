# Copyright 2022 Perception for Physical Interaction Laboratory at Poznan University of Technology
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(swift_nav_ros2)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(SWIFT_NAV_ROS2_LIB_SRC
  src/swift_nav_driver.cpp
)

set(SWIFT_NAV_ROS2_LIB_HEADERS
  include/swift_nav_ros2/swift_nav_driver.hpp
  include/swift_nav_ros2/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${SWIFT_NAV_ROS2_LIB_SRC}
  ${SWIFT_NAV_ROS2_LIB_HEADERS}
)

target_link_libraries(${PROJECT_NAME} /usr/local/lib/libsbp.a)

set(SWIFT_NAV_ROS2_NODE_SRC
  src/swift_nav_driver_node.cpp
)

set(SWIFT_NAV_ROS2_NODE_HEADERS
  include/swift_nav_ros2/swift_nav_driver_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${SWIFT_NAV_ROS2_NODE_SRC}
  ${SWIFT_NAV_ROS2_NODE_HEADERS}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "swift_nav::SwiftNavDriverNode"
  EXECUTABLE ${PROJECT_NAME}_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
#  set(TEST_SOURCES test/test_swift_nav_driver.cpp)
#  set(TEST_SWIFT_NAV_ROS2_EXE test_swift_nav_driver)
#  ament_add_gtest(${TEST_SWIFT_NAV_ROS2_EXE} ${TEST_SOURCES})
#  target_link_libraries(${TEST_SWIFT_NAV_ROS2_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
