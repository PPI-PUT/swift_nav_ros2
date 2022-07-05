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

#ifndef SWIFT_NAV_ROS2__VISIBILITY_CONTROL_HPP_
#define SWIFT_NAV_ROS2__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(SWIFT_NAV_ROS2_BUILDING_DLL) || defined(SWIFT_NAV_ROS2_EXPORTS)
    #define SWIFT_NAV_ROS2_PUBLIC __declspec(dllexport)
    #define SWIFT_NAV_ROS2_LOCAL
  #else  // defined(SWIFT_NAV_ROS2_BUILDING_DLL) || defined(SWIFT_NAV_ROS2_EXPORTS)
    #define SWIFT_NAV_ROS2_PUBLIC __declspec(dllimport)
    #define SWIFT_NAV_ROS2_LOCAL
  #endif  // defined(SWIFT_NAV_ROS2_BUILDING_DLL) || defined(SWIFT_NAV_ROS2_EXPORTS)
#elif defined(__linux__)
  #define SWIFT_NAV_ROS2_PUBLIC __attribute__((visibility("default")))
  #define SWIFT_NAV_ROS2_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define SWIFT_NAV_ROS2_PUBLIC __attribute__((visibility("default")))
  #define SWIFT_NAV_ROS2_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // SWIFT_NAV_ROS2__VISIBILITY_CONTROL_HPP_
