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

#ifndef BUILD_GNSS_HANDLER_HPP
#define BUILD_GNSS_HANDLER_HPP

#include <libsbp/cpp/state.h>
#include <libsbp/cpp/message_handler.h>
#include <libsbp/legacy/cpp/frame_handler.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <libsbp/legacy/imu.h>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

namespace swift_nav {

    class GNSSHandler : private sbp::MessageHandler<sbp_msg_pos_llh_cov_t,
                                                    sbp_msg_vel_ned_cov_t,
                                                    sbp_msg_gps_time_t,
                                                    sbp_msg_imu_raw_t> {

    public:
        GNSSHandler(sbp::State *state, std::string &frame_id);
        std::pair<void*, std::string> getOutput();
        bool get_msg_flag() const;
        void set_msg_flag(bool flag);

    private:
        std::pair<void*, std::string> output;
        bool m_msg_flag;
        std::string frame_id;

        sensor_msgs::msg::NavSatFix m_navSatFixLlh;
        geometry_msgs::msg::TwistWithCovarianceStamped m_twistWithCovarianceStamped;
        sensor_msgs::msg::TimeReference m_timeReference;
        sensor_msgs::msg::Imu m_imu;


        void handle_sbp_msg(uint16_t sender_id, const sbp_msg_pos_llh_cov_t& msg) override;
        void handle_sbp_msg(uint16_t sender_id, const sbp_msg_vel_ned_cov_t& msg) override;
        void handle_sbp_msg(uint16_t sender_id, const sbp_msg_gps_time_t& msg) override;
        void handle_sbp_msg(uint16_t sender_id, const sbp_msg_imu_raw_t& msg) override;

        void convert_to_ros_msg_pos_llh_cov(const sbp_msg_pos_llh_cov_t* msg);
        void convert_to_ros_msg_vel_ned_cov(const sbp_msg_vel_ned_cov_t* msg);
        void convert_to_ros_time(const sbp_msg_gps_time_t *msg);
        void convert_to_ros_imu(const sbp_msg_imu_raw_t *msg);

    };
} // namespace swift_nav

#endif //BUILD_GNSS_HANDLER_HPP
