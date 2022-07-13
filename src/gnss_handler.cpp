//
// Created by tomek on 7/7/22.
//

#include <iostream>
#include <swift_nav_ros2/gnss_handler.hpp>

namespace swift_nav {

    GNSSHandler::GNSSHandler(sbp::State *state, std::string& frame_id)
            : sbp::MessageHandler<sbp_msg_pos_llh_cov_t>(state), frame_id(frame_id) {
        m_msg_flag = false;
    }

    void GNSSHandler::handle_sbp_msg(uint16_t sender_id, const sbp_msg_pos_llh_cov_t &msg) {
        (void) sender_id;
        convert_to_ros_msg_pos_llh_cov(&msg);
        m_msg_flag = true;
        output.first = &m_navSatFixLlh;
        output.second = "nav";
    }

    void GNSSHandler::convert_to_ros_msg_pos_llh_cov(const sbp_msg_pos_llh_cov_t *msg) {
        m_navSatFixLlh.altitude = msg->height;
        m_navSatFixLlh.latitude = msg->lat;
        m_navSatFixLlh.longitude = msg->lon;
        m_navSatFixLlh.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        const std::array<double, 9> cov = {static_cast<double>(msg->cov_n_n), static_cast<double>(msg->cov_n_e),
                                           static_cast<double>(msg->cov_n_d),
                                           static_cast<double>(msg->cov_n_e), static_cast<double>(msg->cov_e_e),
                                           static_cast<double>(msg->cov_e_d),
                                           static_cast<double>(msg->cov_n_d), static_cast<double>(msg->cov_e_d),
                                           static_cast<double>(msg->cov_d_d)
        };
        m_navSatFixLlh.position_covariance = cov;
        m_navSatFixLlh.status.status = static_cast<signed char>(msg->flags);
        m_navSatFixLlh.header.frame_id = frame_id;
    }

    bool GNSSHandler::get_msg_flag() const {
        return m_msg_flag;
    }

    void GNSSHandler::set_msg_flag(bool flag) {
        m_msg_flag = flag;
    }

    std::pair<void *, std::string> GNSSHandler::getOutput() {
        return output;
    }
}