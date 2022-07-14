//
// Created by tomek on 7/7/22.
//

#include <iostream>
#include <swift_nav_ros2/gnss_handler.hpp>

namespace swift_nav {

    GNSSHandler::GNSSHandler(sbp::State *state, std::string& frame_id)
            : sbp::MessageHandler<sbp_msg_pos_llh_cov_t,
                                  sbp_msg_vel_ned_cov_t,
                                  sbp_msg_gps_time_t,
                                  sbp_msg_imu_raw_t>(state),
                                  frame_id(frame_id) {
        m_msg_flag = false;
    }

    void GNSSHandler::handle_sbp_msg(uint16_t sender_id, const sbp_msg_pos_llh_cov_t &msg) {
        (void) sender_id;
        convert_to_ros_msg_pos_llh_cov(&msg);
        m_msg_flag = true;
        output.first = &m_navSatFixLlh;
        output.second = "nav";
    }

    void GNSSHandler::handle_sbp_msg(uint16_t sender_id, const sbp_msg_vel_ned_cov_t &msg) {
      (void) sender_id;
      convert_to_ros_msg_vel_ned_cov(&msg);
      m_msg_flag = true;
      output.first = &m_twistWithCovarianceStamped;
      output.second = "mov";
    }

    void GNSSHandler::handle_sbp_msg(uint16_t sender_id, const sbp_msg_gps_time_t &msg) {
      (void) sender_id;
      convert_to_ros_time(&msg);
      m_msg_flag = true;
      output.first = &m_timeReference;
      output.second = "time";
    }

    void GNSSHandler::handle_sbp_msg(uint16_t sender_id, const sbp_msg_imu_raw_t &msg) {
      (void) sender_id;
      convert_to_ros_imu(&msg);
      m_msg_flag = true;
      output.first = &m_imu;
      output.second = "imu";
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

    void GNSSHandler::convert_to_ros_msg_vel_ned_cov(const sbp_msg_vel_ned_cov_t *msg) {
      // ROS x -> northing, ROS y -> easting, ROS z -> down; [m/s]
      m_twistWithCovarianceStamped.header.frame_id = frame_id;
      m_twistWithCovarianceStamped.twist.twist.linear.x = msg->n * 1000;
      m_twistWithCovarianceStamped.twist.twist.linear.y = msg->e * 1000;
      m_twistWithCovarianceStamped.twist.twist.linear.z = msg->d * 1000;
      const std::array<double, 36> cov = {static_cast<double>(msg->cov_n_n),
                                          static_cast<double>(msg->cov_n_e),
                                          static_cast<double>(msg->cov_n_d), 0, 0, 0,
                                          static_cast<double>(msg->cov_n_e),
                                          static_cast<double>(msg->cov_e_e),
                                          static_cast<double>(msg->cov_e_d), 0, 0, 0,
                                          static_cast<double>(msg->cov_n_d),
                                          static_cast<double>(msg->cov_e_d),
                                          static_cast<double>(msg->cov_d_d), 0, 0, 0,
                                          0, 0, 0, 1, 0, 0,
                                          0, 0, 0, 0, 1, 0,
                                          0, 0, 0, 0, 0, 1
      };
      m_twistWithCovarianceStamped.twist.covariance = cov;
    }

    void GNSSHandler::convert_to_ros_time(const sbp_msg_gps_time_t *msg) {
      m_timeReference.header.frame_id = frame_id;
      m_timeReference.time_ref.nanosec = static_cast<unsigned int>(msg->ns_residual);
      m_timeReference.time_ref.sec = static_cast<int>(msg->tow);
      m_timeReference.source = "gnss";
    }

    void GNSSHandler::convert_to_ros_imu(const sbp_msg_imu_raw_t *msg) {
      m_imu.header.frame_id = frame_id;
      m_imu.orientation.x = msg->gyr_x;
      m_imu.orientation.y = msg->gyr_y;
      m_imu.orientation.z = msg->gyr_z;
      m_imu.linear_acceleration.x = msg->acc_x;
      m_imu.linear_acceleration.y = msg->acc_y;
      m_imu.linear_acceleration.z = msg->acc_z;
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