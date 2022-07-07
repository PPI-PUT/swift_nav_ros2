//
// Created by tomek on 7/7/22.
//

#ifndef BUILD_GNSS_HANDLER_HPP
#define BUILD_GNSS_HANDLER_HPP

#include <libsbp/cpp/state.h>
#include <libsbp/cpp/message_handler.h>
#include <libsbp/legacy/cpp/frame_handler.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <libsbp/imu.h>

namespace swift_nav {

    class GNSSHandler : private sbp::MessageHandler<sbp_msg_pos_llh_cov_t> {

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
        void handle_sbp_msg(uint16_t sender_id, const sbp_msg_pos_llh_cov_t& msg) override;
        void convert_to_ros_msg_pos_llh_cov(const sbp_msg_pos_llh_cov_t* msg);

    };
} // namespace swift_nav

#endif //BUILD_GNSS_HANDLER_HPP
