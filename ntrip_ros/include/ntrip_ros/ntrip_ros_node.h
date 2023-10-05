// Copyright 2021 Institute of Automatic Control RWTH Aachen University
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
//
// Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//

#ifndef NTRIP_ROS_NTRIP_ROS_H
#define NTRIP_ROS_NTRIP_ROS_H

#pragma once

#include <thread>
#include <chrono>
#include <iostream>
#include <ctime>
#include <shared_mutex>
#include <boost/asio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <irt_msgs/msg/rtcml1_e1.hpp>
#include <irt_msgs/msg/rtcm1004.hpp>
#include <irt_msgs/msg/rtcm1005.hpp>
#include <irt_msgs/msg/rtcmv3.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <ntrip_ros/srv/update_reference_station.hpp>

#include <ntrip_ros/msg/ntrip_caster.hpp>

#include "ntripcom.h"
#include "rtcm/rtcm_parser.h"
#include "rtcm/rtcm_types.h"
#include "utils.h"
#include "udp_service.h"

#include "schema/sfrtcm_generated.h"

namespace ntrip_ros {
    typedef std::unique_ptr<flatbuffers::FlatBufferBuilder> FlatBufferBuilderPtr;
class NtripRosNode : public rclcpp::Node{
public:
    explicit NtripRosNode();
    ~NtripRosNode() override;

private:
    static std::chrono::system_clock::duration duration_since_midnight() {
        auto now = std::chrono::system_clock::now();

        time_t tnow = std::chrono::system_clock::to_time_t(now);
        tm *date = std::localtime(&tnow);
        date->tm_hour = 0;
        date->tm_min = 0;
        date->tm_sec = 0;
        auto midnight = std::chrono::system_clock::from_time_t(std::mktime(date));

        return now-midnight;
    }

    bool send_udp(const uint8_t* data, size_t size)
    {
        try {
            udp_socket_->open(boost::asio::ip::udp::v4());
            udp_socket_->send_to(boost::asio::buffer(data, size), *udp_ep_);
            udp_socket_->close();

            std::cout << " Send buffer with size " << size << std::endl;
            return true;
        } catch (const boost::system::system_error& ex) {
            // Exception thrown!
            // Examine ex.code() and ex.what() to see what went wrong!
            RCLCPP_WARN(this->get_logger(), "Couldnot send udp!");
            return false;
        }
    }

    void on_rtcmv3_deadline_timer(const boost::system::error_code& error)
    {
      if(error != boost::asio::error::operation_aborted)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "!!! No RTCMV3 msg since " << msg_deadline_timer_expires_time_ << " sec. Resetting everything ...");
        reset_io_service_ = true;
        rclcpp::sleep_for(1s);
        ntrip_client_->stop();
        ntrip_client_->start(ntrip_settings_);
      }
    }

private:
    ntrip::NtripCom::settings_t ntrip_settings_;
    ntrip::NtripCom::NMEA_GGA_ref_t GGA_ref_{};
    std::vector<double> default_llh_;
    std::unique_ptr<ntrip::NtripCom> ntrip_client_;
    std::unique_ptr<RtcmParser> rtcm_parser_;
    bool pub_1004_{}, pub_1005_{}, pub_rtcmv3_{}, pub_l1e1_{};
    boost::asio::io_service io_service_;
    std::unique_ptr<std::thread> io_service_thread_;

    std::shared_mutex ntrip_setting_mutex_;

    FlatBufferBuilderPtr rtcm_bd_;

    std::atomic_bool reset_io_service_ = false;

    std::unique_ptr<boost::asio::ip::udp::socket> udp_socket_;
    std::unique_ptr<boost::asio::ip::udp::endpoint > udp_ep_;

    rclcpp::TimerBase::SharedPtr update_gga_timer_;
    rclcpp::TimerBase::SharedPtr publish_ntrip_settings_timer_;
    rclcpp::TimerBase::SharedPtr ntrip_monitoring_timer_;
    rclcpp::Time last_rtcm_msg_time_;
    rclcpp::Publisher<ntrip_ros::msg::NtripCaster>::SharedPtr current_ntrip_setting_pub_;
    rclcpp::Publisher<irt_msgs::msg::RTCML1E1>::SharedPtr rtcm_L1E1_pub_;
    rclcpp::Publisher<irt_msgs::msg::RTCM1004>::SharedPtr rtcm_1004_pub_;
    rclcpp::Publisher<irt_msgs::msg::RTCM1005>::SharedPtr rtcm_1005_pub_;
    rclcpp::Publisher<irt_msgs::msg::RTCMV3>::SharedPtr rtcmv3_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr llh_sub_;
    int msg_deadline_timer_expires_time_ = 10;

    std::mutex gga_mutex_;

    rclcpp::Service<ntrip_ros::srv::UpdateReferenceStation>::SharedPtr update_ref_station_srv_;
};







}
#endif //NTRIP_ROS_NTRIP_ROS_H
