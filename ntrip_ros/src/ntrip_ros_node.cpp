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


#include <cstdio>

#include "ntrip_ros_node.h"

namespace ntrip_ros
{
    NtripRosNode::NtripRosNode() :
    Node("NtripRosNode"), io_service_(){
      this->declare_parameter<std::string>("ntrip_server_uri", "80.158.61.104");
      this->declare_parameter<int>("ntrip_server_port", 2101);
      this->declare_parameter<std::string>("ntrip_username", "nw-710989");
      this->declare_parameter<std::string>("ntrip_password", "railir");
      this->declare_parameter<std::string>("ntrip_mountpoint", "VRS_3_3G_NW");
      this->declare_parameter<int>("update_GGA_period", 10);
      this->declare_parameter<bool>("pub_1004", true);
      this->declare_parameter<bool>("pub_1005", true);
      this->declare_parameter<bool>("pub_rtcmv3", true);
      this->declare_parameter<bool>("pub_l1e1", true);
      this->declare_parameter<std::string>("bachmann_udp_ip", "192.168.31.12");
      this->declare_parameter<int>("bachmann_udp_port", 10021);
      this->declare_parameter<int>("msg_deadline_timer_expires_time", 8);

      //udp_srv_->set_address(this->get_parameter("bachmann_udp_ip").as_string(),
       //                     this->get_parameter("bachmann_udp_port").as_int());

      msg_deadline_timer_expires_time_ = this->get_parameter("msg_deadline_timer_expires_time").as_int();
      udp_socket_ = std::make_unique<boost::asio::ip::udp::socket>(io_service_);
      udp_ep_ = std::make_unique<boost::asio::ip::udp::endpoint>(boost::asio::ip::address::from_string(this->get_parameter("bachmann_udp_ip").as_string()),
                                                                  this->get_parameter("bachmann_udp_port").as_int());
      rtcm_bd_ = std::make_unique<flatbuffers::FlatBufferBuilder>(2048);
      default_llh_ = {50.776298, 6.083714, 220.3385};
      this->declare_parameter<std::vector<double>>("start_llh", default_llh_);
      default_llh_ = this->get_parameter("start_llh").as_double_array();

      GGA_ref_.latitude_deg = default_llh_[0];
      GGA_ref_.longitude_deg = default_llh_[1];
      GGA_ref_.altitude_msl_m = default_llh_[2];
      auto since_midnight = duration_since_midnight();
      auto hours = std::chrono::duration_cast<std::chrono::hours>(since_midnight);
      auto minutes = std::chrono::duration_cast<std::chrono::minutes>(since_midnight - hours);
      auto seconds = std::chrono::duration_cast<std::chrono::seconds>(since_midnight - hours - minutes);
      GGA_ref_.utc_hours_since_midnight= hours.count();
      GGA_ref_.utc_minutes_since_midnight = minutes.count();
      GGA_ref_.utc_seconds_since_midnight = seconds.count() + std::chrono::duration_cast<std::chrono::milliseconds>(since_midnight - hours - minutes - seconds).count() / 1000.;

      ntrip_settings_.mountpoint = this->get_parameter("ntrip_mountpoint").as_string();
      ntrip_settings_.server = this->get_parameter("ntrip_server_uri").as_string();
      ntrip_settings_.port = this->get_parameter("ntrip_server_port").as_int();
      ntrip_settings_.password = this->get_parameter("ntrip_password").as_string();
      ntrip_settings_.user = this->get_parameter("ntrip_username").as_string();

      rtcm_parser_ = std::make_unique<RtcmParser>(1024);
      ntrip_client_ = std::make_unique<ntrip::NtripCom>(io_service_, *this);
      ntrip_client_->on_RTCM_bytes(boost::bind(&RtcmParser::addBytesToBuffer, rtcm_parser_.get(), boost::placeholders::_1, boost::placeholders::_2));

      this->get_parameter("pub_1004", pub_1004_);
      this->get_parameter("pub_1005", pub_1005_);
      this->get_parameter("pub_l1e1",  pub_l1e1_);
      this->get_parameter("pub_rtcmv3", pub_rtcmv3_);
      if(pub_l1e1_)
      {
          rtcm_L1E1_pub_ = this->create_publisher<irt_msgs::msg::RTCML1E1>("rtcmL1E1", rclcpp::SystemDefaultsQoS());
      }
      if(pub_1004_)
      {
          rtcm_1004_pub_ = this->create_publisher<irt_msgs::msg::RTCM1004>("rtcm1004", rclcpp::SystemDefaultsQoS());
      }
      if(pub_1005_)
      {
          rtcm_1005_pub_ = this->create_publisher<irt_msgs::msg::RTCM1005>("rtcm1005", rclcpp::SystemDefaultsQoS());
      }
      if(pub_rtcmv3_)
      {
          rtcmv3_pub_ = this->create_publisher<irt_msgs::msg::RTCMV3>("rtcmv3", rclcpp::SystemDefaultsQoS());
      }

      current_ntrip_setting_pub_ = this->create_publisher<ntrip_ros::msg::NtripCaster>("currentNtripSettings", rclcpp::SystemDefaultsQoS());

      llh_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/novatel/gps/fix",
                                                                        rclcpp::SystemDefaultsQoS(),
                                                                         [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
                                                                        {
                                                                          //const std::lock_guard<std::mutex> lg(this->gga_mutex_);
                                                                          //RCLCPP_WARN_STREAM(this->get_logger(), "got new PVT: lat " << msg->latitude << " lon: " << msg->longitude);

                                                                          GGA_ref_.latitude_deg = msg->latitude;
                                                                          GGA_ref_.longitude_deg = msg->longitude;
                                                                          GGA_ref_.altitude_msl_m = msg->altitude;
                                                                            auto since_midnight = duration_since_midnight();
                                                                            auto hours = std::chrono::duration_cast<std::chrono::hours>(since_midnight);
                                                                            auto minutes = std::chrono::duration_cast<std::chrono::minutes>(since_midnight - hours);
                                                                            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(since_midnight - hours - minutes);
                                                                            GGA_ref_.utc_hours_since_midnight= hours.count();
                                                                            GGA_ref_.utc_minutes_since_midnight = minutes.count();
                                                                            GGA_ref_.utc_seconds_since_midnight = seconds.count() + std::chrono::duration_cast<std::chrono::milliseconds>(since_midnight - hours - minutes - seconds).count() / 1000.;

                                                                        });

        update_gga_timer_ = this->create_wall_timer(std::chrono::seconds(this->get_parameter("update_GGA_period").as_int()),
                                [this] {
                                    //const std::lock_guard<std::mutex> lg(this->gga_mutex_);
                                    RCLCPP_INFO_STREAM(this->get_logger(),
                                                       "Update GGA Reference with new lon: " << GGA_ref_.longitude_deg
                                                                                             << " lat: "
                                                                                             << GGA_ref_.latitude_deg
                                                                                             << " height: "
                                                                                             << GGA_ref_.altitude_msl_m
                                                                                             << " at: HH:MM:SS"
                                                                                             << GGA_ref_.utc_hours_since_midnight
                                                                                             << " : "
                                                                                             << GGA_ref_.utc_minutes_since_midnight
                                                                                             << " : "
                                                                                             << GGA_ref_.utc_seconds_since_midnight);
                                    if(this->ntrip_client_)
                                      this->ntrip_client_->set_own_position(GGA_ref_);
                                });

        publish_ntrip_settings_timer_ = this->create_wall_timer(std::chrono::seconds(5),
                                    [this] {
                                     ntrip_setting_mutex_.lock_shared();

                                     ntrip_ros::msg::NtripCaster msg;
                                     msg.header.stamp = this->now();
                                     msg.mountpoint = ntrip_settings_.mountpoint;
                                     msg.uri = ntrip_settings_.server;
                                     msg.port = ntrip_settings_.port;

                                     current_ntrip_setting_pub_->publish(msg);
                                     ntrip_setting_mutex_.unlock_shared();
                                });




      rtcm_parser_->registerCbMsg1004([this](const boost::shared_ptr<RTCM_30_1004_t>& msg) -> void
      {
          if (this->pub_1004_)
          {
              RCLCPP_INFO(this->get_logger(), "On RTCM1004 publishing...");
              irt_msgs::msg::RTCM1004 msg_1004 = ntrip::constructRTCM1004Msg(msg);
              msg_1004.header.stamp = this->now();
              rtcm_1004_pub_->publish(msg_1004);
          } else
              RCLCPP_WARN(this->get_logger(), "On RTCM1004, but NOT publishing...");
          auto tmp1 = ntrip::Convert1004(*msg);
          auto tmp2 = ntrip::TransDataDictionary_1004(tmp1);

      });

      rtcm_parser_->registerCbMsg1005([this](const boost::shared_ptr<RTCM_30_1005_t>& msg) -> void
      {
          if (this->pub_1005_)
          {
              RCLCPP_INFO(this->get_logger(), "On RTCM1005 publishing...");
              irt_msgs::msg::RTCM1005 msg_1005 = ntrip::constructRTCM1005Msg(msg);
              msg_1005.header.stamp = this->now();
              rtcm_1005_pub_->publish(msg_1005);
          } else
              RCLCPP_WARN(this->get_logger(), "On RTCM1005, but NOT publishing...");

          auto tmp1 = ntrip::Convert1005(*msg);
          auto tmp2 = ntrip::TransDataDictionary_1005(tmp1);

      });

      rtcm_parser_->registerCbRTCM33([this](const boost::shared_ptr<RTCM_3_3_t>& msg) -> void
      {
          static double last_tow = 0;
          last_rtcm_msg_time_ = rclcpp::Time(this->now(), RCL_ROS_TIME);
          if (this->pub_l1e1_ && last_tow != msg->TOW)
          {
              last_tow = msg->TOW;
              RCLCPP_INFO(this->get_logger(), "On RTCM33L1E1 publishing...");
              irt_msgs::msg::RTCML1E1 rtcml1E1_msg = ntrip::ConstructRTCM33_L1E1Msg(msg);
              rtcml1E1_msg.header.stamp = this->now();
              this->rtcm_L1E1_pub_->publish(rtcml1E1_msg);

              rtcm_bd_->Clear();
              //IRT::SFusion::RTCM3L1E1Builder rtcm_builder(*rtcm_bd_);
              IRT::SFusion::SFusionTime sf_time;
              std::vector<double> base_vec(rtcml1E1_msg.base.begin(), rtcml1E1_msg.base.end());
              std::vector<uint8_t> svid_vec(rtcml1E1_msg.svid.begin(), rtcml1E1_msg.svid.end());
              std::vector<double> psr_vec(rtcml1E1_msg.pseudorange.begin(), rtcml1E1_msg.pseudorange.end());
              std::vector<uint8_t> type_vec(rtcml1E1_msg.type.begin(), rtcml1E1_msg.type.end());
              std::vector<double> cp_vec(rtcml1E1_msg.carrier.begin(), rtcml1E1_msg.carrier.end());
              std::vector<double> cn0_vec(rtcml1E1_msg.cn0.begin(), rtcml1E1_msg.cn0.end());
              std::vector<uint16_t> locktime_vec(rtcml1E1_msg.locktime.begin(), rtcml1E1_msg.locktime.end());
              auto rtcm = IRT::SFusion::CreateRTCM3L1E1Direct(*rtcm_bd_, &sf_time,
                                                              rtcml1E1_msg.tow,
                                                              &base_vec,
                                                              &svid_vec,
                                                              &psr_vec,
                                                              &type_vec,
                                                              &cp_vec,
                                                              &cn0_vec,
                                                              &locktime_vec,
                                                              rtcml1E1_msg.reference_station_id);
              rtcm_bd_->Finish(rtcm);
              auto bf_size = rtcm_bd_->GetSize();
              this->send_udp(rtcm_bd_->GetBufferPointer(), bf_size);


          } else
              RCLCPP_WARN(this->get_logger(), "On RTCM33L1E1, but NOT publishing...");
          //send_udp(p_dd4_3_3, Length_DD_RTCM_3_3_L1E1);

          if (this->pub_rtcmv3_)
          {
              RCLCPP_INFO(this->get_logger(), "On RTCM33v3 publishing...");
              irt_msgs::msg::RTCMV3 rtcmv3 = ntrip::constctRTCMV3Msg(msg);
              rtcmv3.header.stamp = this->now();
              this->rtcmv3_pub_->publish(rtcmv3);
          } else
              RCLCPP_WARN(this->get_logger(), "On RTCM33v3, but NOT publishing...");
      });

      RCLCPP_INFO(this->get_logger(), "On starting ...");
      ntrip_client_->start(ntrip_settings_);
      ntrip_client_->set_own_position(GGA_ref_);

      io_service_thread_ = std::make_unique<std::thread>([this]()->void{
          auto last_time = std::chrono::system_clock::now();
          while(rclcpp::ok())
          {
              try {
                  if(reset_io_service_)
                  {
                      reset_io_service_ = false;
                      io_service_.reset();
                      io_service_.restart();
                      RCLCPP_WARN(this->get_logger(), "RESET AND RESTARTED IO SERVICE!");
                  }
                  io_service_.run_one();

                  double duration = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - last_time).count();
                  if(duration > 5)
                  {
                      RCLCPP_INFO(this->get_logger(), "....... IO_SERVICE Heartbeat .......");
                      last_time = std::chrono::system_clock::now();
                  }
              }
              catch (std::exception& ex)
              {
                  RCLCPP_ERROR_STREAM(this->get_logger(), "IO_SERVICE Threading throwing error: " << ex.what());
                  reset_io_service_ = true;
              }
          }
      });


    update_ref_station_srv_ = this->create_service<ntrip_ros::srv::UpdateReferenceStation>("update_reference_station", [&](const std::shared_ptr<ntrip_ros::srv::UpdateReferenceStation::Request> request,
                                                                                                                           std::shared_ptr<ntrip_ros::srv::UpdateReferenceStation::Response> response) {
        
        
        ntrip_setting_mutex_.lock();

        if(request->mountpoint != ntrip_settings_.mountpoint || request->uri != ntrip_settings_.server || ntrip_settings_.port != request->port)
        {
          ntrip_settings_.mountpoint = request->mountpoint;
          ntrip_settings_.server = request->uri;
          ntrip_settings_.port = request->port;
          ntrip_settings_.password = request->password;
          ntrip_settings_.user = request->username;
          ntrip_client_->update_reference_station(ntrip_settings_);
          RCLCPP_WARN_STREAM(this->get_logger(), "UPDATING NTRIP SETTINGS: Server " << ntrip_settings_.server << " Port " << ntrip_settings_.port << " Mountpoint " << ntrip_settings_.mountpoint);
        }else
        {
          RCLCPP_WARN_STREAM(this->get_logger(), "On ntrip service call, but NOT UPDATING NTRIP SETTINGS: Server " << ntrip_settings_.server << " Port " << ntrip_settings_.port << " Mountpoint " << ntrip_settings_.mountpoint);
      
        }
        
        ntrip_setting_mutex_.unlock();
        response->success = true;
    });

        last_rtcm_msg_time_ = rclcpp::Time(this->now(), RCL_ROS_TIME);

        ntrip_monitoring_timer_ = this->create_wall_timer(std::chrono::seconds(10),
                                                          [this] {
            auto now = rclcpp::Time(this->now(), RCL_ROS_TIME);
            RCLCPP_WARN_STREAM(this->get_logger(), "on ntrip client monitoring timer at: " << std::fixed << now.seconds() << " Checking if the msg is received ...");
            auto time_diff_sec = (now - last_rtcm_msg_time_).seconds();
            if( time_diff_sec < msg_deadline_timer_expires_time_)
                return;
            RCLCPP_ERROR_STREAM(this->get_logger(), "!!!!!! NO RTCM msg since " << time_diff_sec << " sec. Resetting everything...");
            reset_io_service_  = true;
            rclcpp::sleep_for(1s);
            ntrip_client_->stop();
            ntrip_client_->start(ntrip_settings_);
        });
    }

    NtripRosNode::~NtripRosNode()
    {
        //udp_srv_->End_UDP();
        io_service_.stop();
        io_service_thread_->join();
    }

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  //udp_srv->Start_UDP();
  //boost::asio::io_service io_service;
  auto ntrip_node = std::make_shared<ntrip_ros::NtripRosNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(ntrip_node);
    executor.spin();
  return 0;
}

