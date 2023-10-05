/*
 * ntripcom.cpp
 *
 *  Created on: 15.06.2018
 *      Author: Eg: inital version
 *				Kd: added non-chunked mode, some bug fixes
 *				Zh: optimization
 */


#include <iostream>
#include <iomanip>
#include <boost/lexical_cast.hpp>
#include "ntripcom.h"

#define dataReceptionTimeoutSeconds 10

namespace ntrip
{
    NtripCom::NtripCom(boost::asio::io_service& io_service1, rclcpp::Node& node) : node_(node),
        io_service_(io_service1), resolver_(io_service_), tcp_socket_(io_service_)
    {
      retry_timer_ = std::make_unique<boost::asio::deadline_timer>(io_service_);
      data_reception_timeout_timer_ = std::make_unique<boost::asio::deadline_timer>(io_service_);
      status_.connected = false;
      status_.type = nc_status;
      status_.message = "disconnected";
    }

    void NtripCom::start(const settings_t& settings) {
      settings_ = settings;
      init();
    }

    void NtripCom::update_reference_station(const settings_t &settings) {
        stop();
        start(settings);
    }

    void NtripCom::init() {
      // form the request_
      RCLCPP_INFO_STREAM(node_.get_logger(), "NtripClient on initializing ...using ntrip server " << settings_.server
      << " port:" << settings_.port << " mountpoint: " << settings_.mountpoint);
      std::ostream request_stream(&request_);
      request_stream << "GET /" << settings_.mountpoint << " HTTP/1.1\r\n";
      request_stream << "Host: " << settings_.server << "\r\n";
      request_stream << "Ntrip-Version: Ntrip/2.0\r\n";
      request_stream << "User-Agent: NTRIP NtripClientPOSIX/$Revision: 1.50 $\r\n";
      request_stream << "Connection: close\r\n";
      std::string userAndPassword = settings_.user + ":" + settings_.password;
      request_stream << "Authorization: Basic " << base64_encode(reinterpret_cast<const unsigned char*>(userAndPassword.c_str()), (unsigned int)userAndPassword.size()) << "\r\n";
      request_stream << "\r\n\r\n"; // request_ must ending on empty line

      // setting timeout to stop initialization if no data is received
      resetDataReceptionTimeoutTimer();

      // Start an asynchronous resolve to translate the server and service names
      // into a list of endpoints.
      status_.connected = false;
      change_status(nc_status,"Resolving server address");
      boost::asio::ip::tcp::resolver::query query(settings_.server, boost::lexical_cast<std::string>(settings_.port));
      resolver_.async_resolve(query, boost::bind(&NtripCom::handle_resolve, this,
                                                       boost::asio::placeholders::error,
                                                       boost::asio::placeholders::iterator));
    }

    void NtripCom::stop() {
      retry_timer_->cancel();
      disconnect();
      change_status(nc_status,"stop");
      on_stop_signal_();
    }

    void NtripCom::disconnect() {
      status_.connected = false;
      tcp_socket_.close();
    }

    boost::signals2::connection NtripCom::on_RTCM_bytes(const rtcm_bytes_signal_t::slot_type &subscriber) {
      return rtcm_bytes_signal_.connect(subscriber);
    }

    boost::signals2::connection NtripCom::on_status_change(const on_status_signal_t::slot_type &subscriber) {
      return on_status_signal_.connect(subscriber);
    }

    boost::signals2::connection NtripCom::on_stop(const on_stop_signal_t::slot_type &subscriber) {
      return on_stop_signal_.connect(subscriber);
    }

    void NtripCom::handle_resolve(const boost::system::error_code& error, boost::asio::ip::tcp::resolver::iterator endpoint_iterator) {
      RCLCPP_WARN_STREAM(node_.get_logger(), "NtripClient on resolving ...using ntrip server " << settings_.server);
      if (!error) {
        // Attempt a connection to the first endpoint in the list. Each endpoint
        // will be tried until we successfully establish a connection.
        boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator;
        std::stringstream statusmsgstream;
        statusmsgstream << "Resolve server address \"" << settings_.server << "\": " << endpoint.address() << ":" << endpoint.port();
        change_status(nc_status,statusmsgstream.str());
        if(tcp_socket_.is_open())
          tcp_socket_.close();
        tcp_socket_.async_connect(endpoint, boost::bind(&NtripCom::handle_connect, this, boost::asio::placeholders::error, ++endpoint_iterator));
      } else if (error != boost::asio::error::operation_aborted) {
        std::stringstream errormsgstream;
        errormsgstream << "Could not resolve " << settings_.server << ":" << settings_.port << ": " << error.message();
        change_status(nc_error,errormsgstream.str());
        resetRetryTimer();
      }
    }

    void NtripCom::handle_connect(const boost::system::error_code& error, boost::asio::ip::tcp::resolver::iterator endpoint_iterator) {
      if (!error) {
        // The connection was successful. Send the request_.
        change_status(nc_status,"Connection to server successfully");
        RCLCPP_INFO(node_.get_logger(), "NtripClient connected successfully!");
        boost::asio::async_write(tcp_socket_, request_, boost::bind(&NtripCom::handle_write_request, this, boost::asio::placeholders::error, boost::placeholders::_2));
      } else if (endpoint_iterator != boost::asio::ip::tcp::resolver::iterator()) {
        // The connection failed. Try the next endpoint in the list.
        change_status(nc_status,"connection failed, trying next endpoint");
        tcp_socket_.close();
        boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator;
        tcp_socket_.async_connect(endpoint, boost::bind(&NtripCom::handle_connect, this, boost::asio::placeholders::error, ++endpoint_iterator));
      } else if (error != boost::asio::error::operation_aborted) {
        std::stringstream errormsgstream;
        errormsgstream << "Connect failed: " << error.message();
        change_status(nc_error,errormsgstream.str());
        resetRetryTimer();
      }
    }

    void NtripCom::handle_write_request(const boost::system::error_code& error, size_t bytes_transferred) {
      if (!error) {
        // Read the response_ status line.
        RCLCPP_INFO(node_.get_logger(), "NtripClient: writing request...");
        change_status(nc_status,"Request sent, reading status line");
        resetDataReceptionTimeoutTimer();
        boost::asio::async_read_until(tcp_socket_, response_, "\r\n", boost::bind(&NtripCom::handle_read_status_line, this, boost::asio::placeholders::error));
      } else if (error != boost::asio::error::operation_aborted) {
        std::stringstream errormsgstream;
        errormsgstream << "Write error: " << error.message();
        change_status(nc_error,errormsgstream.str());
        resetRetryTimer();
      }
    }

    void NtripCom::handle_read_status_line(const boost::system::error_code& error) {
      if (!error) {
        // Check that response_ is OK.
        std::istream response_stream(&response_);
        std::string http_version;
        std::string status_message;

        response_stream >> http_version;
        unsigned int status_code;
        response_stream >> status_code;

        getline(response_stream, status_message); // get rest of first line
        RCLCPP_INFO_STREAM(node_.get_logger(), "http_version: " << http_version << " status message: " << status_message);
        if (!response_stream || (http_version.substr(0, 5) != "HTTP/" && http_version.substr(0, 3) != "ICY")) {
          std::stringstream errormsgstream;
          errormsgstream << "Invalid server response_";
          change_status(nc_error,errormsgstream.str());
          std::cerr << "Invalid server response_: " << std::endl << &response_ << std::endl;
          stop(); // do not retry
          return;
        }

        if (status_code == 200) {
          // reponse was something like "HTTP/1.1 200 OK"
          // valid request_, desired NtripSource/mountpoint exists
          change_status(nc_status,"Request successful (200 OK)");
          handle_read_headers(error);
          // Read the response_ headers, which are terminated by a blank line.
          //boost::asio::async_read_until(tcp_socket_, response_, "\r\n\r\n", boost::bind(&NtripCom::handle_read_headers, this, boost::asio::placeholders::error));
        } else if (status_code == 404) {
          std::stringstream errormsgstream;
          errormsgstream << "Mountpoint \"" << settings_.mountpoint << "\" not found";
          change_status(nc_error,errormsgstream.str());
          stop(); // do not retry
        } else if (status_code == 401) {
          std::stringstream errormsgstream;
          errormsgstream << "Unauthorized user \"" << settings_.user << "\", password wrong?";
          change_status(nc_error,errormsgstream.str());
          stop(); // do not retry
        } else {
          std::stringstream errormsgstream;
          errormsgstream << "Server replay:" << status_code << status_message;
          change_status(nc_error,errormsgstream.str());
          stop(); // do not retry
        }
      } else if (error != boost::asio::error::operation_aborted) {
        std::stringstream errormsgstream;
        errormsgstream << "Read status line error: " << error.message();
        change_status(nc_error,errormsgstream.str());
        resetRetryTimer();
      }
    }

    void NtripCom::handle_read_headers(const boost::system::error_code& error) {

      if (!error) {
        // Process the response_ headers.
        std::istream response_stream(&response_);
        std::string header_line;
        bool contentTypeGnssData = false;
        transfer_encoding_chunked_ = false;
        while (getline(response_stream, header_line) && header_line != "\r") {

          // debug out
          std::cout << header_line << std::endl;

          if (header_line.find("Content-Type: gnss/data") != std::string::npos) contentTypeGnssData = true;

          if (header_line.find("Transfer-Encoding: chunked") != std::string::npos) transfer_encoding_chunked_ = true;
        }

        if (transfer_encoding_chunked_ && !contentTypeGnssData) {
          std::stringstream errormsgstream;
          errormsgstream << "Server replay header does not contain \"Content-Type: gnss/data\"";
          change_status(nc_error,errormsgstream.str());
          stop(); // do not retry
          return;
        }

        if (!transfer_encoding_chunked_) {
          std::stringstream errormsgstream;
          errormsgstream << "Server replay header does not contain \"Transfer-Encoding: chunked\", using non-chunked mode";
          change_status(nc_status, errormsgstream.str());
            //std::cout << "!!!!!!!!!!!!!!!!!!!!Server replay header does not contain \"Transfer-Encoding: chunked\", using non-chunked mode" << std::endl;
          //stop(); // do not retry
          //return;
        }


        // send nmea_request_message_ if set:
        send_nmea_request_message();
        // read RTCM data next:
        boost::asio::async_read(tcp_socket_, response_, boost::asio::transfer_at_least(1),
                                boost::bind(&NtripCom::handle_read_rtcm_data, this, boost::asio::placeholders::error, boost::placeholders::_2));
      } else if (error != boost::asio::error::operation_aborted) {
        std::stringstream errormsgstream;
        errormsgstream << "Error reading server response_ header: " << error.message();
        change_status(nc_error,errormsgstream.str());
        resetRetryTimer();
      }
    }

    void NtripCom::handle_read_rtcm_data(const boost::system::error_code& error, size_t transferred_bytes) {

      if (!error) {
        // rtcm data received
        resetDataReceptionTimeoutTimer();
        bool chunked_mode = transfer_encoding_chunked_;
        // copy response_ to string
        std::istream response_stream(&response_);
        std::istreambuf_iterator<char> eos;
        std::string readBuf(std::istreambuf_iterator<char>(response_stream), eos);
        std::cout << "handle_read_rtcm_data with bytes in size: " << transferred_bytes << std::endl;
        if (chunked_mode) {
          static std::string parseBuf;
          static unsigned int chunksize = 0;
          static bool expectDataLine = false;

          // copy readBuf to parseBuf
          parseBuf += readBuf;
          while (!parseBuf.empty()) {

            if (!expectDataLine) {
              // expecting line with chunk size as hexidecimal number as string

              // find next linebreak
              size_t linebreakPos = parseBuf.find("\r\n");
              if (linebreakPos != std::string::npos) {
                // line break found, copy characters in front of line break to line
                std::string line = parseBuf.substr(0,linebreakPos);
                // delete line including line break from parseBuf
                parseBuf = parseBuf.substr(linebreakPos+2,parseBuf.size()-(linebreakPos+2));

                // parse hexadecimal number in line
                chunksize = 0;
                if (!line.empty() && line.find_first_not_of("0123456789ABCDEFabcdef") == std::string::npos) {
                  // line contains only valid characters for hexadecimal number
                  unsigned int intvalue;
                  std::stringstream ss;
                  ss << std::hex << line;
                  ss >> intvalue;
                  if (intvalue > 0) {
                    chunksize = intvalue; // expected number of characters in next line
                    expectDataLine = true; // next chunk data line
                  }
                }
                if (!chunksize) {
                  std::stringstream errormsgstream;
                  errormsgstream << "NtripCom discarding line, not a hexadecimal number:" << line;
                  change_status(nc_error,errormsgstream.str());
                  // next expect chunk size line again
                }
              } else {
                // no line break found, line will be completed in next read
                break; // break while loop
              }
            } else { // expecting line with chunk data

              if (parseBuf.size() >= chunksize + 2) { // enough characters in parseBuf for chunk including trailing line break

                // copy chunk characters to chunk
                std::string chunk = parseBuf.substr(0,chunksize);
                // copy possible line break after chunk
                std::string afterChunk = parseBuf.substr(chunksize,2);
                // delete chunk characters including possible line break from parseBuf
                parseBuf = parseBuf.substr(chunksize+2,parseBuf.size()-(chunksize+2));

                if (afterChunk == "\r\n") {
                  // there is a line break after the chunk --> chunk is valid

                  // send on_RTCM_bytes signal
                  boost::shared_ptr<uint8_t[]> data(new uint8_t[chunksize]);
                  memcpy(data.get(),chunk.c_str(),chunksize);
                  rtcm_bytes_signal_(data, chunksize);

                  //stringstream statusstream;
                  //statusstream << "RTCM " << chunksize << " bytes";
                  //change_status(nc_status,statusstream.str());

                  expectDataLine = false; // next: chunk size line
                } else {
                  std::stringstream errormsgstream;
                  errormsgstream << "NtripCom discarding chunk, chunk not terminated by linebreak (chunksize" << chunksize << ")";
                  change_status(nc_error,errormsgstream.str());
                  expectDataLine = false; // next: chunk size line
                }
              } else {
                // chunk will be completed in next read
                break; // break while loop
              }
            }
          }
        } else {
          // non-chunked mode
          unsigned int num_bytes = readBuf.size();
          boost::shared_ptr<uint8_t[]> data(new uint8_t[num_bytes]);
          memcpy(data.get(), readBuf.c_str(), num_bytes);
          rtcm_bytes_signal_(data, num_bytes);
        }

        // Continue reading
        boost::asio::async_read(tcp_socket_, response_, boost::asio::transfer_at_least(1),
                                boost::bind(&NtripCom::handle_read_rtcm_data, this, boost::asio::placeholders::error, boost::placeholders::_2));
      } else if (error != boost::asio::error::operation_aborted) {
        std::stringstream errormsgstream;
        errormsgstream << "RTCM data reading error: " << error.message();
        change_status(nc_error,errormsgstream.str());
        resetRetryTimer();
      }
    }

    void NtripCom::set_own_position(NMEA_GGA_ref_t GGA_ref) {
      std::stringstream GGA_ss;
      GGA_ss << "GPGGA,";

      // UTC Time: HHMMSS.SS
      GGA_ss << std::setfill('0') << std::setw(2) << GGA_ref.utc_hours_since_midnight
             << std::setfill('0') << std::setw(2) << GGA_ref.utc_minutes_since_midnight
             << std::setfill('0') << std::setw(4) << std::fixed << std::setprecision(2) << GGA_ref.utc_seconds_since_midnight << ",";
      // Latitude
      int latsign = (GGA_ref.latitude_deg >= 0) ? 1 : -1;
      int lat = floor(latsign*GGA_ref.latitude_deg);
      double latmin = (latsign*GGA_ref.latitude_deg-lat*1.0)*60.0;
      GGA_ss << std::setfill('0') << std::setw(2) << lat
             << std::setfill('0') << std::setw(11) << std::fixed << std::setprecision(8) << latmin << ",";
      if (latsign >= 0) {
        GGA_ss << "N,";
      } else {
        GGA_ss << "S,";
      }
      // Longitude
      int lonsign = (GGA_ref.longitude_deg >= 0) ? 1 : -1;
      int lon = floor(lonsign * GGA_ref.longitude_deg);
      double lonmin = (lonsign * GGA_ref.longitude_deg - lon * 1.0) * 60.0;
      GGA_ss << std::setfill('0') << std::setw(3) << lon
             << std::setfill('0') << std::setw(11) << std::fixed << std::setprecision(8) << lonmin << ",";
      if (lonsign >= 0) {
        GGA_ss << "E,";
      } else {
        GGA_ss << "W,";
      }
      // ition fix indicator, Satellites used, HDOP
      GGA_ss << "1,8,01.0,";
      // MSL Altitude meter
      GGA_ss << std::fixed << std::setprecision(2) << GGA_ref.altitude_msl_m << ",M,";
      // geoid separation, Age of Differential Corrections, Diff. Reference Station ID
      GGA_ss << ",M,,0";

      nmea_request_message_ = "$" + GGA_ss.str() + "*" + NMEAchecksum(GGA_ss.str()) + "\r\n";
      // if connection established, send message
      if (status_.connected)  {
        //send_nmea_request_message();
      }
    }

    void NtripCom::send_nmea_request_message() {
      if (!nmea_request_message_.empty()) {
        boost::asio::streambuf nmea_request;
        std::ostream nmea_request_stream(&nmea_request);
        nmea_request_stream << nmea_request_message_;
        //change_status(nc_status,"Sending NMEA Position to Ntrip");
        RCLCPP_INFO_STREAM(node_.get_logger(), "Sending NMEA to NtripServer: " << nmea_request_message_);
        boost::asio::async_write(tcp_socket_, nmea_request,
                                 boost::bind(&NtripCom::handle_nmea_write_request, this, boost::asio::placeholders::error, boost::placeholders::_2));

      }
    }

    std::string NtripCom::NMEAchecksum(const std::string& gga_part) {
      // nmea checksum: xor of all characters between $ and *
      char checksum = 0;
      for (char i : gga_part) {
        checksum ^= i;
      }
      char char1_val = (checksum & 0xF0) >> 4;
      char char2_val = checksum & 0x0F;
      std::stringstream char12;
      char12 << (char)((char1_val <= 9) ? (char1_val + '0') : (char1_val - 10 + 'A'))
             << (char)((char2_val <= 9) ? (char2_val + '0') : (char2_val - 10 + 'A'));
      return char12.str();
    }

    void NtripCom::handle_nmea_write_request(const boost::system::error_code& error, size_t bytes_transferred) {
      if (!error) {
        // Read the response_ status line.
        change_status(nc_status,"NMEA message sent to Ntrip server");
      } else if (error != boost::asio::error::operation_aborted) {
        std::stringstream errormsgstream;
        errormsgstream << "NMEA write error: " << error.message();
        change_status(nc_error,errormsgstream.str());
      }
    }

    void NtripCom::change_status(statustype_t statusType, const std::string& statusmsg) {

      if (statusType == nc_error) 
      {
        std::cerr << "On Ntrip error: " << statusmsg << std::endl;
        stop();
        std::cerr << "On Ntrip error: Socket disconnected !" << std::endl;
        init();
        std::cerr << "On Ntrip error: New connection initialized!" << std::endl;
      }
      //else cout << "Ntrip status: " << statusmsg << endl;
      status_.type = statusType;
      status_.message = statusmsg;
      on_status_signal_(status_);
      RCLCPP_INFO_STREAM(node_.get_logger(), "NtripClient Status: " << statusmsg);
    }

    NtripCom::status_t NtripCom::get_status() {
      return status_;
    }

    void NtripCom::resetRetryTimer() {
      disconnect();
      retry_timer_->cancel();
      retry_timer_->expires_from_now(boost::posix_time::seconds(1));
      retry_timer_->async_wait(boost::bind(&NtripCom::onRetryTimer,this,boost::placeholders::_1));
    }

    void NtripCom::onRetryTimer(const boost::system::error_code& error) {
      if (error != boost::asio::error::operation_aborted) {
        init();
      }
    }

    void NtripCom::resetDataReceptionTimeoutTimer() {
      data_reception_timeout_timer_->cancel();
      data_reception_timeout_timer_->expires_from_now(boost::posix_time::seconds(dataReceptionTimeoutSeconds));
      data_reception_timeout_timer_->async_wait(boost::bind(&NtripCom::onDataReceptionTimeoutTimer, this, boost::placeholders::_1));
    }

    void NtripCom::onDataReceptionTimeoutTimer(const boost::system::error_code& error) {
      if (!error || error != boost::asio::error::operation_aborted) {
        if (status_.connected) {
          std::stringstream errormsgstream;
          errormsgstream << "RTCM data reception time out";
          change_status(nc_error,errormsgstream.str());
          resetRetryTimer();
        }
      }
    }

// source: http://www.adp-gmbh.ch/cpp/common/base64.html
    std::string NtripCom::base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len) {
      std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
      std::string ret;
      int i = 0;
      int j = 0;
      unsigned char char_array_3[3];
      unsigned char char_array_4[4];

      while (in_len--) {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3) {
          char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
          char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
          char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
          char_array_4[3] = char_array_3[2] & 0x3f;

          for(i = 0; (i <4) ; i++)
            ret += base64_chars[char_array_4[i]];
          i = 0;
        }
      }
      if (i)
      {
        for(j = i; j < 3; j++)
          char_array_3[j] = '\0';
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++)
          ret += base64_chars[char_array_4[j]];

        while((i++ < 3))
          ret += '=';
      }
      return ret;
    }
}







