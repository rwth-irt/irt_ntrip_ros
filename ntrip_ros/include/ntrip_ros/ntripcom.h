/*
 * ntripcom.h
 *
 *  Created on: 15.06.2018
 *      Author: Eg: inital version
 *				Kd: added non-chunked mode, some bug fixes
 *				Zh: optimization
 */

#ifndef _NTRIPCOM_
#define _NTRIPCOM_

#include <string>
#include <boost/asio.hpp>
#include <boost/signals2.hpp>
#include <rclcpp/rclcpp.hpp>

// source for TCP client: http://www.boost.org/doc/libs/1_36_0/doc/html/boost_asio/example/http/client/async_client.cpp

namespace ntrip
{
    class NtripCom {
    public:
        typedef struct {
            std::string   server;
            unsigned int  port;
            std::string   mountpoint;
            std::string   user;
            std::string   password;
            //std::string nmea = "";
        } settings_t;

        typedef struct  {
            double        latitude_deg;
            double        longitude_deg;
            double        altitude_msl_m;
            long 	      utc_hours_since_midnight;
            long 	      utc_minutes_since_midnight;
            double 	      utc_seconds_since_midnight;
        } NMEA_GGA_ref_t;

        typedef enum { nc_status = 0, nc_error = 1 } statustype_t;

        typedef struct {
            bool connected;
            statustype_t type;
            std::string message;
        } status_t;


        typedef boost::signals2::signal<void(boost::shared_ptr<unsigned char[]> bytes, size_t no_of_bytes)> rtcm_bytes_signal_t;
        typedef boost::signals2::signal<void(NtripCom::status_t status)> on_status_signal_t;
        typedef boost::signals2::signal<void()> on_stop_signal_t;

        explicit NtripCom(boost::asio::io_service& io_service1, rclcpp::Node& node);

        boost::signals2::connection on_RTCM_bytes(const rtcm_bytes_signal_t::slot_type &subscriber);
        boost::signals2::connection on_status_change(const on_status_signal_t::slot_type &subscriber);
        boost::signals2::connection on_stop(const on_stop_signal_t::slot_type &subscriber);
        status_t get_status();

        void start(const settings_t& settings);

        void update_reference_station(const settings_t& settings);

        // send own position as NMEA GGA message (only for VRS etc.)
        void set_own_position(NMEA_GGA_ref_t GGA_ref);
        void stop();

    private:
        rclcpp::Node &node_;
        settings_t settings_;
        boost::asio::io_service& io_service_;
        boost::asio::ip::tcp::endpoint remote_server_endpoint_;
        boost::asio::ip::tcp::resolver resolver_;
        boost::asio::ip::tcp::socket tcp_socket_;
        std::unique_ptr<boost::asio::deadline_timer> retry_timer_;
        std::unique_ptr<boost::asio::deadline_timer> data_reception_timeout_timer_;

        rtcm_bytes_signal_t rtcm_bytes_signal_;
        on_status_signal_t on_status_signal_;
        on_stop_signal_t on_stop_signal_;
        boost::asio::streambuf request_;
        boost::asio::streambuf response_;
        std::string nmea_request_message_;
        status_t status_;

        bool transfer_encoding_chunked_{};

        void init();
        void disconnect();

        void handle_resolve(const boost::system::error_code& error, boost::asio::ip::tcp::resolver::iterator endpoint_iterator);
        void handle_connect(const boost::system::error_code& error, boost::asio::ip::tcp::resolver::iterator endpoint_iterator);
        void handle_write_request(const boost::system::error_code& error, std::size_t bytes_transferred);
        void handle_read_status_line(const boost::system::error_code& error);
        void handle_read_headers(const boost::system::error_code& error);
        void handle_read_rtcm_data(const boost::system::error_code& error, size_t transferred_bytes);
        void handle_nmea_write_request(const boost::system::error_code& error, size_t bytes_transferred);

        void change_status(statustype_t statusType, const std::string& statusmsg);

        void resetRetryTimer();
        void onRetryTimer(const boost::system::error_code& e);
        void resetDataReceptionTimeoutTimer();
        void onDataReceptionTimeoutTimer(const boost::system::error_code& error);

        void send_nmea_request_message();
        static std::string NMEAchecksum(const std::string& gga_part);
        static std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len);
    };
}
#endif /* _NTRIPCOM_ */
