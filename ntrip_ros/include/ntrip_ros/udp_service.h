//
// Created by haoming on 08.02.23.
//

#ifndef NTRIP_ROS_UDP_SERVICE_H
#define NTRIP_ROS_UDP_SERVICE_H

#pragma once

#include "config.h"

#include <map>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <cstdlib>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include "iomanip"
#include <boost/shared_ptr.hpp>
#include <utility>


const char DEST_IP_ADDRESS[] = "192.168.31.12";//"192.168.8.130";//"134.130.45.90";
const int DEST_PORT = 25000;

using namespace std;


class UDP_Service
{
public:
    explicit UDP_Service() { sock_fd = socket(AF_INET, SOCK_DGRAM, 0);};
    explicit UDP_Service(std::string dest_ip, int dest_port);
    int sock_fd = -1;
    void Start_UDP();
    void do_Send(const boost::shared_ptr<uint8_t>& msg, size_t size);
    void End_UDP() const;

    void set_address(std::string dest_ip, int dest_port)
    {
        dest_ip_add_ = std::move(dest_ip);
        dest_port_ = dest_port;
    }

private:

    std::string dest_ip_add_;
    int dest_port_ = 0;

    int send_num{};
    int len{};
    struct sockaddr_in addr_serv{};
};

#endif //NTRIP_ROS_UDP_SERVICE_H
