//
// Created by haoming on 08.02.23.
//

#include "udp_service.h"

#include <utility>

using namespace std;

//Constructor
UDP_Service:: UDP_Service(std::string dest_ip, int dest_port) : dest_ip_add_(dest_ip), dest_port_(dest_port)
{
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
}


void UDP_Service::Start_UDP()
{
    std::cout<<"Start_UDP" << std::endl;
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        std::cout<<"Socket error wth code: "<< sock_fd << std::endl;
        //exit(1);
    }
    else
    {
        std::cout<<"Socket successfully!"<<std::endl;
    }
}

void UDP_Service::do_Send(const boost::shared_ptr<uint8_t>& msg, size_t size)
{
    len = sizeof(addr_serv);
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_addr.s_addr = inet_addr(dest_ip_add_.c_str());
    addr_serv.sin_port = htons(dest_port_);

    send_num = sendto(sock_fd, msg.get(), size, 0, (struct sockaddr *)&addr_serv, len);

    if(send_num < 0)
    {
        perror("Sendto Error:");
        //exit(1);
    }
    else
    {
        switch (size)
        {
            case Length_1004:
                std::cout<<"Messages Struct <RTCM_30_1004> are successfully sent!"<<std::endl;
                break;

            case Length_1005:
                std::cout<<"Messages Struct <RTCM_30_1005> are successfully sent!"<<std::endl;
                break;

            case Length_DD_4_1004:
                std::cout<<"Messages Struct <Data_Dictionary_4_0_1004> are successfully sent!"<<std::endl;
                break;

            case Length_DD_4_1005:
                std::cout<<"Messages Struct <Data_Dictionary_4_0_1005> are successfully sent!"<<std::endl;
                break;

            case Length_DD_RTCM_3_3_L1E1:
                std::cout<<"Messages Struct <Data_Dictionary_RTCM_3_3_L1_E1> are successfully sent!"<<std::endl;
                break;
        }
        std::cout<<"How many Bytes are sent:"<<send_num<<" Bytes."<<std::endl;
    }
}

void UDP_Service::End_UDP() const
{
    close(sock_fd);
}

