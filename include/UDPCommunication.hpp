#ifndef ECU_COMMUNICATION_UDPSERVER_HPP
#define ECU_COMMUNICATION_UDPSERVER_HPP
#define BUFFERSIZE 2048

#include <stdlib.h>
#include <sys/socket.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdint.h>
#include <string>

namespace ecu_communication {

enum class udp_server_error_type {
    none = 0,
    get_sockfd_error = 1,
    bind_error = 2,
    recv_error = 3,
    send_error = 4
};

struct udp_communication_params_type {
    std::string local_ip;
    uint16_t local_port;
    std::string remote_ip;
    uint16_t remote_port;
};

class UDPCommunication {
public:
    udp_communication_params_type params;

    udp_server_error_type error;
    UDPCommunication();
    bool init();
    intmax_t recv();


    const char* getClientIP();
    intmax_t get_recv_len();

public:
    int sockfd;
    sockaddr_in local_addr;
    sockaddr_in remote_addr;
    sockaddr_in client_addr;
    intmax_t recv_len;
    intmax_t send_len;
    socklen_t client_ip_len;
    uint8_t buffer[BUFFERSIZE];


    bool sendToClient(uint8_t *buffer, size_t send_len);
    bool sendToRemote(uint8_t *buffer, size_t send_len);
};

}

#endif