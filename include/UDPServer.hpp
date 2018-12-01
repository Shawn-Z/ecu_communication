#ifndef ECU_COMMUNICATION_UDPSERVER_HPP
#define ECU_COMMUNICATION_UDPSERVER_HPP

#include <stdlib.h>
#include <sys/socket.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace ecu_communication {

enum class udp_server_error_type {
    none = 0,
    get_server_sockfd_error = 1,
    bind_error = 2,
    recv_from_error = 3
};

class UDPServer {
public:
    udp_server_error_type error;
    UDPServer();
    bool init(uint16_t port);
    bool process(uint8_t *buffer, int64_t buffer_size);
    const char* getClientIP();
    int64_t get_recv_len();

private:
    int server_sockfd_;
    sockaddr_in server_ip_;
    sockaddr_in client_ip_;
    int64_t recv_len_;
    socklen_t client_ip_len_;
};

}

#endif